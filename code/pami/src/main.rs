mod config;
mod events;
mod pami_asserv;
mod ui;

use std::time::{Duration, Instant};
use asserv::differential::conf::*;
use asserv::{differential::Asserv, maths::XYA};
use board_pami::{PamiBoard, PamiButtonsState, Vbatt};
use vlx::VlxSensor;
use config::PamiConfig;
use crate::pami_asserv::{ASSERV_PERIOD, PamiAsservHardware};
use crate::events::{AsservOrder, Periodicity, UiEvent};

#[cfg(target_os = "espidf")]
type PamiBoardImpl = board_pami::EspPamiBoard<'static>;
#[cfg(not(target_os = "espidf"))]
use board_pami::MockPamiBoard as PamiBoardImpl;


fn main() {
    let mut board = PamiBoardImpl::init();

    let mac_addr = board.bt_mac_address();
    let config = if let Some(config) = PamiConfig::by_bt_mac(&mac_addr) {
        log::info!("Board '{}' with color {:?}", config.name, config.color);
        config
    } else {
        panic!("Board is not configured (MAC address: {mac_addr:?})");
    };

    log::info!("Start UI thread");
    let ui_queue = ui::Ui::run(board.display().unwrap());

    let passkey_notifier = {
        let ui_queue = ui_queue.clone();
        move |_addr, key| { ui_queue.send(UiEvent::KeypassNotif(key)).unwrap(); }
    };
    let (rome_tx, rome_rx) = board.rome(config.name.into(), passkey_notifier).unwrap();

    let mut vlx = board.vlx_sensor().unwrap();
    log::info!("Initialiaze VLX");
    if let Err(err) = vlx.init() {
        log::error!("VLX initalization failed: {err:?}");
    }

    let (asserv_order_send, asserv_order_recv) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        log::info!("Start BLE RX thread");
        loop {
            if let Ok(data) = rome_rx.recv_timeout(std::time::Duration::from_millis(500)) {
                match rome::Message::decode(&data) {
                    Ok(rome::Message::AsservGotoXy { x, y }) => {
                        log::info!("ROME: goto_xy: {x},{y}");
                        asserv_order_send.send(AsservOrder::GotoXy(x, y)).unwrap();
                    }
                    Ok(rome::Message::AsservGotoA { a }) => {
                        log::info!("ROME: goto_a: {a}");
                        asserv_order_send.send(AsservOrder::GotoA(a)).unwrap();
                    }
                    Ok(rome::Message::AsservGotoXyRel { dx, dy }) => {
                        log::info!("ROME: goto_xy_rel: {dx},{dy}");
                        asserv_order_send.send(AsservOrder::GotoXyRel(dx, dy)).unwrap();
                    }
                    Ok(rome::Message::AsservGotoARel { da }) => {
                        log::info!("ROME: goto_a_rel: {da}");
                        asserv_order_send.send(AsservOrder::GotoARel(da)).unwrap();
                    }
                    Ok(rome::Message::AsservResetPosition { x, y, a }) => {
                        log::info!("ROME: reset_position: {x},{y},{a}");
                        asserv_order_send.send(AsservOrder::ResetPosition(XYA::new(x, y, a))).unwrap();
                    }
                    Ok(msg) => {
                        log::warn!("ROME: ignored message: {}", msg.message_id());
                    }
                    Err(err) => log::error!("ROME RX error: {err:?}"),
                }
            }
        }
    });

    let mut buttons = board.buttons().unwrap();
    let mut vbatt = board.vbatt().unwrap();

    log::info!("Start BLE TX dummy main loop");

    let mut button_state = PamiButtonsState(0);

    let mut asserv = Asserv::new(PamiAsservHardware::new(&mut board));
    asserv.set_conf(AsservConf {
        pid_dist: PidConf {
            gain_p: 10,
            gain_i: 1,
            .. Default::default()
        },
        pid_angle: PidConf {
            gain_p: 200,
            gain_i: 5,
            .. Default::default()
        },
        trajectory: TrajectoryConf {
            a_speed: 30.0,
            a_acc: 100.0,
            xy_speed: 2000.0,
            xy_acc: 1000.0,
            xy_stop_window: 20.0,
            xy_aim_angle_window: 0.05,
            xy_cruise_angle_window: 1.5,
            xy_approach_window: 50.0,
            a_stop_window: 0.03,
            xy_idle_speed: 0.01,
            a_idle_speed: 0.01,
        },
        motors: MotorsConf::from_dimensions(75.0, 30.0, 256),
        update_period: ASSERV_PERIOD,
    });

    let mut asserv_period = Periodicity::new(ASSERV_PERIOD);
    let mut asserv_tm_period = Periodicity::new(Duration::from_millis(500)); //TODO large value for debug
    let mut buttons_period = Periodicity::new(Duration::from_millis(200));
    let mut vbatt_period = Periodicity::new(Duration::from_millis(2000));
    let mut vlx_period = Periodicity::new(Duration::from_millis(2000));

    loop {
        let now = Instant::now();

        // Note: "disconnected" errors are not detected
        while let Ok(order) = asserv_order_recv.try_recv() {
            match order {
                AsservOrder::GotoXy(x, y) => asserv.goto_xy(x, y),
                AsservOrder::GotoA(a) => asserv.goto_a(a),
                AsservOrder::GotoXyRel(dx, dy) => asserv.goto_xy_rel(dx, dy),
                AsservOrder::GotoARel(da) => asserv.goto_a(da),
                AsservOrder::ResetPosition(xya) => asserv.reset_position(xya),
            }
        }

        if asserv_period.update(&now) {
            asserv.update(asserv_period.period());
        }

        if asserv_tm_period.update(&now) {
            let position = asserv.cs.position();
            let message = rome::Message::AsservTelemetry {
                x: position.x,
                y: position.y,
                a: position.a,
                done: asserv.idle(),
            };
            if let Err(err) = rome_tx.send(message.encode()) {
                log::error!("ROME send error: {:?}", err);
            }
        }

        /*
        // Heartbeat + test BLE message
        if tick % 10 == 0 {
            //heartbeat_led.toggle().ok();
            let data = format!("tx-{tick}");
            log::debug!("BLE send data: {:?}", data);
            if let Err(err) = rome_tx.send(data.as_bytes().into()) {
                log::error!("BLE send error: {:?}", err);
            }
        }
        */

        // VLX capture
        if vlx_period.update(&now) {
            match vlx.get_distance() {
                Ok(distance) => log::info!("VLX sensor distance: {distance:?}"),
                Err(err) => log::error!("Failed to get VLX sensor distance: {err:?}"),
            }
        }

        // Dpad buttons
        if buttons_period.update(&now) {
            let new_state = buttons.read_state();
            if new_state != button_state {
                let dpad = new_state.dpad();
                ui_queue.send(UiEvent::Dpad(dpad)).unwrap();
                log::info!("New buttons state: {}  {:08b}", dpad.as_char(), new_state.0);
                button_state = new_state;
            }
        }

        // Battery voltage
        if vbatt_period.update(&now) {
            let (vbatt_mv, vbatt_pct) = vbatt.read_vbatt();
            log::info!("Battery: {vbatt_mv} mV, {vbatt_pct}%");
            ui_queue.send(UiEvent::Battery { percent: vbatt_pct }).unwrap();
            if let Err(err) = rome_tx.send(rome::Message::BatteryLevel { mv: vbatt_mv, percent: vbatt_pct }.encode()) {
                log::error!("BLE send error: {:?}", err);
            }
        }

        if let Some(duration) = asserv_period.next().checked_duration_since(now) {
            std::thread::sleep(duration);
        }
    }
}

