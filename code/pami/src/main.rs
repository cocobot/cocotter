use std::time::{Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use asserv::differential::{conf::*, Asserv};
use asserv::{rome::AsservRome, differential::rome::AsservDiffRome};
use board_common::Periodicity;
use board_pami::{BatteryLevel, PamiBoard};
use vlx::VlxSensor;
use pami::config::PamiConfig;
use pami::events::*;
use pami::pami_asserv::{ASSERV_PERIOD, PamiAsservHardware};
use pami::ui::Ui;

#[cfg(target_os = "espidf")]
type PamiBoardImpl = board_pami::EspPamiBoard<'static>;
#[cfg(not(target_os = "espidf"))]
use board_pami::MockPamiBoard as PamiBoardImpl;

#[cfg(target_os = "espidf")]
fn flush_display(display: &mut board_pami::esp::PamiDisplay) {
    display.flush().unwrap();
}
#[cfg(not(target_os = "espidf"))]
fn flush_display(_display: &mut board_pami::mock::PamiDisplay) {}


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
    let (ui_events, ui_triggers) = Ui::run_thread(config.name, board.display().unwrap(), flush_display);

    // Use a "enabled" flag to disable during a match
    let passkey_enabled = Arc::new(AtomicBool::new(true));
    let passkey_notifier = {
        let passkey_enabled = passkey_enabled.clone();
        let ui_events = ui_events.clone();
        move |_addr, key| {
            if passkey_enabled.load(Ordering::Relaxed) {
                ui_events.send(UiEvent::KeypassNotif(key)).unwrap();
            }
        }
    };
    let (rome_tx, rome_rx) = board.rome(config.name.into(), passkey_notifier).unwrap();

    let mut vlx = board.vlx_sensor().unwrap();
    log::info!("Initialize VLX");
    if let Err(err) = vlx.init() {
        log::error!("VLX initalization failed: {err:?}");
    }

    let mut buttons = board.buttons().unwrap();
    let mut battery_level = board.battery_level().unwrap();

    log::info!("Start BLE TX dummy main loop");

    let mut asserv = Asserv::new(PamiAsservHardware::new(&mut board), ASSERV_PERIOD);
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
    });

    let mut asserv_period = Periodicity::new(ASSERV_PERIOD);
    let mut asserv_tm_period = Periodicity::new(Duration::from_millis(500)); //TODO large value for debug
    let mut buttons_period = Periodicity::new(Duration::from_millis(200));
    let mut battery_period = Periodicity::new(Duration::from_millis(2000));
    let mut vlx_period = Periodicity::new(Duration::from_millis(2000));

    let mut starting_cord_read = board.starting_cord().unwrap();

    // Keep track of last state, to avoid useless UI updates
    let mut battery_percent = 0;
    let mut button_state = buttons.read_state();
    let mut emergency_stop_state = asserv.hardware_mut().emergency_stop_active();
    let mut starting_cord_state = (starting_cord_read)();
    let (mut team, mut start_delay, mut pami_role) = {
        let switches = button_state.switches();
        (
            match switches[0] {
                false => Team::Left,
                true => Team::Right,
            },
            match switches[1] {
                false => 90,
                true => 3,
            },
            match switches[2] {
                false => PamiRole::Granary,
                true => PamiRole::Land,
            },
        )
    };
    let mut main_step = MainStep::Free;

    // Initialize state and UI state
    ui_events.send(UiEvent::ChangeTeam(team)).unwrap();
    ui_events.send(UiEvent::ChangeStartDelay(start_delay)).unwrap();
    ui_events.send(UiEvent::ChangeRole(pami_role)).unwrap();
    ui_events.send(UiEvent::EmergencyStop(emergency_stop_state)).unwrap();

    loop {
        let now = Instant::now();

        // Starting cord: check whenever possible
        //TODO Add anti-rebound?
        {
            let new_state = (starting_cord_read)();
            if new_state != starting_cord_state {
                log::info!("Starting cord state: {}", new_state);
                starting_cord_state = new_state;
                match (main_step, new_state) {
                    (MainStep::Free, false) => {
                        // Initial unplugging, no change
                    }
                    (MainStep::Free, true) => {
                        // Starting cord plugging, preparing for match
                        log::warn!("Ready for match, remove starting cord to start");
                        main_step = MainStep::CordPlugged;
                        // Note: it's not possible to disable BLE advertising.
                        // It would require to make RomePeripheral available for non-ESP target,
                        // and update the board interface.
                        passkey_enabled.store(false, Ordering::Relaxed);
                    }
                    (MainStep::CordPlugged, false) => {
                        log::warn!("Start match, team {}, delay {}s, role {:?}", team.name(), start_delay, pami_role);
                        main_step = MainStep::Match;
                        //TODO Match starts! Trigger events (UI, etc.)
                    }
                    (MainStep::CordPlugged, true) => {
                        unreachable!();
                    }
                    (MainStep::Match, _) => {
                        // Ignore changes during a match
                    }
                }
            }
        }

        // Process ROME input messages
        for data in rome_rx.try_iter() {
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !asserv.on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }

        if asserv_period.update(&now) {
            asserv.update(asserv_period.period());
        }

        if asserv_tm_period.update(&now) {
            if let Err(err) = rome_tx.send(asserv.asserv_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Err(err) = rome_tx.send(asserv.asserv_diff_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
        }

        // VLX capture
        if vlx_period.update(&now) {
            match vlx.get_distance() {
                Ok(distance) => log::info!("VLX sensor distance: {distance:?}"),
                Err(err) => log::error!("Failed to get VLX sensor distance: {err:?}"),
            }
        }

        // Buttons: dpad, switches, emergency stop
        if buttons_period.update(&now) {
            let new_state = buttons.read_state();
            if new_state != button_state {
                let dpad = new_state.dpad();
                if dpad != button_state.dpad() {
                    ui_events.send(UiEvent::Dpad(dpad)).unwrap();
                }
                // Note: don't detect switch changes at runtime, only at startup
                button_state = new_state;
            }

            let new_stop = asserv.hardware_mut().emergency_stop_active();
            if new_stop != emergency_stop_state {
                emergency_stop_state = new_stop;
                log::info!("Emergency stop changed: {new_stop}");
                ui_events.send(UiEvent::EmergencyStop(new_stop)).unwrap();
            }
        }

        // Battery voltage
        if battery_period.update(&now) {
            let new_vbatt = battery_level.read_vbatt();
            let (vbatt_mv, vbatt_pct) = new_vbatt;
            if let Err(err) = rome_tx.send(rome::Message::BatteryLevel { mv: vbatt_mv, percent: vbatt_pct }.encode()) {
                log::error!("BLE send error: {:?}", err);
            }
            if vbatt_pct != battery_percent {
                battery_percent = vbatt_pct;
                log::info!("Battery: {vbatt_mv} mV, {vbatt_pct}%");
                ui_events.send(UiEvent::Battery { percent: vbatt_pct }).unwrap();
            }
        }

        // Process UI events
        while let Ok(trigger) = ui_triggers.try_recv() {
            match trigger {
                UiTrigger::ChangeTeam(new) => {
                    team = new;
                    ui_events.send(UiEvent::ChangeTeam(new)).unwrap();
                }
                UiTrigger::ChangeStartDelay(new) => {
                    start_delay = new;
                    ui_events.send(UiEvent::ChangeStartDelay(new)).unwrap();
                }
                UiTrigger::ChangeRole(new) => {
                    pami_role = new;
                    ui_events.send(UiEvent::ChangeRole(new)).unwrap();
                }
            }
        }

        if let Some(duration) = asserv_period.next().checked_duration_since(now) {
            std::thread::sleep(duration);
        }
    }
}

