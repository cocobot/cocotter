mod config;
mod ui;

use std::{thread, time::Duration};
use vlx::VlxSensor;
use board_pami::{PamiBoard, PamiButtonsState, Vbatt};
use config::PamiConfig;
use ui::UiEvent;

#[cfg(target_os = "espidf")]
use board_pami::EspPamiBoard as PamiBoardImpl;
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

    std::thread::spawn(move || {
        log::info!("Start BLE RX thread");
        loop {
            if let Ok(data) = rome_rx.recv_timeout(std::time::Duration::from_millis(500)) {
                match rome::Message::decode(&data) {
                    Ok(msg) => log::info!("ROME RX: {msg:?}"),
                    Err(err) => log::error!("ROME RX error: {err:?}"),
                }
            }
        }
    });

    let mut buttons = board.buttons().unwrap();
    let mut vbatt = board.vbatt().unwrap();

    log::info!("Start BLE TX dummy main loop");
    let mut tick = 0u32;
    const PERIOD: Duration = Duration::from_millis(100);

    let mut button_state = PamiButtonsState(0);

    loop {
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
        if tick % 10 == 0 {
            match vlx.get_distance() {
                Ok(distance) => log::info!("VLX sensor distance: {distance:?}"),
                Err(err) => log::error!("Failed to get VLX sensor distance: {err:?}"),
            }
        }

        // Dpad buttons
        if tick % 2 == 0 {
            let new_state = buttons.read_state();
            if new_state != button_state {
                let dpad = new_state.dpad();
                ui_queue.send(UiEvent::Dpad(dpad)).unwrap();
                log::info!("New buttons state: {}  {:08b}", dpad.as_char(), new_state.0);
                button_state = new_state;
            }
        }

        // Battery voltage
        if tick % 50 == 0 {
            let (vbatt_mv, vbatt_pct) = vbatt.read_vbatt();
            log::info!("Battery: {vbatt_mv} mV, {vbatt_pct}%");
            ui_queue.send(UiEvent::Battery { percent: vbatt_pct }).unwrap();
            if let Err(err) = rome_tx.send(rome::Message::BatteryLevel { mv: vbatt_mv, percent: vbatt_pct }.encode()) {
                log::error!("BLE send error: {:?}", err);
            }
        }

        tick += 1;
        thread::sleep(PERIOD);
    }
}
