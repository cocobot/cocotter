mod config;
use ble::{run_ble_with_central, BleScanResult, RomePeripheral};
use vlx::common::VlxSensor;

use std::{thread, time::Duration};
use config::PAMIConfig;

#[cfg(target_os = "espidf")]    
use board_pami::BoardPami;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardPami;


fn main() {
    let mut board = BoardPami::new();

    //if we panic here, it means the board is not configured. 
    //should be fixed, robot will not work.
    let config = match PAMIConfig::get_config() {
        Some(config) => {
            log::info!("Board id {} is configured with color {:?}", config.id, config.color);
            config
        },
        None => {
            panic!("Board is not configured (mac address is {:?})", PAMIConfig::get_mac_address());
        }
    };

    fn scan_dump(scan_result: BleScanResult) {
        if let Some(name) = scan_result.local_name() {
            log::info!("SCAN: addr: {}, name: {}", scan_result.addr, name);
        } else {
            log::info!("SCAN: addr: {}", scan_result.addr);
        }
    }

    let device_name = format!("PAMI {}", config.id);
    let (ble_server, ble_client) = run_ble_with_central(board.ble.take().unwrap(), scan_dump);
    let (rome_tx, rome_rx) = RomePeripheral::run(ble_server, device_name);
    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

    // ble_client.start_scanning(10).unwrap();

    log::info!("Initializing VLX sensors...");
    let mut vlx = board.init_vlx_sensors().unwrap();

    std::thread::spawn(move || {
        log::info!("Start BLE RX thread");
        loop {
            if let Ok(data) = rome_rx.recv_timeout(std::time::Duration::from_millis(500)) {
                log::info!("BLE received data: {:?}", data);
            }
        }
    });

    let mut buttons = board.buttons.take().unwrap();

    log::info!("Start BLE TX dummy main loop");
    let mut tick = 0u32;
    loop {
        led_heartbeat.toggle().ok();
        let data = format!("tx-{tick}");

        log::debug!("BLE send data: {:?}", data);
        if let Err(err) = rome_tx.send(data.as_bytes().into()) {
            log::error!("BLE send error: {:?}", err);
        }

        // Print VLX sensor distance (only back sensor is working)
        if let Ok(distance) = vlx.back.get_distance() {
            log::info!("VLX sensor distance: {:?}", distance);
        } else {
            log::error!("Failed to get VLX sensor distance");
        }

        let buttons_state = buttons.read_inputs();
        log::info!("Buttons: {buttons_state:?}");

        tick += 1;
        thread::sleep(Duration::from_millis(2000));
    }
}
