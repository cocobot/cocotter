mod config;

use std::{thread, time::Duration};
use config::PAMIConfig;

#[cfg(target_os = "espidf")]    
use board_pami::BoardPami;

#[cfg(target_os = "linux")]
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

    //set the color of the led to the color of the board
    log::info!("Setting led color to {:?}", config.color);

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();
    loop {
        led_heartbeat.toggle().ok();
        thread::sleep(Duration::from_millis(500));
    }
}
