use std::{thread, time::Duration};

#[cfg(target_os = "espidf")]    
use board_sabotter::BoardSabotter;

#[cfg(target_os = "linux")]
use board_simulator::BoardSabotter;

fn main() {
    let mut board = BoardSabotter::new();

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

    loop {
        led_heartbeat.toggle().ok();
        thread::sleep(Duration::from_millis(500));
    }
}
