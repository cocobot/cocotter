use std::{thread, time::Duration};

#[cfg(target_os = "espidf")]    
use board_sabotter::BoardSabotter;
#[cfg(target_os = "espidf")]    
use comm::Comm;
#[cfg(target_os = "linux")]
use board_simulator::{BoardSabotter, comm::Comm};


fn main() {
    let mut board = BoardSabotter::new();
    let (rome_tx, rome_rx) = Comm::run(board.ble.take().unwrap(), "Galipeur".to_string());

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

    loop {
        led_heartbeat.toggle().ok();
        thread::sleep(Duration::from_millis(500));

        //test ble
        loop {
            if let Ok(mut data) = rome_rx.recv_timeout(Duration::from_millis(100)) {
                println!("Recv {:?}", data);
                data[0] = 0xFF;
                rome_tx.send(data).unwrap();
            }
            else {
                break;
            }
        }
    }
}
