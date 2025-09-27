use std::sync::mpsc::{self, Receiver, Sender};

pub struct BleComm;

impl BleComm {
    pub fn run(_ble: super::FakeBle, name: String) -> (Sender<Box<[u8]>>, Receiver<Box<[u8]>>) {
        log::info!("Starting simulated BLE with name: {}", name);

        let (tx_to_ble, rx_from_app) = mpsc::channel::<Box<[u8]>>();
        let (tx_to_app, rx_from_ble) = mpsc::channel::<Box<[u8]>>();

        std::thread::spawn(move || {
            log::info!("Simulated BLE thread started");
            loop {
                if let Ok(data) = rx_from_app.recv_timeout(std::time::Duration::from_millis(100)) {
                    log::debug!("Simulated BLE received data to send: {:?}", data);
                    // In simulation, just echo back the data
                    let mut response = data.clone();
                    if response.len() > 0 {
                        response[0] = !response[0]; // Invert first byte to show it was processed
                    }
                    let _ = tx_to_app.send(response);
                }
            }
        });

        (tx_to_ble, rx_from_ble)
    }
}