use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::net::TcpStream;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;
use log::{info, warn, error};
use crate::FakeBle;

pub struct BleComm;

impl BleComm {
    pub fn run(_ble: FakeBle, name: &'static str) -> (Sender<Box<[u8]>>, Receiver<Box<[u8]>>) {
        let (rome_tx, rome_tx_receiver): (Sender<Box<[u8]>>, Receiver<Box<[u8]>>) = mpsc::channel();
        let (rome_rx_sender, rome_rx): (Sender<Box<[u8]>>, Receiver<Box<[u8]>>) = mpsc::channel();

        // Get Rome server address from environment variable or use default
        let rome_addr = std::env::var("ROME_SERVER")
            .unwrap_or_else(|_| "127.0.0.1:7113".to_string());

        // Spawn TCP connection thread
        thread::spawn(move || {
            info!("Starting Rome client for '{}'", name);
            info!("Rome server address: {}", rome_addr);

            loop {
                match TcpStream::connect(&rome_addr) {
                    Ok(mut stream) => {
                        info!("Connected to Rome server at {}", rome_addr);

                        // Disable Nagle's algorithm for low latency
                        stream.set_nodelay(true).expect("Failed to set TCP_NODELAY");

                        // Set read timeout to allow checking for outgoing data
                        stream.set_read_timeout(Some(Duration::from_millis(100))).ok();

                        let mut buffer = [0u8; 1024];

                        loop {
                            // Check for data to send
                            match rome_tx_receiver.try_recv() {
                                Ok(data) => {
                                    if let Err(e) = stream.write_all(&data) {
                                        error!("Failed to send data: {:?}", e);
                                        break;
                                    }
                                    if let Err(e) = stream.flush() {
                                        error!("Failed to flush stream: {:?}", e);
                                        break;
                                    }
                                }
                                Err(TryRecvError::Empty) => {}
                                Err(TryRecvError::Disconnected) => {
                                    error!("rome_tx channel disconnected");
                                    return;
                                }
                            }

                            // Try to read incoming data
                            match stream.read(&mut buffer) {
                                Ok(0) => {
                                    // Connection closed
                                    warn!("Rome server closed connection");
                                    break;
                                }
                                Ok(n) => {
                                    // Send received data to rome_rx channel
                                    let data = buffer[..n].to_vec();
                                    if rome_rx_sender.send(data.into_boxed_slice()).is_err() {
                                        error!("Failed to send to rome_rx channel");
                                        return;
                                    }
                                }
                                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock ||
                                            e.kind() == std::io::ErrorKind::TimedOut => {
                                    // No data available, continue
                                }
                                Err(e) => {
                                    error!("Failed to read from Rome server: {:?}", e);
                                    break;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to connect to Rome server at {}: {:?}", rome_addr, e);
                    }
                }

                // Wait before reconnecting
                thread::sleep(Duration::from_secs(2));
                info!("Attempting to reconnect to Rome server...");
            }
        });

        (rome_tx, rome_rx)
    }
}
