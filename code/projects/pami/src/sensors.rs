use board_pami_2023::VlxType;

use crate::events::{Event, EventSystem};


pub struct Sensors {
    tof: VlxType,
    event: EventSystem,
}

impl Sensors {
    pub fn new(device: VlxType, event: &EventSystem) {
        let instance = Sensors {
            tof: device,
            event: event.clone(),
        };

        std::thread::Builder::new().stack_size(32768).name("Sensors".to_string()).spawn(move || {
            instance.run();
        }).unwrap();
    }

    pub fn run(mut self) {
        log::info!("Initializing sensor...");
        self.tof.init().unwrap();
        log::info!("Sensor initialized.");

        loop {
            match self.tof.get_distance() {
                Ok(data) => {
                    //get min distance by column
                    let mut min_distance = [i16::MAX; 8];
                    for row in 4..8 {
                        for col in 0..8 {
                            if data[row][col] < min_distance[7 - col] {
                                min_distance[7 - col] = data[row][col];
                            }
                        }
                    }
                    self.event.send_event(Event::BackDistance { distance: min_distance });
                }
                Err(e) => {
                    log::error!("Error reading sensor data: {:?}", e);
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(250));
        }
    }
}