use board_pami_2023::{TCA6408AType, VlxType};

use crate::{events::{Event, EventSystem}, pwm::PWMEvent};


pub struct Sensors {
    tof: VlxType,
    line_sensor: TCA6408AType,
    event: EventSystem,
}

impl Sensors {
    pub fn new(tof: VlxType, line_sensor: TCA6408AType, event: &EventSystem) {
        let instance = Sensors {
            tof: tof,
            line_sensor,
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

        self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LineLedLvl(1.0) });

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
                    //log::error!("Error reading sensor data: {:?}", e);
                }
            }
            let line_sensor = self.line_sensor.get_input().unwrap();
            let activated = [
                line_sensor & 0b000_00001 != 0,
                line_sensor & 0b000_00010 != 0,
                line_sensor & 0b000_00100 != 0,
                line_sensor & 0b000_01000 != 0,
                line_sensor & 0b000_10000 != 0,
                line_sensor & 0b001_00000 != 0,
                line_sensor & 0b010_00000 != 0,
                line_sensor & 0b100_00000 != 0,
            ];
            log::info!("Line sensor activated: {:?}", activated);
            self.event.send_event(Event::Line { activated });

            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    }
}