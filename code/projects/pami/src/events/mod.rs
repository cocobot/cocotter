use std::{sync::{mpsc, Arc, Mutex}, time::Instant};

use cocotter::{position::robot_coordinate::RobotCoordinate};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

use crate::pwm::{OverrideState, PWMEvent};

pub type EventFilter = fn(&Event) -> bool;
pub type EventCallback = Box<dyn FnMut(&Event) -> () + Send + 'static>;

#[derive(Debug, Clone)]
pub enum Event {
    //game events
    GameStarted { timestamp: Instant, test_mode: bool },

    //analog inputs
    Vbatt {voltage_mv: f32},
    VbattPercent {_percent: u8},

    //sensors
    BackDistance { distance: [i16; 8] },
    Line { activated: [bool; 8]},
    
    ////pwm outputs
    Pwm { pwm_event : PWMEvent},
    OverridePwm { pwm_event : PWMEvent, override_state: OverrideState},

    ////asserv
    Position { coords: RobotCoordinate::<2> },
    //MotorDebug {timestamp: u16, left_tick: i32, right_tick: i32, left_pwm: i16, right_pwm: i16}, 
    //PIDDebug {timestamp: u16, target_d: f32, current_d: f32, output_d: f32, target_a: f32, current_a: f32, output_a: f32},
}

#[derive(Clone)]
pub struct EventSystem {
    sender: mpsc::Sender<(Event, Instant)>,
    callbacks: Arc<Mutex<Vec<(Option<EventFilter>, EventCallback)>>>,
}

impl EventSystem {
    pub fn new() -> Self {
        let (sender, receiver) = mpsc::channel();

        let instance = EventSystem {
            sender,
            callbacks: Arc::new(Mutex::new(Vec::new())),
        };
        let ret = instance.clone();

        ThreadSpawnConfiguration {
            name: Some("Events\0".as_bytes()),
            stack_size: 8192 * 4,
            ..Default::default()
        }.set().unwrap();

        std::thread::Builder::new().spawn(move || {
            instance.run(receiver);
        }).unwrap();

        ret
    }

    fn run(self, receiver: mpsc::Receiver<(Event, Instant)>) {      
        loop {
            match receiver.recv() {
                Ok(msg) => {
                    let (event, timestamp) = msg;
                    if timestamp.elapsed().as_millis() > 1000 {
                        log::warn!("Event received with a delay of more than 1 second: {:?}", event);
                    }

                    // Process the event
                    for (filter, callback) in self.callbacks.lock().unwrap().iter_mut() {
                        if filter.is_none() || filter.unwrap()(&event) {
                            callback(&event);
                        }
                    }
                }
                Err(_) => {
                    log::error!("Event receiver error");
                }
            }   
        }
    }
    

    pub fn send_event(&self, event: Event) {
        match self.sender.send((event, Instant::now())) {
            Ok(_) => {}
            Err(e) => {
                log::error!("Failed to send event: {:?}", e);
            }
        }
    }

    pub fn register_receiver_callback<F>(
        &self, 
        filter: Option<EventFilter>, 
        callback: F
    ) where 
        F: FnMut(&Event) -> () + Send + 'static,
    {
        self.callbacks.lock().unwrap().push((
            filter, 
            Box::new(callback)
        ));
    }

    pub fn no_debug_filter(evt: &Event) -> bool {
        match evt {
            //Event::MotorDebug { .. } => false,
            //Event::PIDDebug { .. } => false,
            _ => true,
        }
    }
}