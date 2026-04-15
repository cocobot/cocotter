use std::{thread::sleep, time::Duration};
use asserv::holonomic::RobotSide;
use board_common::Team;
use embedded_hal::digital::InputPin;
use board_sabotter::{SabotterBoard, SabotterInputs};
use flume::Sender;

use crate::led::LedMessage;
use crate::meca::Meca;
use crate::sensors::Sensors;

pub mod types;

pub struct Strat<B: SabotterBoard> {
    side: Team,
    leds: Sender<LedMessage>,
    sensors: Sensors<B>,
    meca: Meca<B>,

    inputs: SabotterInputs<B::ExInputPin, B::ExInputPin>,
}

impl<B : SabotterBoard + 'static> Strat<B> {
    pub fn init(board: &mut B, leds: Sender<LedMessage>, sensors: Sensors<B>, meca: Meca<B>) {
        let instance = Self {
            side: Team::None,
            leds,
            sensors,
            meca,

            inputs: board.inputs().take().unwrap(),
        };

        std::thread::Builder::new()
            .name("strat".into())
            .stack_size(8192)
            .spawn(move || {
                instance.run();
            })
            .expect("spawn strat");
    }

    fn run(mut self) {
        self.prepare_match();
    }


    //----------

    fn prepare_match(&mut self) {
        log::info!("Color selection");

        
        //waiting for starter to be inserted
        loop {
            let side = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };

            self.side = side;
            self.leds.send(LedMessage::GameSide { side }).ok();

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_low().unwrap_or(false) {
                log::info!("Color selected"); 
                self.meca.init();               
                break;
            }                       
        }

        //waiting for starter to be removed
        self.meca.proxy.set_color_led_pwm(255);
        loop {
            let side = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };

            self.side = side;
            self.leds.send(LedMessage::GameSide { side }).ok();

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_high().unwrap_or(false) {
                self.meca.proxy.set_color_led_pwm(0);
                log::info!("Match started");                
                break;
            }                        

            self.meca.calobration_check_color();
        }
        
        self.meca.calibration_position();
        //self.meca.test_algo(0);

        loop {            
    
            std::thread::sleep(Duration::from_secs(1));
        }

        log::info!("Set meca in init position");
        //self.meca.prepare();
        

        log::warn!("Todo: recallage");
    }
    
}