use std::sync::{Arc, Mutex};
use std::{thread::sleep, time::Duration};
use asserv::holonomic::{Asserv, RobotSide, TableSide};
use asserv::maths::XY;
use board_common::Team;
use embedded_hal::digital::InputPin;
use board_sabotter::{SabotterBoard, SabotterInputs};
use flume::Sender;

use crate::arfast;
use crate::led::LedMessage;
use crate::meca::Meca;
use crate::movement::MovementLowLevelHardware;
use crate::sensors::Sensors;
use crate::strat::utils::{AsservHelper, arfast};

pub mod utils;
pub mod errors;

pub struct Strat<B: SabotterBoard> {
    side: Team,
    leds: Sender<LedMessage>,
    sensors: Sensors<B>,
    meca: Meca<B>,
    asserv: AsservHelper<B>,

    inputs: SabotterInputs<B::ExInputPin, B::ExInputPin>,
}

impl<B : SabotterBoard + 'static> Strat<B> {
    pub fn init(board: &mut B, leds: Sender<LedMessage>, sensors: Sensors<B>, meca: Meca<B>, asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>) {
        let instance = Self {
            side: Team::None,
            leds,
            sensors,
            meca,
            asserv: AsservHelper::new(asserv),

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
        loop {
            let side = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };

            self.side = side;
            self.leds.send(LedMessage::GameSide { side }).ok();

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_high().unwrap_or(false) {
                log::info!("Match started");                
                break;
            }
        }
        
        self.test_movement();

        loop {            
            std::thread::sleep(Duration::from_secs(1));
        }
    }

    fn test_movement(&mut self) {
        let prefered_side = RobotSide::Left;
                
        //meca raise drop
        let side = self.meca.prepare_direct_take(Some(prefered_side)).unwrap();
        self.asserv.goto_xya(230.0, 200.0, arfast(prefered_side, TableSide::Left)).ok();

        self.asserv.run_path(&[
            XY::new(230.0, 200.0),
            XY::new(200.0, 440.0),
            XY::new(130.0, 440.0),
        ]).ok();

        //meca take
        self.meca.direct_take(side);
        self.asserv.goto_a(arfast!(Left, Down)).ok();

        self.asserv.run_path(&[
            XY::new(230.0, 200.0),
            XY::new(200.0, 440.0),
            XY::new(130.0, 440.0),
        ]).ok();

        self.meca.drop(side);
        self.asserv.goto_xya(0.0, 100.0, arfast!(Left, Down)).ok();
    }
}