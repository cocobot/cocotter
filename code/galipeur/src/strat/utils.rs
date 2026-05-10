use std::sync::{Arc, Mutex};

use asserv::{holonomic::{Asserv, RobotSide, TableSide}, maths::{XY, XYA}};
use board_sabotter::SabotterBoard;

use board_sabotter::movement::MovementLowLevelHardware;
use crate::{opponent_detection::OpponentDetection, strat::errors::StrategyError};

pub const fn arfast(face: RobotSide, side: TableSide) -> f32 {
    match (face, side) {
        (RobotSide::Left,  TableSide::Left)  => std::f32::consts::PI *  1.0/6.0,
        (RobotSide::Left,  TableSide::Right) => std::f32::consts::PI * -5.0/6.0,
        (RobotSide::Left,  TableSide::Up)    => std::f32::consts::PI * -1.0/3.0,
        (RobotSide::Left,  TableSide::Down)  => std::f32::consts::PI *  2.0/3.0,
        (RobotSide::Right, TableSide::Left)  => std::f32::consts::PI *  5.0/6.0,
        (RobotSide::Right, TableSide::Right) => std::f32::consts::PI * -1.0/6.0,
        (RobotSide::Right, TableSide::Up)    => std::f32::consts::PI *  1.0/3.0,
        (RobotSide::Right, TableSide::Down)  => std::f32::consts::PI * -2.0/3.0,
        (RobotSide::Back,  TableSide::Left)  => std::f32::consts::PI * -1.0/2.0,
        (RobotSide::Back,  TableSide::Right) => std::f32::consts::PI *  1.0/2.0,
        (RobotSide::Back,  TableSide::Up)    => std::f32::consts::PI *  1.0,
        (RobotSide::Back,  TableSide::Down)  => std::f32::consts::PI *  0.0,
    }
}

#[macro_export]
macro_rules! arfast {
    ($face:ident, $side: ident) => { $crate::strat::utils::arfast(asserv::holonomic::RobotSide::$face, asserv::holonomic::TableSide::$side) }
}

#[derive(Clone)]
pub struct AsservHelper<B: SabotterBoard> {
    asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>,
    opponent_detection: OpponentDetection,
}

impl<B: SabotterBoard> AsservHelper<B> {
    pub fn new(asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>, opponent_detection: OpponentDetection) -> Self {
        Self { asserv, opponent_detection }
    }

    pub fn position(&self) -> XYA {
        let asserv = self.asserv.lock().unwrap();
        *asserv.cs.position()
    }

    pub fn teleport(&self, x: f32, y: f32, a: f32) {
        self.asserv.lock().unwrap().teleport(XYA::new(x, y, a));
    }
    
    pub fn reset_position(&self, x: f32, y: f32, a: f32) {
        self.asserv.lock().unwrap().reset_position(XYA::new(x, y, a));
    }

    pub fn disable_motor_control(&self) {
        self.asserv.lock().unwrap().cs.disable_motor_control();
    }

    pub fn enable_motor_control(&self) {
        self.asserv.lock().unwrap().cs.enable_motor_control();
    }

    pub fn goto_xya(&self, x: f32, y: f32, a: f32) -> Result<(), StrategyError> {
        if !self.asserv.lock().unwrap().goto_xya(x, y, a) {
            return Err(StrategyError::OpponentDetected);
        }
        self.wait()
    }

    pub fn goto_a(&self, a: f32) -> Result<(), StrategyError> {
        if !self.asserv.lock().unwrap().goto_a(a) {
            return Err(StrategyError::OpponentDetected);
        }
        self.wait()
    }

    pub fn run_path(&self, path: &[XY]) -> Result<(), StrategyError> {
        if !self.asserv.lock().unwrap().run_path(path) {
            return Err(StrategyError::OpponentDetected);
        }
        self.wait()
    }

    fn wait(&self) -> Result<(), StrategyError> {
        //TODO damien use passive waiting with Sender/Receiver
        let mut was_slow = false;
        let mut saved_cruise_speed = 0.0f32;
        let mut saved_cruise_acc = 0.0f32;
        loop {
            if self.opponent_detection.must_stop() {
                if was_slow {
                    let mut asserv = self.asserv.lock().unwrap();
                    asserv.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                }
                self.asserv.lock().unwrap().stop();
                return Err(StrategyError::OpponentDetected);
            }

            let is_slow = self.opponent_detection.must_slow();
            if is_slow && !was_slow {
                let mut asserv = self.asserv.lock().unwrap();
                (saved_cruise_speed, saved_cruise_acc) = asserv.xy_cruise_speed();
                let slow_speed = self.opponent_detection.slow_cruise_speed();
                log::warn!("Slow triggered: cruise {saved_cruise_speed} -> {slow_speed}");
                asserv.set_xy_cruise_speed(slow_speed, saved_cruise_acc);
                was_slow = true;
            } else if !is_slow && was_slow {
                log::warn!("Slow cleared: cruise -> {saved_cruise_speed}");
                let mut asserv = self.asserv.lock().unwrap();
                asserv.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                was_slow = false;
            }

            let asserv = self.asserv.lock().unwrap();
            if asserv.done_xy() && asserv.done_a() {
                if was_slow {
                    drop(asserv);
                    self.asserv.lock().unwrap().set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                }
                return Ok(());
            }
            drop(asserv);

            std::thread::sleep(std::time::Duration::from_millis(25));
        }
    }
}
