use std::sync::{Arc, Mutex};

use asserv::{holonomic::{Asserv, RobotSide, TableSide}, maths::XY};
use board_sabotter::SabotterBoard;

use crate::{movement::MovementLowLevelHardware, strat::errors::StrategyError};

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
}

impl<B: SabotterBoard> AsservHelper<B> {
    pub fn new(asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>) -> Self {
        Self { asserv }
    }

    pub fn goto_xya(&self, x: f32, y: f32, a: f32) -> Result<(), StrategyError> {
        self.asserv.lock().unwrap().goto_xya(x, y, a);
        self.wait()
    }

    pub fn goto_a(&self, a: f32) -> Result<(), StrategyError> {
        self.asserv.lock().unwrap().goto_a(a);
        self.wait()
    }

    /// Warp the robot to `xya`. In the simulator the chassis is
    /// actually moved; on physical hardware only the software pose
    /// estimate is updated (the caller is responsible for having
    /// placed the robot at that pose).
    pub fn teleport(&self, x: f32, y: f32, a: f32) {
        use asserv::maths::XYA;
        self.asserv.lock().unwrap().teleport(XYA::new(x, y, a));
    }

    pub fn run_path(&self, path: &[XY]) -> Result<(), StrategyError> {
        self.asserv.lock().unwrap().run_path(path);
        self.wait()
    }

    fn wait(&self) -> Result<(), StrategyError> {
        //TODO damien use passive waiting with Sender/Receiver
        loop {
            let asserv = self.asserv.lock().unwrap();
            if asserv.done_xy() && asserv.done_a() {
                return Ok(());
            }
            drop(asserv);
            
            std::thread::sleep(std::time::Duration::from_millis(25));
        }        
    }
}