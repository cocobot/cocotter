use std::cell::Cell;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use asserv::{holonomic::{Asserv, RobotSide, TableSide}, maths::{XY, XYA}};
use board_sabotter::SabotterBoard;

use board_sabotter::movement::MovementLowLevelHardware;
use crate::{opponent_detection::OpponentDetection, strat::errors::StrategyError};

const MATCH_DURATION_SECS: u64 = 100;

#[derive(Clone, Copy, PartialEq)]
pub enum StopMode {
    Reject,
    WaitAndResume,
}

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
    stop_mode: StopMode,
    match_start: Arc<Mutex<Option<Instant>>>,
    motor_disabled: Cell<bool>,
    allow_pre_end_of_match: Cell<bool>,
}

impl<B: SabotterBoard> AsservHelper<B> {
    pub fn new(asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>, opponent_detection: OpponentDetection) -> Self {
        Self { asserv, opponent_detection, stop_mode: StopMode::Reject, match_start: Arc::new(Mutex::new(None)), motor_disabled: Cell::new(false), allow_pre_end_of_match: Cell::new(false) }
    }

    pub fn set_stop_mode(&mut self, mode: StopMode) {
        self.stop_mode = mode;
    }

    pub fn set_match_started(&self) {
        *self.match_start.lock().unwrap() = Some(Instant::now());
    }

    pub fn is_end_of_match(&self) -> bool {
        self.match_start.lock().unwrap()
            .map_or(false, |start| start.elapsed().as_secs() >= MATCH_DURATION_SECS)
    }

    pub fn is_pre_end_of_match(&self) -> bool {
        if self.allow_pre_end_of_match.get() {
            return false;
        }

        self.match_start.lock().unwrap()
            .map_or(false, |start| start.elapsed().as_secs() >= MATCH_DURATION_SECS - 17)
    }

    pub fn allow_pre_end_of_match(&self, val: bool) {
        self.allow_pre_end_of_match.set(val);
    }

    pub fn ellapsed_time_since_start(&self) ->  Duration{
        self.match_start.lock().unwrap().map_or(Duration::from_secs(1000),|start| start.elapsed())
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

    pub fn xy_cruise_speed(&self) -> (f32, f32) {
        self.asserv.lock().unwrap().xy_cruise_speed()
    }

    pub fn set_xy_cruise_speed(&self, speed: f32, acc: f32) {
        self.asserv.lock().unwrap().set_xy_cruise_speed(speed, acc);
    }

    pub fn disable_motor_control(&self) {
        self.motor_disabled.set(true);
        self.asserv.lock().unwrap().cs.disable_motor_control();
    }

    pub fn enable_motor_control(&self) {
        if self.motor_disabled.get() {
            self.motor_disabled.set(false);
            self.asserv.lock().unwrap().cs.enable_motor_control();
        }
    }

    pub fn goto_xya(&self, x: f32, y: f32, a: f32) -> Result<(), StrategyError> {
        loop {
            if self.is_end_of_match() { return Err(StrategyError::EndOfMatch); }
            if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }
            
            self.enable_motor_control();
            if !self.asserv.lock().unwrap().goto_xya(x, y, a) {
                if self.stop_mode == StopMode::WaitAndResume {
                    self.wait_opponent_clear();
                    continue;
                }
                return Err(StrategyError::OpponentDetected);
            }
            match self.wait() {
                Ok(()) => return Ok(()),
                Err(_) if self.stop_mode == StopMode::WaitAndResume => continue,
                Err(e) => return Err(e),
            }
        }
    }

    pub fn goto_xy_rel(&self, dx: f32, dy: f32) -> Result<(), StrategyError> {
        if self.is_end_of_match() { return Err(StrategyError::EndOfMatch); }
        if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }

        let start = self.position();
        let target_x = start.x + dx;
        let target_y = start.y + dy;
        self.enable_motor_control();
        if !self.asserv.lock().unwrap().goto_xy_rel(dx, dy) {
            if self.stop_mode == StopMode::WaitAndResume {
                self.wait_opponent_clear();
            } else {
                return Err(StrategyError::OpponentDetected);
            }
        }
        loop {
            match self.wait() {
                Ok(()) => return Ok(()),
                Err(_) if self.stop_mode == StopMode::WaitAndResume => {
                    if self.is_end_of_match() { return Err(StrategyError::EndOfMatch); }
                    if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }

                    self.enable_motor_control();
                    if !self.asserv.lock().unwrap().goto_xya(target_x, target_y, self.position().a) {
                        return Err(StrategyError::OpponentDetected);
                    }
                    continue;
                },
                Err(e) => return Err(e),
            }
        }
    }

    pub fn goto_a(&self, a: f32) -> Result<(), StrategyError> {
        loop {
            if self.is_end_of_match() { return Err(StrategyError::EndOfMatch); }
            if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }

            self.enable_motor_control();
            if !self.asserv.lock().unwrap().goto_a(a) {
                if self.stop_mode == StopMode::WaitAndResume {
                    self.wait_opponent_clear();
                    continue;
                }
                return Err(StrategyError::OpponentDetected);
            }
            match self.wait() {
                Ok(()) => return Ok(()),
                Err(_) if self.stop_mode == StopMode::WaitAndResume => continue,
                Err(e) => return Err(e),
            }
        }
    }

    pub fn run_path(&self, path: &[XY]) -> Result<(), StrategyError> {
        loop {
            if self.is_end_of_match() { return Err(StrategyError::EndOfMatch); }
            if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }


            self.enable_motor_control();
            if !self.asserv.lock().unwrap().run_path(path) {
                if self.stop_mode == StopMode::WaitAndResume {
                    self.wait_opponent_clear();
                    continue;
                }
                return Err(StrategyError::OpponentDetected);
            }
            match self.wait() {
                Ok(()) => return Ok(()),
                Err(_) if self.stop_mode == StopMode::WaitAndResume => continue,
                Err(e) => return Err(e),
            }
        }
    }

    fn wait_opponent_clear(&self) {
        log::warn!("Preflight blocked, waiting for opponent to clear...");
        while self.opponent_detection.must_stop() {
            self.opponent_detection.clear_stop();
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
        log::warn!("Opponent cleared");
    }

    fn wait(&self) -> Result<(), StrategyError> {
        //TODO damien use passive waiting with Sender/Receiver
        let mut was_slow = false;
        let (saved_cruise_speed, saved_cruise_acc) = self.xy_cruise_speed();
        loop {
            if self.is_end_of_match() {
                self.disable_motor_control();
                if was_slow {
                    self.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                }
                return Err(StrategyError::EndOfMatch);
            }
            if self.is_pre_end_of_match() { return Err(StrategyError::PreEndOfMatch); }


            if self.opponent_detection.must_stop() {
                if was_slow {
                    self.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                }
                //
                self.disable_motor_control();
                self.asserv.lock().unwrap().stop();

                if self.stop_mode == StopMode::WaitAndResume {
                    self.wait_opponent_clear();
                }

                return Err(StrategyError::OpponentDetected);
            }

            let is_slow = self.opponent_detection.must_slow();
            if is_slow && !was_slow {
                let slow_speed = self.opponent_detection.slow_cruise_speed();
                log::warn!("Slow triggered: cruise {saved_cruise_speed} -> {slow_speed}");
                self.set_xy_cruise_speed(slow_speed, saved_cruise_acc);
                was_slow = true;
            } else if !is_slow && was_slow {
                log::warn!("Slow cleared: cruise -> {saved_cruise_speed}");
                self.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                was_slow = false;
            }

            let asserv = self.asserv.lock().unwrap();
            if asserv.done_xy() && asserv.done_a() {
                if was_slow {
                    drop(asserv);
                    self.set_xy_cruise_speed(saved_cruise_speed, saved_cruise_acc);
                }
                return Ok(());
            }
            drop(asserv);

            std::thread::sleep(std::time::Duration::from_millis(25));
        }
    }
}
