use crate::conf::PidConf;
use crate::maths::{Matrix33, MATRIX33_IDENTITY};


/// Implement asserv hardware behavior
///
/// [get_motor_offsets()] and [get_gyro_offset()] are called exactly once per aserv update.
pub trait AsservHardware {
    /// Enable or disable break on all motors
    fn set_motors_break(&mut self, enable: bool);
    /// Set consign of motors
    fn set_motor_consigns(&mut self, values: [f32; 3]);
    /// Return motor encoder offset, since last call
    fn get_motor_offsets(&mut self) -> [f32; 3];
    /// Get angle offset from gyroscope, since last call
    fn get_gyro_offset(&mut self) -> f32;
}


/// A structure with all dynamic asserv configuration
///
/// It is intended to be used with [Asserv::set_conf()] at startup.
/// Having everything in one struct helps to not forget a value.
#[derive(Clone, Default)]
pub struct AsservConf {
    pub pid_x: PidConf,
    pub pid_y: PidConf,
    pub pid_a: PidConf,
    pub trajectory: TrajectoryConf,
    pub motors: MotorsConf,
}


#[derive(Clone, Default)]
pub struct TrajectoryConf {
    /// Maximum angular speed
    pub a_speed: f32,
    /// Maximum angular acceleration
    pub a_acc: f32,
    /// Maximum linear speed, between trajectory points
    pub xy_cruise_speed: f32,
    /// Maximum linear acceleration, between trajectory points
    pub xy_cruise_acc: f32,
    /// Speed near intermediate trajectory points
    pub xy_steering_speed: f32,
    /// Acceleration near intermediate trajectory points
    pub xy_steering_acc: f32,
    /// Speed near the last trajectory points
    pub xy_stop_speed: f32,
    /// Acceleration near the last trajectory points
    pub xy_stop_acc: f32,
    /// Tolerance linear distance for reaching intermediate trajectory points
    pub xy_steering_window: f32,
    /// Tolerance linear distance for reaching the last trajectory point
    pub xy_stop_window: f32,
    /// Tolerance angle distance for reaching the last trajectory point
    pub a_stop_window: f32,
    /// Speed for autoset moves
    pub autoset_speed: f32,
    /// Waiting time (update ticks) before and after autoset move
    pub autoset_wait: u8,
    /// Autoset move duration (update ticks)
    pub autoset_duration: u8,
}


#[derive(Clone)]
pub struct MotorsConf {
    /// Matrix used to convert target velocity (relative to robot) to motor duty cycles
    pub matrix: Matrix33,
    /// Matrix used to convert motor encoder values to displacement (relative to robot)
    pub inv_matrix: Matrix33,
}

impl Default for MotorsConf {
    fn default() -> Self {
        Self {
            matrix: MATRIX33_IDENTITY,
            inv_matrix: MATRIX33_IDENTITY,
        }
    }
}

