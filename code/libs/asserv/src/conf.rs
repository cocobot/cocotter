use crate::maths::{Matrix33, MATRIX33_IDENTITY};


/// Implement asserv hardware behavior
///
/// [get_motor_offset()] and [get_gyro_offset()] are called exactly once per aserv update.
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
/// Having everything in one struct helps to not forgot a value.
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
    /// Speed around near the last trajectory points
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


#[derive(Clone)]
pub struct PidConf {
    /// Gain of Proportionnal module
    pub gain_p: i16,
    /// Gain of Integral module
    pub gain_i: i16,
    /// Gain of Derivate module
    pub gain_d: i16,

    /// In saturation levels
    pub max_in: i32,
    /// Integral saturation levels
    pub max_i: i32,
    /// Out saturation levels
    pub max_out: i32,

    /// Big common divisor for output
    pub out_shift: u8,
}

impl Default for PidConf {
    fn default() -> Self {
        Self {
            gain_p: 1,
            gain_i: 0,
            gain_d: 0,
            max_in: 0,
            max_i: 0,
            max_out: 0,
            out_shift: 0,
        }
    }
}

impl PidConf {
    pub fn set_gains(&mut self, gp: i16, gi: i16, gd: i16) {
        self.gain_p = gp;
        self.gain_i = gi;
        self.gain_d = gd;
    }

    pub fn set_maximums(&mut self, max_in: i32, max_i: i32, max_out: i32) {
        self.max_in = max_in;
        self.max_i = max_i;
        self.max_out = max_out;
    }
}

