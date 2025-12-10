use core::time::Duration;
use crate::conf::PidConf;


/// Implement asserv hardware behavior
///
/// [get_motor_offsets()] and [get_gyro_offset()] are called exactly once per aserv update.
pub trait AsservHardware {
    /// Return true if emergency stop is active
    fn emergency_stop_active(&mut self) -> bool;
    /// Set consign of motors (filtered PWM)
    fn set_motor_consigns(&mut self, values: [f32; 2]);
    /// Return motor encoder offset, since last call
    fn get_motor_offsets(&mut self) -> [f32; 2];
}


/// A structure with all dynamic asserv configuration
///
/// It is intended to be used with [Asserv::set_conf()] at startup.
/// Having everything in one struct helps to not forget a value.
#[derive(Clone, Default)]
pub struct AsservConf {
    pub pid_dist: PidConf,
    pub pid_angle: PidConf,
    pub motors: MotorsConf,
    pub trajectory: TrajectoryConf,
    /// Period between asserv updates
    ///
    /// Value is used to scale speeds and accelerations values
    pub update_period: Duration,
}


#[derive(Clone, Default)]
pub struct TrajectoryConf {
    /// Maximum angular speed
    pub a_speed: f32,
    /// Maximum angular acceleration
    pub a_acc: f32,
    /// Maximum linear speed
    pub xy_speed: f32,
    /// Maximum linear acceleration
    pub xy_acc: f32,
    /// Tolerance linear distance
    pub xy_stop_window: f32,
    /// Tolerance angle when aiming before linear move
    pub xy_aim_angle_window: f32,
    /// Tolerance angle during linear move
    pub xy_cruise_angle_window: f32,
    /// Distance under which angle consign is not updated
    pub xy_approach_window: f32,
    /// Tolerance angle distance for angle move
    pub a_stop_window: f32,
}


#[derive(Clone)]
pub struct MotorsConf {
    /// Ratio applied to convert encoder ticks to millimeters
    pub tick_to_mm: f32,
    /// Ratio applied to convert motor ticks to radians
    pub tick_to_rad: f32,
}

impl MotorsConf {
    /// Initalize from dimensions
    pub const fn from_dimensions(wheel_distance: f32, wheel_diameter: f32, encoder_ticks: usize) -> Self {
        Self {
            tick_to_mm: core::f32::consts::PI * wheel_diameter / encoder_ticks as f32,
            tick_to_rad: core::f32::consts::TAU * wheel_distance / (wheel_diameter * encoder_ticks as f32),
        }
    }
}

// Note: the Default implementation is unlikely to work, but it makes it possible
// to use `.. Default::default()` to initialize parent structures.
impl Default for MotorsConf {
    fn default() -> Self {
        Self {
            tick_to_mm: 1.0,
            tick_to_rad: 1.0,
        }
    }
}

