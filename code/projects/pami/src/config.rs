use cocotter::{pid::PIDConfiguration, position::regular::RegularPositionConfiguration, ramp::RampConfiguration};

pub const ASSERV_PERIOD_MS : u64 = 50;

pub const POSITION_CONFIG : RegularPositionConfiguration = RegularPositionConfiguration {
    tick_to_mm: 1.0,
    tick_to_rad: 1.0,
};

//unit are mm/s and mm/s^2
pub const DISTANCE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 5.0,
    acceleration: 10.0,
};

//unit are deg/s and deg/s^2
pub const ANGLE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 5.0,
    acceleration: 10.0,
};

pub const DISTANCE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 1.0,
    ki: 0.0,
    kd: 0.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub const ANGLE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 1.0,
    ki: 0.0,
    kd: 0.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};