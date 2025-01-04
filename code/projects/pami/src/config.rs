use cocotter::{pid::PIDConfiguration, position::regular::RegularPositionConfiguration, ramp::RampConfiguration};

pub const ASSERV_PERIOD_MS : u64 = 10;
pub const ASSERV_PWM_OFFSET_MEAS_PERIOD_MS : u64 = 10;


pub const POSITION_CONFIG : RegularPositionConfiguration = RegularPositionConfiguration {
    tick_to_mm: 2.71666,
    tick_to_rad: 225.448, //112.724,
};

//unit are mm/s and mm/s^2
pub const DISTANCE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 500.0,
    acceleration: 1000.0,
};

//unit are deg/s and deg/s^2
pub const ANGLE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 50.0,
    acceleration: 10.0,
};

pub const DISTANCE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 2.0,
    ki: 0.0,
    kd: 4.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub const ANGLE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 200.0,
    ki: 0.0,
    kd: 10.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};