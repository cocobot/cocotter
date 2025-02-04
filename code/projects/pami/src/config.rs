use cocotter::{pid::PIDConfiguration, position::PositionConfiguration, ramp::RampConfiguration};

pub const ASSERV_PERIOD_MS : u64 = 10;

pub const ASSERV_PWM_OFFSET_MEAS_PERIOD_MS : u64 = 10;

pub const ASSERV_DEAD_ZONE_SPEED : f32 = 5.0; // 


pub const POSITION_CONFIG : PositionConfiguration = PositionConfiguration {
    tick_to_mm: 2.71666,
    tick_to_rad: 225.448, //112.724,
};

//unit are mm/s and mm/s^2
pub const DISTANCE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 1000.0,
    acceleration: 2000.0,
};

//unit are deg/s and deg/s^2
pub const ANGLE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 500.0,
    acceleration: 1000.0,
};

pub const DISTANCE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 2.0,
    ki: 0.0,
    kd: 4.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub const ANGLE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 20.0,
    ki: 0.0,
    kd: 8.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};