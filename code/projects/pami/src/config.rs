use cocotter::{pid::PIDConfiguration, position::PositionConfiguration, ramp::RampConfiguration};
use esp_hal::efuse::Efuse;
use phf::phf_map;

pub const ASSERV_PERIOD_MS : u64 = 10;


pub const POSITION_CONFIG : PositionConfiguration = PositionConfiguration {
    tick_to_mm: 2.71666,
    tick_to_rad: 225.448, //112.724,
};

//unit are mm/s and mm/s^2
pub const DISTANCE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 100.0,
    acceleration: 200.0,
};

//unit are deg/s and deg/s^2
pub const ANGLE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 100.0,
    acceleration: 100.0,
};

pub const DISTANCE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 2.0,
    ki: 0.0,
    kd: 0.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub const ANGLE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 50.0,
    ki: 0.0,
    kd: 0.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

#[derive(Debug, Clone, Copy)]
pub struct MotorQuadrantConfiguration {
    pub gain: f32,
    pub min_pwm: i16,
    pub boost_pwm: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorConfiguration {
    pub invert: bool,
    pub forward: MotorQuadrantConfiguration,
    pub backward: MotorQuadrantConfiguration,
}


pub struct PAMIConfig {
    pub id: usize,
    pub color: &'static str,

    pub invert_encoder: [bool; 2],
    pub left_motor: MotorConfiguration,
    pub right_motor: MotorConfiguration,
}


static CONFIGS: phf::Map<[u8; 6], PAMIConfig> = phf_map! {
    //violet (ID = 0)
    [116, 77, 189, 81, 207, 136] => PAMIConfig {
        id: 0,
        color: "Violet",

        invert_encoder: [false, false],

        left_motor: MotorConfiguration {
            invert: false,
            forward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
            backward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
        },

        right_motor: MotorConfiguration {
            invert: false,
            forward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
            backward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
        },
    },

    //yellow (ID = 1)
    [116, 77, 189, 81, 239, 32] => PAMIConfig {
        id: 1,
        color: "Yellow",

        invert_encoder: [false, false],

        left_motor: MotorConfiguration {
            invert: false,
            forward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
            backward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
        },

        right_motor: MotorConfiguration {
            invert: false,
            forward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
            backward: MotorQuadrantConfiguration {
                gain: 1.0,
                min_pwm: 15,
                boost_pwm: 65,
            },
        },
    },
};

impl PAMIConfig {
    pub fn get_config() -> Option<&'static PAMIConfig> {
        let mac = Efuse::read_base_mac_address();

        match CONFIGS.get(&mac) {
            Some(config) => Some(config),
            None => panic!("No config found for this MAC address : {:?}", mac),
        }
    }
}