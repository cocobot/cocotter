use cocotter::{pid::PIDConfiguration, position::PositionConfiguration, ramp::RampConfiguration};
use esp_idf_svc::sys::{esp_mac_type_t_ESP_MAC_BT, esp_read_mac};
use phf::phf_map;

use crate::game::GameStrategy;

pub const ASSERV_PERIOD_MS : u64 = 10;
pub const GAME_TIME_SECONDS : u64 = 100;
pub const PAMI_START_TIME_SECONDS : u64 = 85;

pub const POSITION_CONFIG : PositionConfiguration = PositionConfiguration {
    tick_to_mm: 2.67666666666667,
    tick_to_rad: 203.3183431, 
};

//unit are mm/s and mm/s^2
pub const DISTANCE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 200.0,
    acceleration: 750.0,
};

//unit are deg/s and deg/s^2
pub const ANGLE_RAMP_CONFIG : RampConfiguration = RampConfiguration {
    max_speed: 2.5,
    acceleration: 35.0,
};

pub const DISTANCE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 40.0,
    ki: 0.0,
    kd: 10.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub const ANGLE_PID_CONFIG : PIDConfiguration = PIDConfiguration {
    kp: 1500.0,
    ki: 0.0,
    kd: 1000.0,
    max_integral: 0.0,
    max_err_for_integral: 0.0,
};

pub struct PAMIConfig {
    pub id: usize,
    pub color: &'static str,
    pub strategy: GameStrategy,
}


static CONFIGS: phf::Map<[u8; 6], PAMIConfig> = phf_map! {
    //red (ID = 0)
    [116, 77, 189, 81, 207, 138] => PAMIConfig {
        id: 0,
        color: "Red",
        strategy: GameStrategy::FarPit,
    },

    //yellow (ID = 1)
    [116, 77, 189, 81, 239, 34] => PAMIConfig {
        id: 1,
        color: "Yellow",
        strategy: GameStrategy::Superstar,
    },
};

impl PAMIConfig {
    pub fn get_config() -> Option<&'static PAMIConfig> {

        let mut mac: [u8; 6] = [0; 6];
        log::info!("MAC address : {:?}", mac);
        unsafe {
            let raw_ptr = &mut mac as *mut [u8; 6] as *mut u8;
            esp_read_mac(raw_ptr, esp_mac_type_t_ESP_MAC_BT);
        };

        log::info!("MAC address : {:?}", mac);

        match CONFIGS.get(&mac) {
            Some(config) => Some(config),
            None => panic!("No config found for this MAC address : {:?}", mac),
        }
    }
}