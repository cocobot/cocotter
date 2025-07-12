use phf::phf_map;
use std::env;

#[cfg(target_os = "espidf")]
use esp_idf_svc::sys::{esp_mac_type_t_ESP_MAC_BT, esp_read_mac};


pub struct PAMIConfig {
    pub id: usize,
    pub color: [f32; 3],
}

static CONFIGS: phf::Map<[u8; 6], PAMIConfig> = phf_map! {
    //red (ID = 0)
    [116, 77, 189, 81, 207, 138] => PAMIConfig {
        id: 0,
        color: [1.0, 0.0, 0.0],
    },

    //yellow (ID = 1)
    [116, 77, 189, 81, 239, 34] => PAMIConfig {
        id: 1,
        color: [1.0, 0.8, 0.0],
    },

    //blue (ID = 2)
    [116, 77, 189, 82, 75, 170] => PAMIConfig {
        id: 2,
        color: [0.0, 0.0, 1.0],
    },

    //pink (ID = 3)
    [116, 77, 189, 82, 134, 222] => PAMIConfig {
        id: 3,
        color: [0.75, 0.0, 0.75],
    },
};

impl PAMIConfig {
    pub fn get_mac_address() -> [u8; 6] {
        #[cfg(target_os = "espidf")]
        {
            //safety: we know that the mac address is 6 bytes long
            //and that the esp_read_mac function is safe to call
            let mut mac: [u8; 6] = [0; 6];
            unsafe {
                let raw_ptr = &mut mac as *mut [u8; 6] as *mut u8;
                esp_read_mac(raw_ptr, esp_mac_type_t_ESP_MAC_BT);
            };

            return mac
        }
        #[cfg(target_os = "linux")]
        {
            let robot_id = env::args().nth(1).expect("Except first argument to be the id of the robot");
            let robot_id = robot_id.parse::<usize>().unwrap();
            let mac = CONFIGS.entries().find(|(_, config)| config.id == robot_id).map(|(mac, _)| *mac).expect("Invalid robot id");

            return mac
        }
    }
    pub fn get_config() -> Option<&'static PAMIConfig> {       
        CONFIGS.get(&PAMIConfig::get_mac_address())
    }
}