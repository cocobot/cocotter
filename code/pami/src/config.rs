
pub struct PamiConfig {
    pub name: &'static str,
    pub bt_mac: [u8; 6],
    pub color: [f32; 3],
}

static CONFIGS: [PamiConfig; 4] = [
    PamiConfig {
        name: "PAMI0",
        bt_mac: [0x74, 0x4D, 0xBD, 0x51, 0xCF, 0x8A],
        color: [1.0, 0.0, 0.0],  // red
    },
    PamiConfig {
        name: "PAMI1",
        bt_mac: [0x74, 0x4D, 0xBD, 0x51, 0xEF, 0x22],
        color: [1.0, 0.8, 0.0],  // yellow
    },
    PamiConfig {
        name: "PAMI2",
        bt_mac: [0x74, 0x4D, 0xBD, 0x52, 0x4B, 0xAA],
        color: [0.0, 0.0, 1.0],  // blue
    },
    PamiConfig {
        name: "PAMI3",
        bt_mac: [0x74, 0x4D, 0xBD, 0x52, 0x86, 0xDE],
        color: [0.75, 0.0, 0.75],  // pink
    },
];

impl PamiConfig {
    pub fn by_bt_mac(mac: &[u8; 6]) -> Option<&'static PamiConfig> {
        CONFIGS.iter().find(|conf| &conf.bt_mac == mac)
    }
}

