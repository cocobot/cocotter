#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(improper_ctypes)]
#![allow(clippy::all)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));


// Macros, binded manually, and not all of them

/// cbindgen:ignore
mod constants {
    pub const VL53L5CX_DEFAULT_I2C_ADDRESS: u16 = 0x52;
    pub const VL53L5CX_RESOLUTION_4X4: u8 = 16;
    pub const VL53L5CX_RESOLUTION_8X8: u8 = 64;
    pub const VL53L5CX_TARGET_ORDER_CLOSEST: u8 = 1;
    pub const VL53L5CX_TARGET_ORDER_STRONGEST: u8 = 2;
    pub const VL53L5CX_RANGING_MODE_CONTINUOUS: u8 = 1;
    pub const VL53L5CX_RANGING_MODE_AUTONOMOUS: u8 = 3;
    pub const VL53L5CX_POWER_MODE_SLEEP: u8 = 0;
    pub const VL53L5CX_POWER_MODE_WAKEUP: u8 = 1;

    pub const VL53L5CX_STATUS_OK: u8 = 0;
    pub const VL53L5CX_STATUS_TIMEOUT_ERROR: u8 = 1;
    pub const VL53L5CX_STATUS_CORRUPTED_FRAME: u8 = 2;
    pub const VL53L5CX_STATUS_CRC_CSUM_FAILED: u8 = 3;
    pub const VL53L5CX_STATUS_XTALK_FAILED: u8 = 4;
    pub const VL53L5CX_MCU_ERROR: u8 = 66;
    pub const VL53L5CX_STATUS_INVALID_PARAM: u8 = 127;
    pub const VL53L5CX_STATUS_ERROR: u8 = 255;

    pub const VL53L5CX_NB_THRESHOLDS: u8 = 64;
    pub const VL53L5CX_DCI_DET_THRESH_CONFIG: u16 = 0x5488;
    pub const VL53L5CX_DCI_DET_THRESH_GLOBAL_CONFIG: u16 = 0xB6E0;
    pub const VL53L5CX_DCI_DET_THRESH_START: u16 = 0xB6E8;
    pub const VL53L5CX_DCI_DET_THRESH_VALID_STATUS: u16 = 0xB9F0;
    pub const VL53L5CX_LAST_THRESHOLD: u8 = 128;
    pub const VL53L5CX_DISTANCE_MM: u8 = 1;
    pub const VL53L5CX_SIGNAL_PER_SPAD_KCPS: u8 = 2;
    pub const VL53L5CX_RANGE_SIGMA_MM: u8 = 4;
    pub const VL53L5CX_AMBIENT_PER_SPAD_KCPS: u8 = 8;
    pub const VL53L5CX_NB_TARGET_DETECTED: u8 = 9;
    pub const VL53L5CX_TARGET_STATUS: u8 = 12;
    pub const VL53L5CX_NB_SPADS_ENABLED: u8 = 13;
    pub const VL53L5CX_MOTION_INDICATOR: u8 = 19;
    pub const VL53L5CX_IN_WINDOW: u8 = 0;
    pub const VL53L5CX_OUT_OF_WINDOW: u8 = 1;
    pub const VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER: u8 = 2;
    pub const VL53L5CX_GREATER_THAN_MAX_CHECKER: u8 = 3;
    pub const VL53L5CX_EQUAL_MIN_CHECKER: u8 = 4;
    pub const VL53L5CX_NOT_EQUAL_MIN_CHECKER: u8 = 5;
    pub const VL53L5CX_OPERATION_NONE: u8 = 0;
    pub const VL53L5CX_OPERATION_OR: u8 = 0;
    pub const VL53L5CX_OPERATION_AND: u8 = 2;
}
pub use constants::*;

