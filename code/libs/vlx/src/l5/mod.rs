mod platform;

use crate::bindings::*;
use crate::{DistanceData, VlxError, VlxSensor, VlxI2cDriver, ZoneAlarm};


macro_rules! status_result {
    ($expr:expr) => {
        match unsafe { $expr } {
            VL53L5CX_STATUS_OK => Ok(()),
            VL53L5CX_STATUS_TIMEOUT_ERROR => Err(VlxError::TimeoutError),
            VL53L5CX_STATUS_INVALID_PARAM => Err(VlxError::InvalidParam),
            VL53L5CX_STATUS_ERROR => Err(VlxError::LowLevelError),
            // Assume communication error
            _ => Err(VlxError::CommunicationError),
        }
    }
}

pub const ALARMS_COUNT: usize = VL53L5CX_NB_THRESHOLDS as usize;
pub const DEFAULT_ADDR: u16 = 0x29;  // = VL53L5CX_DEFAULT_I2C_ADDRESS / 2

pub struct VL53L5CX {
    address: u16,  // Never equal to DEFAULT_ADDR
    conf: Box<VL53L5CX_Configuration>,
}

impl VL53L5CX {
    pub fn new(_driver: &VlxI2cDriver, address: u8) -> Self {
        let address = address as u16;
        if address == DEFAULT_ADDR {
            panic!("Cannot use reserved default VLX address");
        }

        // Zero-initialize configuration for now, will be properly initialized in init
        Self {
            address,
            conf: Box::default(),
        }
    }

    fn enable_thresholds(&mut self, enable: bool) -> Result<(), VlxError> {
        status_result!(vl53l5cx_set_detection_thresholds_enable(&mut *self.conf, enable as u8))
    }

    /// Return measure resolution (zone dimensions)
    fn resolution(&self) -> (u8, u8) {
        // For now, hardcoded
        (4, 4)
    }

    /// Convert (x, y) coordinates into a zone index
    fn zone_index(&self, x: u8, y: u8) -> Option<u8> {
        let (width, height) = self.resolution();
        if x < width && y < height {
            Some(y * width + x)
        } else {
            None
        }
    }
}

impl VlxSensor for VL53L5CX {
    fn init(&mut self) -> Result<(), VlxError> {
        assert_ne!(self.address, DEFAULT_ADDR);  // Note: already checked in constructor

        self.conf.platform.address = self.address;
        let mut is_alive = 0u8;
        let status = unsafe { vl53l5cx_is_alive(&mut *self.conf, &mut is_alive) };
        if status != 0 || is_alive == 0 {
            // No device at the configured address, try the default address
            self.conf.platform.address = DEFAULT_ADDR;

            // Check if sensor is alive
            status_result!(vl53l5cx_is_alive(&mut *self.conf, &mut is_alive))?;
            if is_alive == 0 {
                return Err(VlxError::SensorNotFound);
            }
            // Initialize sensor
            status_result!(vl53l5cx_init(&mut *self.conf))?;

            // Set final I2C address
            status_result!(vl53l5cx_set_i2c_address(&mut *self.conf, self.address * 2))?;
            self.conf.platform.address = self.address;

            // Set 4x4 resolution for better performance
            // Note: 4x4 resolution is hardcoded in `resolution()`
            status_result!(vl53l5cx_set_resolution(&mut *self.conf, VL53L5CX_RESOLUTION_4X4))?;

            // Set power mode to wake up
            status_result!(vl53l5cx_set_power_mode(&mut *self.conf, VL53L5CX_POWER_MODE_WAKEUP))?;
        }

        // Note: if device address is correct, assume it's already configured

        // Start ranging automatically
        status_result!(vl53l5cx_start_ranging(&mut *self.conf))?;

        Ok(())
    }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {
        // Check if data is ready
        let mut is_data_ready = 0u8;
        status_result!(vl53l5cx_check_data_ready(&mut *self.conf, &mut is_data_ready))?;
        if is_data_ready == 0 {
            return Err(VlxError::DataNotReady);
        }

        // Get ranging data
        let mut results = VL53L5CX_ResultsData::default();
        status_result!(vl53l5cx_get_ranging_data(&mut *self.conf, &mut results))?;

        let (width, height) = self.resolution();

        // Extract all zone distances - VL53L5CX can be configured for different resolutions
        // Use u16::MAX for invalid measurement
        let distances: Vec<u16> = (0..(width * height) as usize).map(|i| {
            // Check if target is detected and status is valid (5 & 9 mean valid measurement)
            if results.target_status[i] == 5 || results.target_status[i] == 9 {
                results.distance_mm[i] as u16
            } else {
                u16::MAX
            }
        }).collect();

        Ok(DistanceData::new(width, height, distances))
    }

    fn set_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        const fn default_threshold() -> VL53L5CX_DetectionThresholds {
            VL53L5CX_DetectionThresholds {
                param_low_thresh: 0,
                param_high_thresh: 0,
                measurement: 0,
                type_: 0,
                zone_num: VL53L5CX_LAST_THRESHOLD,
                mathematic_operation: 0,
            }
        }

        const fn alarm_threshold(low: u16, high: u16, zone_num: u8) -> VL53L5CX_DetectionThresholds {
            VL53L5CX_DetectionThresholds {
                param_low_thresh: low as i32,
                param_high_thresh: high as i32,
                measurement: VL53L5CX_DISTANCE_MM,
                type_: VL53L5CX_IN_WINDOW,
                zone_num,
                mathematic_operation: VL53L5CX_OPERATION_OR,
            }
        }

        if alarms.len() > ALARMS_COUNT as usize {
            Err(VlxError::TooManyAlarms)
        } else if alarms.is_empty() {
            // Disable detection thresholds plugin to clear all alarms
            self.enable_thresholds(false)
        } else {
            self.enable_thresholds(true)?;
            let single_zone = if alarms.len() == 1 && alarms[0].zone.is_none() { Some(&alarms[0]) } else { None };
            let mut thresholds: [VL53L5CX_DetectionThresholds; ALARMS_COUNT] = std::array::from_fn(|i| {
                if let Some(alarm) = alarms.get(i).or(single_zone) {
                    let zone_num = alarm.zone.and_then(|z| self.zone_index(z.0, z.1)).unwrap_or(VL53L5CX_LAST_THRESHOLD);
                    alarm_threshold(alarm.low, alarm.high, zone_num)
                } else {
                    default_threshold()
                }
            });
            status_result!(vl53l5cx_set_detection_thresholds(&mut *self.conf, thresholds.as_mut_ptr()))
        }
    }
}
