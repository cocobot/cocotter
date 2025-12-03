mod platform;

use std::{thread, time::Duration};
use crate::bindings::*;
use crate::{DistanceData, VlxSensor, VlxError, VlxI2cDriver, ZoneAlarm};


/// Default I2C address of VL53L1X devices
pub const DEFAULT_ADDR: u16 = 0x29;

macro_rules! status_result {
    ($expr:expr) => {
        match unsafe { $expr } {
            0 => Ok(()),
            _ => Err(VlxError::LowLevelError),
        }
    }
}


pub struct VL53L1X {
    address: u8,
}

impl VL53L1X {
    pub fn new(_driver: &VlxI2cDriver, address: u8) -> Self {
        Self {
            address,
        }
    }

    fn set_distance_threshold(&mut self, low: u16, high: u16, interrupt: bool) -> Result<(), VlxError> {
        // VL53L1X has threshold detection functionality
        // Set distance threshold for interrupt generation
        status_result!(
            VL53L1X_SetDistanceThreshold(
                self.address as u16,
                low,              // Low threshold in mm
                high,             // High threshold in mm
                3,                // Window mode: 0=below low, 1=above high, 2=out of window, 3=in window
                interrupt as u8,  // Interrupt on new sample ready
            )
        )
    }
}


impl VlxSensor for VL53L1X {
    fn init(&mut self) -> Result<(), VlxError> {
        assert_ne!(self.address as u16, DEFAULT_ADDR, "VL53L1X cannot use default address, please set a different address");

        // Check if alreay initialized
        let mut sensor_id = 0u16;
        let status = unsafe { VL53L1X_GetSensorId(self.address as u16, &mut sensor_id) };
        if status == 0 && sensor_id != 0xEACC {
            return Ok(());
        }

        // Start with default I2C address
        status_result!(VL53L1X_GetSensorId(DEFAULT_ADDR, &mut sensor_id))?;
        if sensor_id != 0xEACC {
            return Err(VlxError::SensorNotFound);
        }

        // Wait for sensor to boot
        loop {
            let mut boot_state = 0u8;
            status_result!(VL53L1X_BootState(DEFAULT_ADDR, &mut boot_state))?;
            if boot_state != 0 {
                break;
            }
            // Small delay
            thread::sleep(Duration::from_millis(10));
        }

        // Initialize sensor with default settings using default address
        status_result!(VL53L1X_SensorInit(DEFAULT_ADDR))?;

        // Set the new I2C address
        status_result!(VL53L1X_SetI2CAddress(DEFAULT_ADDR, self.address * 2))?;

        // Continue configuration with the final address
        let final_addr = self.address as u16;

        // Configure sensor with default settings

        // Long range mode
        status_result!(VL53L1X_SetDistanceMode(final_addr, 1))?;
        // 100ms timing budget
        status_result!(VL53L1X_SetTimingBudgetInMs(final_addr, 100))?;
        // 100ms interval
        status_result!(VL53L1X_SetInterMeasurementInMs(final_addr, 100))?;
        // Start ranging automatically
        status_result!(VL53L1X_StartRanging(final_addr))?;

        Ok(())
    }

    fn get_distance(&mut self) -> Result<crate::DistanceData, VlxError> {
        let mut is_data_ready = 0u8;
        let mut distance = 0u16;
        let mut range_status = 0u8;

        // Check if data is ready
        status_result!(VL53L1X_CheckForDataReady(self.address as u16, &mut is_data_ready))?;
        if is_data_ready == 0 {
            return Err(VlxError::DataNotReady);
        }

        // Get range status first to check measurement validity
        status_result!(VL53L1X_GetRangeStatus(self.address as u16, &mut range_status))?;
        // Check if measurement is valid (0 = no error)
        if range_status != 0 {
            // Clear interrupt even for invalid measurements
            let _ = unsafe { VL53L1X_ClearInterrupt(self.address as u16) };
            return Err(VlxError::InvalidMeasure);
        }

        // Get distance measurement
        status_result!(VL53L1X_GetDistance(self.address as u16, &mut distance))?;
        // Clear interrupt to arm for next measurement
        status_result!(VL53L1X_ClearInterrupt(self.address as u16))?;

        Ok(DistanceData::new_single(distance))
    }

    fn set_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        if alarms.len() > 1 {
            Err(VlxError::TooManyAlarms)
        } else if alarms.is_empty() {
            // Disable threshold detection by setting very wide thresholds
            self.set_distance_threshold(0, u16::MAX, false)
        } else {
            let alarm = &alarms[0];
            self.set_distance_threshold(alarm.low, alarm.high, true)
        }
    }
}
