pub mod platform;

use crate::bindings;
use crate::{VlxSensor, VlxError};
use crate::esp::VlxI2cDriver;
use std::{thread, time::Duration};


/// Default I2C address of VL53L1X devices
pub const DEFAULT_ADDR: u16 = 0x29;

pub struct VL53L1X {
    address: u8,
}

impl VL53L1X {
    pub fn new(_driver: &VlxI2cDriver, address: u8) -> Self {
        Self {
            address,
        }
    }
}


impl VlxSensor for VL53L1X {
    fn init(&mut self) -> Result<(), VlxError> {
        assert_ne!(self.address as u16, DEFAULT_ADDR, "VL53L1X cannot use default address, please set a different address");

        // Check if alreay initialized
        let mut sensor_id = 0u16;
        let status = unsafe { bindings::VL53L1X_GetSensorId(self.address as u16, &mut sensor_id) };
        if status == 0 && sensor_id != 0xEACC {
            return Ok(());
        }

        // Start with default I2C address (0x29)
        let status = unsafe { bindings::VL53L1X_GetSensorId(DEFAULT_ADDR, &mut sensor_id) };
        if status != 0 || sensor_id != 0xEACC {
            return Err(VlxError::InitError);
        }

        // Wait for sensor to boot
        loop {
            let mut boot_state = 0u8;
            let status = unsafe { bindings::VL53L1X_BootState(DEFAULT_ADDR, &mut boot_state) };
            if status != 0 {
                return Err(VlxError::InitError);
            }
            if boot_state != 0 {
                break;
            }

            // Small delay
            thread::sleep(Duration::from_millis(10));
        }

        // Initialize sensor with default settings using default address
        let status = unsafe { bindings::VL53L1X_SensorInit(DEFAULT_ADDR) };
        if status != 0 {
            return Err(VlxError::InitError);
        }


        // Set the new I2C address
        let status = unsafe { bindings::VL53L1X_SetI2CAddress(DEFAULT_ADDR, self.address * 2) };
        if status != 0 {
            return Err(VlxError::InitError);
        }

        // Continue configuration with the final address
        let final_addr = self.address as u16;

        // Configure sensor with default settings
        let status = unsafe { bindings::VL53L1X_SetDistanceMode(final_addr, 1) }; // Long range mode
        if status != 0 {
            return Err(VlxError::InitError);
        }

        let status = unsafe { bindings::VL53L1X_SetTimingBudgetInMs(final_addr, 100) }; // 100ms timing budget
        if status != 0 {
            return Err(VlxError::InitError);
        }

        let status = unsafe { bindings::VL53L1X_SetInterMeasurementInMs(final_addr, 100) }; // 100ms interval
        if status != 0 {
            return Err(VlxError::InitError);
        }

        // Start ranging automatically
        let status = unsafe { bindings::VL53L1X_StartRanging(final_addr) };
        if status != 0 {
            return Err(VlxError::InitError);
        }
        Ok(())
    }

    fn get_distance(&mut self) -> Result<crate::DistanceData, VlxError> {
        let mut is_data_ready = 0u8;
        let mut distance = 0u16;
        let mut range_status = 0u8;

        // Check if data is ready
        let status = unsafe { bindings::VL53L1X_CheckForDataReady(self.address as u16, &mut is_data_ready) };
        if status != 0 {
            return Err(VlxError::RangingError);
        }

        if is_data_ready == 0 {
            return Err(VlxError::RangingError);
        }

        // Get range status first to check measurement validity
        let status = unsafe { bindings::VL53L1X_GetRangeStatus(self.address as u16, &mut range_status) };
        if status != 0 {
            return Err(VlxError::RangingError);
        }

        // Check if measurement is valid (0 = no error)
        if range_status != 0 {
            // Clear interrupt even for invalid measurements
            let _ = unsafe { bindings::VL53L1X_ClearInterrupt(self.address as u16) };
            return Err(VlxError::RangingError);
        }

        // Get distance measurement
        let status = unsafe { bindings::VL53L1X_GetDistance(self.address as u16, &mut distance) };
        if status != 0 {
            return Err(VlxError::RangingError);
        }

        // Clear interrupt to arm for next measurement
        let status = unsafe { bindings::VL53L1X_ClearInterrupt(self.address as u16) };
        if status != 0 {
            return Err(VlxError::RangingError);
        }

        Ok(crate::DistanceData::new_single(distance))
    }

    fn set_alarm(&mut self, low: u16, high: u16, _zone: Option<(usize, usize)>) -> Result<(), VlxError> {
        // VL53L1X has threshold detection functionality
        // Set distance threshold for interrupt generation
        let status = unsafe {
            bindings::VL53L1X_SetDistanceThreshold(
                self.address as u16,
                low,   // Low threshold in mm
                high,  // High threshold in mm
                3,     // Window mode: 0=below low, 1=above high, 2=out of window, 3=in window
                1      // Interrupt on new sample ready
            )
        };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }
        Ok(())
    }

    fn set_multiple_alarms(&mut self, alarms: &[crate::ZoneAlarm]) -> Result<(), VlxError> {
        // VL53L1X is single zone, so we just use the first alarm if any
        if let Some(alarm) = alarms.first() {
            self.set_alarm(alarm.low, alarm.high, Some(alarm.zone))
        } else {
            Ok(())
        }
    }

    fn clear_alarm(&mut self) -> Result<(), VlxError> {
        // Disable threshold detection by setting very wide thresholds
        let status = unsafe {
            bindings::VL53L1X_SetDistanceThreshold(
                self.address as u16,
                0,          // Low threshold = 0mm
                u16::MAX,   // High threshold = max range
                3,          // Window mode: in window (always triggered)
                0           // Disable interrupt
            )
        };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }
        Ok(())
    }

}
