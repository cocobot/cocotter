pub mod platform;

use crate::Sensor;
use core::marker::PhantomData;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub struct VL53L1X<I2C> {
    i2c: I2C,
    address: u8,
    _phantom: PhantomData<I2C>,
}

impl<I2C> VL53L1X<I2C> 
where
    I2C: Write + WriteRead + Send + Sync + 'static,
    <I2C as WriteRead>::Error: std::error::Error + Send + Sync + 'static,
    <I2C as Write>::Error: Into<<I2C as WriteRead>::Error>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            _phantom: PhantomData,
        }
    }
}


impl<I2C> Sensor for VL53L1X<I2C> 
where
    I2C: Write + WriteRead + Send + Sync + 'static,
    <I2C as WriteRead>::Error: std::error::Error + Send + Sync + 'static,
    <I2C as Write>::Error: Into<<I2C as WriteRead>::Error>,
{
    type Error = VL53L1XError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // Start with default I2C address (0x29)
            let default_addr = 0x29u16;
            
            // Initialize sensor with default settings using default address
            let status = crate::VL53L1X_SensorInit(default_addr);
            if status != 0 {
                return Err(VL53L1XError::InitError);
            }
            
            // Wait for sensor to boot
            let mut boot_state = 0u8;
            loop {
                let status = crate::VL53L1X_BootState(default_addr, &mut boot_state);
                if status != 0 {
                    return Err(VL53L1XError::InitError);
                }
                if boot_state == 1 {
                    break;
                }
                // Small delay - in real implementation, use proper delay
                for _ in 0..10000 { core::hint::spin_loop(); }
            }
            
            // Verify sensor ID
            let mut sensor_id = 0u16;
            let status = crate::VL53L1X_GetSensorId(default_addr, &mut sensor_id);
            if status != 0 || sensor_id != 0xEEAC {
                return Err(VL53L1XError::InitError);
            }
            
            // Set the new I2C address if different from default
            if self.address != 0x29 {
                let status = crate::VL53L1X_SetI2CAddress(default_addr, self.address);
                if status != 0 {
                    return Err(VL53L1XError::InitError);
                }
            }
            
            // Continue configuration with the final address
            let final_addr = self.address as u16;
            
            // Configure sensor with default settings
            let status = crate::VL53L1X_SetDistanceMode(final_addr, 2); // Long range mode
            if status != 0 {
                return Err(VL53L1XError::InitError);
            }
            
            let status = crate::VL53L1X_SetTimingBudgetInMs(final_addr, 100); // 100ms timing budget
            if status != 0 {
                return Err(VL53L1XError::InitError);
            }
            
            let status = crate::VL53L1X_SetInterMeasurementInMs(final_addr, 100); // 100ms interval
            if status != 0 {
                return Err(VL53L1XError::InitError);
            }
            
            // Start ranging automatically
            let status = crate::VL53L1X_StartRanging(final_addr);
            if status != 0 {
                return Err(VL53L1XError::InitError);
            }
        }
        Ok(())
    }
    
    fn get_distance(&mut self) -> Result<crate::DistanceData, Self::Error> {
        unsafe {
            let mut is_data_ready = 0u8;
            let mut distance = 0u16;
            let mut range_status = 0u8;
            
            // Check if data is ready
            let status = crate::VL53L1X_CheckForDataReady(self.address as u16, &mut is_data_ready);
            if status != 0 {
                return Err(VL53L1XError::RangingError);
            }
            
            if is_data_ready == 0 {
                return Err(VL53L1XError::RangingError);
            }
            
            // Get range status first to check measurement validity
            let status = crate::VL53L1X_GetRangeStatus(self.address as u16, &mut range_status);
            if status != 0 {
                return Err(VL53L1XError::RangingError);
            }
            
            // Check if measurement is valid (0 = no error)
            if range_status != 0 {
                // Clear interrupt even for invalid measurements
                let _ = crate::VL53L1X_ClearInterrupt(self.address as u16);
                return Err(VL53L1XError::RangingError);
            }
            
            // Get distance measurement
            let status = crate::VL53L1X_GetDistance(self.address as u16, &mut distance);
            if status != 0 {
                return Err(VL53L1XError::RangingError);
            }
            
            // Clear interrupt to arm for next measurement
            let status = crate::VL53L1X_ClearInterrupt(self.address as u16);
            if status != 0 {
                return Err(VL53L1XError::RangingError);
            }
            
            Ok(crate::DistanceData::Single(distance))
        }
    }
    
    fn set_alarm(&mut self, low: u16, high: u16, _zone: Option<(usize, usize)>) -> Result<(), Self::Error> {
        unsafe {
            // VL53L1X has threshold detection functionality
            // Set distance threshold for interrupt generation
            let status = crate::VL53L1X_SetDistanceThreshold(
                self.address as u16,
                low,   // Low threshold in mm
                high,  // High threshold in mm  
                3,     // Window mode: 0=below low, 1=above high, 2=out of window, 3=in window
                1      // Interrupt on new sample ready
            );
            if status != 0 {
                return Err(VL53L1XError::ConfigError);
            }
        }
        Ok(())
    }
    
    fn set_multiple_alarms(&mut self, alarms: &[crate::ZoneAlarm]) -> Result<(), Self::Error> {
        // VL53L1X is single zone, so we just use the first alarm if any
        if let Some(alarm) = alarms.first() {
            self.set_alarm(alarm.low, alarm.high, Some(alarm.zone))
        } else {
            Ok(())
        }
    }
    
    fn clear_alarm(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // Disable threshold detection by setting very wide thresholds
            let status = crate::VL53L1X_SetDistanceThreshold(
                self.address as u16,
                0,          // Low threshold = 0mm
                u16::MAX,   // High threshold = max range
                3,          // Window mode: in window (always triggered)
                0           // Disable interrupt
            );
            if status != 0 {
                return Err(VL53L1XError::ConfigError);
            }
        }
        Ok(())
    }
    
}

#[derive(Debug)]
pub enum VL53L1XError {
    I2cError,
    InitError,
    RangingError,
    ConfigError,
}