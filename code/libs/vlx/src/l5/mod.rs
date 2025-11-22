pub mod platform;

use crate::bindings;
use crate::common::*;
use crate::esp::VlxI2cDriver;


/// Default I2C address of VL53L1X devices
pub const DEFAULT_ADDR: u16 = 0x29;

pub struct VL53L5CX {
    address: u8,  // Never equal to DEFAULT_ADDR
    conf: Box<bindings::VL53L5CX_Configuration>,
}

impl VL53L5CX {
    pub fn new(_driver: &VlxI2cDriver, address: u8) -> Self {
        if address as u16 == DEFAULT_ADDR {
            panic!("Cannot use reserved default VLX address");
        }

        // Zero-initialize configuration for now, will be properly initialized in init
        let conf = Box::new(bindings::VL53L5CX_Configuration::default());
        Self {
            address,
            conf,
        }
    }
}


impl VL53L5CX {
    /// Convert (x, y) coordinates into zone index
    fn get_zone_index(&self, x: usize, y: usize) -> Option<usize> {
        let (width, height) = (4, 4); //TODO Use configured resolution
        if x < width && y < height {
            Some(y * width + x)
        } else {
            None
        }
    }
}

impl VlxSensor for VL53L5CX {
    fn init(&mut self) -> Result<(), VlxError> {
        assert_ne!(self.address, 0x29);  // Note: already checked in constructor

        self.conf.platform.address = self.address as u16;
        let mut is_alive = 0u8;
        let status = unsafe { bindings::vl53l5cx_is_alive(&mut *self.conf, &mut is_alive) };
        if status != 0 || is_alive == 0 {
            // No device at the configured address, try the default address
            self.conf.platform.address = DEFAULT_ADDR;

            // Check if sensor is alive
            let status = unsafe { bindings::vl53l5cx_is_alive(&mut *self.conf, &mut is_alive) };
            if status != 0 || is_alive == 0 {
                return Err(VlxError::InitError);
            }

            // Initialize sensor
            let status = unsafe { bindings::vl53l5cx_init(&mut *self.conf) };
            if status != 0 {
                return Err(VlxError::InitError);
            }

            // Set final I2C address
            let status = unsafe { bindings::vl53l5cx_set_i2c_address(&mut *self.conf, (self.address * 2) as u16) };
            if status != 0 {
                return Err(VlxError::InitError);
            }
            self.conf.platform.address = self.address as u16;

            // Set 4x4 resolution for better performance
            // Note: 4x4 resolution is asumed in `get_zone_index()`
            let status = unsafe { bindings::vl53l5cx_set_resolution(&mut *self.conf, 16) }; // VL53L5CX_RESOLUTION_4X4
            if status != 0 {
                return Err(VlxError::InitError);
            }

            // Set power mode to wake up
            let status = unsafe { bindings::vl53l5cx_set_power_mode(&mut *self.conf, 1) }; // VL53L5CX_POWER_MODE_WAKEUP
            if status != 0 {
                return Err(VlxError::InitError);
            }
        }

        // Note: if device address is correct, assume it's already configured

        // Start ranging automatically
        let status = unsafe { bindings::vl53l5cx_start_ranging(&mut *self.conf) };
        if status != 0 {
            return Err(VlxError::InitError);
        }

        Ok(())
    }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {

        // Check if data is ready
        let mut is_data_ready = 0u8;
        let status = unsafe { bindings::vl53l5cx_check_data_ready(&mut *self.conf, &mut is_data_ready) };
        if status != 0 || is_data_ready == 0 {
            return Err(VlxError::RangingError);
        }

        // Get ranging data
        let mut results = bindings::VL53L5CX_ResultsData::default();
        let status = unsafe { bindings::vl53l5cx_get_ranging_data(&mut *self.conf, &mut results) };
        if status != 0 {
            return Err(VlxError::RangingError);
        }

        const MAX_ZONES: usize = 16;  // hardcoded for now, should be changed later

        // Extract all zone distances - VL53L5CX can be configured for different resolutions
        // Default 4x4 = 16 zones, but let's check the actual number of zones
        // Use u16::MAX for invalid measurement
        let mut distances = vec![u16::MAX; MAX_ZONES];

        // The number of zones depends on resolution setting
        // For 4x4 resolution: 16 zones (4x4 grid)
        // For 8x8 resolution: 64 zones (8x8 grid)

        for i in 0..MAX_ZONES {
            // Check if target is detected and status is valid (5 = valid measurement)
            if results.target_status[i] == 5 {
                distances[i] = results.distance_mm[i] as u16;
            }
        }

        // Déterminer les dimensions selon la résolution
        // Par défaut on a configuré 4x4 (16 zones)
        let (width, height) = match MAX_ZONES {
            16 => (4, 4),   // 4x4 résolution
            64 => (8, 8),   // 8x8 résolution
            _ => {
                // Au cas où on aurait une résolution différente, on essaie de deviner
                let size = (MAX_ZONES as f64).sqrt() as usize;
                if size * size == MAX_ZONES {
                    (size, size)
                } else {
                    // Fallback: on considère que c'est une ligne
                    (MAX_ZONES, 1)
                }
            }
        };

        Ok(DistanceData::new(width, height, distances))
    }

    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError> {
        // VL53L5CX has detection threshold functionality via plugins
        // Enable detection thresholds plugin for basic alarm functionality
        let status = unsafe { bindings::vl53l5cx_set_detection_thresholds_enable(&mut *self.conf, 1) };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }

        // Créer un tableau de seuils (64 max selon l'API)
        let mut detection_thresholds = [bindings::VL53L5CX_DetectionThresholds::default(); 64];
        let mut threshold_index = 0;

        // Déterminer les zones à configurer
        let zones_to_configure = if let Some((x, y)) = zone {
            // Zone spécifique
            match self.get_zone_index(x, y) {
                Some(idx) => vec![idx],
                None => return Err(VlxError::ConfigError),
            }
        } else {
            // Toutes les zones (4x4 = 16 zones par défaut)
            (0..16).collect()
        };

        // Configurer les seuils pour chaque zone
        for zone_idx in zones_to_configure {
            if zone_idx < 64 && threshold_index < 64 {
                // Configuration des seuils de distance
                detection_thresholds[threshold_index] = bindings::VL53L5CX_DetectionThresholds {
                    param_low_thresh: low as i32,
                    param_high_thresh: high as i32,
                    measurement: 1, // VL53L5CX_DISTANCE_MM
                    type_: 0,       // VL53L5CX_IN_WINDOW
                    zone_num: zone_idx as u8,
                    mathematic_operation: 0, // VL53L5CX_OPERATION_OR
                };
                threshold_index += 1;
            }
        }

        // Marquer la fin des seuils avec VL53L5CX_LAST_THRESHOLD
        if threshold_index < 64 {
            detection_thresholds[threshold_index].zone_num = 128; // VL53L5CX_LAST_THRESHOLD
        }

        // Appliquer la configuration des seuils
        let status = unsafe { bindings::vl53l5cx_set_detection_thresholds(&mut *self.conf, detection_thresholds.as_mut_ptr()) };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }

        Ok(())
    }

    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        // Enable detection thresholds plugin
        let status = unsafe { bindings::vl53l5cx_set_detection_thresholds_enable(&mut *self.conf, 1) };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }

        // Créer un tableau de seuils (64 max selon l'API)
        let mut detection_thresholds = [bindings::VL53L5CX_DetectionThresholds::default(); 64];
        let mut threshold_index = 0;

        // Configurer les seuils pour chaque alarme
        for alarm in alarms {
            if threshold_index >= 64 {
                break; // Limite atteinte
            }

            // Convertir les coordonnées en index de zone
            if let Some(zone_idx) = self.get_zone_index(alarm.zone.0, alarm.zone.1) {
                detection_thresholds[threshold_index] = bindings::VL53L5CX_DetectionThresholds {
                    param_low_thresh: alarm.low as i32,
                    param_high_thresh: alarm.high as i32,
                    measurement: 1, // VL53L5CX_DISTANCE_MM
                    type_: 0,       // VL53L5CX_IN_WINDOW
                    zone_num: zone_idx as u8,
                    mathematic_operation: 0, // VL53L5CX_OPERATION_OR
                };
                threshold_index += 1;
            }
        }

        // Marquer la fin des seuils avec VL53L5CX_LAST_THRESHOLD
        if threshold_index < 64 {
            detection_thresholds[threshold_index].zone_num = 128; // VL53L5CX_LAST_THRESHOLD
        }

        // Appliquer la configuration des seuils
        let status = unsafe { bindings::vl53l5cx_set_detection_thresholds(&mut *self.conf, detection_thresholds.as_mut_ptr()) };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }

        Ok(())
    }

    fn clear_alarm(&mut self) -> Result<(), VlxError> {
        // Disable detection thresholds plugin to clear all alarms
        let status = unsafe { bindings::vl53l5cx_set_detection_thresholds_enable(&mut *self.conf, 0) };
        if status != 0 {
            return Err(VlxError::ConfigError);
        }
        Ok(())
    }

}
