pub mod platform;

use crate::Sensor;
use core::marker::PhantomData;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub struct VL53L5CX<I2C> {
    i2c: I2C,
    address: u8,
    configuration: crate::VL53L5CX_Configuration,
    _phantom: PhantomData<I2C>,
}

impl<I2C> VL53L5CX<I2C> 
where
    I2C: Write + WriteRead + Send + Sync + 'static,
    <I2C as WriteRead>::Error: std::error::Error + Send + Sync + 'static,
    <I2C as Write>::Error: Into<<I2C as WriteRead>::Error>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        // Initialize configuration with zeros - will be properly initialized by vl53l5cx_init
        let configuration = unsafe { core::mem::zeroed::<crate::VL53L5CX_Configuration>() };
        
        Self {
            i2c,
            address,
            configuration,
            _phantom: PhantomData,
        }
    }
}


impl<I2C> VL53L5CX<I2C> {
    /// Convertit des coordonnées (x, y) en index de zone
    fn get_zone_index(&self, x: usize, y: usize) -> Option<usize> {
        // Détermine la résolution actuelle basée sur la configuration
        // Par défaut, on assume 4x4 (16 zones)
        let (width, height) = (4, 4); // TODO: récupérer la vraie résolution de la configuration
        
        if x < width && y < height {
            Some(y * width + x)
        } else {
            None
        }
    }
}

impl<I2C> Sensor for VL53L5CX<I2C> 
where
    I2C: Write + WriteRead + Send + Sync + 'static,
    <I2C as WriteRead>::Error: std::error::Error + Send + Sync + 'static,
    <I2C as Write>::Error: Into<<I2C as WriteRead>::Error>,
{
    type Error = VL53L5CXError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // Initialize configuration with default address (0x29)
            self.configuration.platform.address = 0x29;
            
            // Check if sensor is alive
            let mut is_alive = 0u8;
            let status = crate::vl53l5cx_is_alive(&mut self.configuration, &mut is_alive);
            if status != 0 || is_alive == 0 {
                return Err(VL53L5CXError::InitError);
            }
            
            // Initialize sensor
            let status = crate::vl53l5cx_init(&mut self.configuration);
            if status != 0 {
                return Err(VL53L5CXError::InitError);
            }
            
            // Set I2C address if different from default
            if self.address != 0x29 {
                let status = crate::vl53l5cx_set_i2c_address(&mut self.configuration, self.address as u16);
                if status != 0 {
                    return Err(VL53L5CXError::InitError);
                }
            }
            
            // Configure default settings
            // Set 4x4 resolution for better performance
            let status = crate::vl53l5cx_set_resolution(&mut self.configuration, 16); // VL53L5CX_RESOLUTION_4X4
            if status != 0 {
                return Err(VL53L5CXError::InitError);
            }
            
            // Set ranging frequency to 15Hz
            let status = crate::vl53l5cx_set_ranging_frequency_hz(&mut self.configuration, 15);
            if status != 0 {
                return Err(VL53L5CXError::InitError);
            }
            
            // Set power mode to wake up
            let status = crate::vl53l5cx_set_power_mode(&mut self.configuration, 1); // VL53L5CX_POWER_MODE_WAKEUP
            if status != 0 {
                return Err(VL53L5CXError::InitError);
            }
            
            // Start ranging automatically
            let status = crate::vl53l5cx_start_ranging(&mut self.configuration);
            if status != 0 {
                return Err(VL53L5CXError::InitError);
            }
        }
        Ok(())
    }
    
    fn get_distance(&mut self) -> Result<crate::DistanceData, Self::Error> {
        unsafe {
            let mut is_data_ready = 0u8;
            let mut results = core::mem::zeroed::<crate::VL53L5CX_ResultsData>();
            
            // Check if data is ready
            let status = crate::vl53l5cx_check_data_ready(&mut self.configuration, &mut is_data_ready);
            if status != 0 {
                return Err(VL53L5CXError::RangingError);
            }
            
            if is_data_ready == 0 {
                return Err(VL53L5CXError::RangingError);
            }
            
            // Get ranging data
            let status = crate::vl53l5cx_get_ranging_data(&mut self.configuration, &mut results);
            if status != 0 {
                return Err(VL53L5CXError::RangingError);
            }
            
            // Extract all zone distances - VL53L5CX can be configured for different resolutions
            // Default 4x4 = 16 zones, but let's check the actual number of zones
            let mut distances = Vec::new();
            
            // The number of zones depends on resolution setting
            // For 4x4 resolution: 16 zones (4x4 grid)
            // For 8x8 resolution: 64 zones (8x8 grid)
            let max_zones = if results.distance_mm.len() > 64 { 64 } else { results.distance_mm.len() };
            
            for i in 0..max_zones {
                // Check if target is detected and status is valid (5 = valid measurement)
                if results.nb_target_detected[i] > 0 && results.target_status[i] == 5 {
                    distances.push(results.distance_mm[i] as u16);
                } else {
                    // Push u16::MAX for invalid measurements to maintain zone indexing
                    distances.push(u16::MAX);
                }
            }
            
            if distances.is_empty() {
                Err(VL53L5CXError::RangingError)
            } else {
                // Déterminer les dimensions selon la résolution
                // Par défaut on a configuré 4x4 (16 zones)
                let (width, height) = match max_zones {
                    16 => (4, 4),   // 4x4 résolution
                    64 => (8, 8),   // 8x8 résolution
                    _ => {
                        // Au cas où on aurait une résolution différente, on essaie de deviner
                        let size = (max_zones as f64).sqrt() as usize;
                        if size * size == max_zones {
                            (size, size)
                        } else {
                            // Fallback: on considère que c'est une ligne
                            (max_zones, 1)
                        }
                    }
                };
                
                let multizone = crate::MultiZoneData::new(width, height, distances);
                Ok(crate::DistanceData::MultiZone(multizone))
            }
        }
    }
    
    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), Self::Error> {
        unsafe {
            // VL53L5CX has detection threshold functionality via plugins
            // Enable detection thresholds plugin for basic alarm functionality
            let status = crate::vl53l5cx_set_detection_thresholds_enable(&mut self.configuration, 1);
            if status != 0 {
                return Err(VL53L5CXError::ConfigError);
            }
            
            // Créer un tableau de seuils (64 max selon l'API)
            let mut detection_thresholds = [core::mem::zeroed::<crate::VL53L5CX_DetectionThresholds>(); 64];
            let mut threshold_index = 0;
            
            // Déterminer les zones à configurer
            let zones_to_configure = if let Some((x, y)) = zone {
                // Zone spécifique
                match self.get_zone_index(x, y) {
                    Some(idx) => vec![idx],
                    None => return Err(VL53L5CXError::ConfigError),
                }
            } else {
                // Toutes les zones (4x4 = 16 zones par défaut)
                (0..16).collect()
            };
            
            // Configurer les seuils pour chaque zone
            for zone_idx in zones_to_configure {
                if zone_idx < 64 && threshold_index < 64 {
                    // Configuration des seuils de distance
                    detection_thresholds[threshold_index] = crate::VL53L5CX_DetectionThresholds {
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
            let status = crate::vl53l5cx_set_detection_thresholds(&mut self.configuration, detection_thresholds.as_mut_ptr());
            if status != 0 {
                return Err(VL53L5CXError::ConfigError);
            }
        }
        Ok(())
    }
    
    fn set_multiple_alarms(&mut self, alarms: &[crate::ZoneAlarm]) -> Result<(), Self::Error> {
        unsafe {
            // Enable detection thresholds plugin
            let status = crate::vl53l5cx_set_detection_thresholds_enable(&mut self.configuration, 1);
            if status != 0 {
                return Err(VL53L5CXError::ConfigError);
            }
            
            // Créer un tableau de seuils (64 max selon l'API)
            let mut detection_thresholds = [core::mem::zeroed::<crate::VL53L5CX_DetectionThresholds>(); 64];
            let mut threshold_index = 0;
            
            // Configurer les seuils pour chaque alarme
            for alarm in alarms {
                if threshold_index >= 64 {
                    break; // Limite atteinte
                }
                
                // Convertir les coordonnées en index de zone
                if let Some(zone_idx) = self.get_zone_index(alarm.zone.0, alarm.zone.1) {
                    detection_thresholds[threshold_index] = crate::VL53L5CX_DetectionThresholds {
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
            let status = crate::vl53l5cx_set_detection_thresholds(&mut self.configuration, detection_thresholds.as_mut_ptr());
            if status != 0 {
                return Err(VL53L5CXError::ConfigError);
            }
        }
        Ok(())
    }
    
    fn clear_alarm(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // Disable detection thresholds plugin to clear all alarms
            let status = crate::vl53l5cx_set_detection_thresholds_enable(&mut self.configuration, 0);
            if status != 0 {
                return Err(VL53L5CXError::ConfigError);
            }
        }
        Ok(())
    }
    
}

#[derive(Debug)]
pub enum VL53L5CXError {
    I2cError,
    InitError,
    RangingError,
    ConfigError,
}