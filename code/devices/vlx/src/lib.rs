pub mod l1;
pub mod l5;

use core::marker::PhantomData;

// Include generated C bindings
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

/// I2C trait that sensor drivers need to implement
pub trait I2c {
    type Error;
    
    /// Write data to a register
    fn write(&mut self, addr: u8, reg: u16, data: &[u8]) -> Result<(), Self::Error>;
    
    /// Read data from a register  
    fn read(&mut self, addr: u8, reg: u16, data: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorType {
    L1,
    L5,
}

pub struct Config<EN, RST> {
    pub i2c_address: u8,
    pub sensor_type: SensorType,
    pub enable_fn: Option<EN>,
    pub reset_fn: Option<RST>,
}

/// Configuration d'alarme pour une zone spécifique
#[derive(Debug, Clone)]
pub struct ZoneAlarm {
    pub zone: (usize, usize),  // Coordonnées (x, y) de la zone
    pub low: u16,              // Seuil bas en mm
    pub high: u16,             // Seuil haut en mm
}

pub trait Sensor {
    type Error;
    
    fn init(&mut self) -> Result<(), Self::Error>;
    fn get_distance(&mut self) -> Result<DistanceData, Self::Error>;
    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), Self::Error>;
    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), Self::Error>;
    fn clear_alarm(&mut self) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone)]
pub struct MultiZoneData {
    pub width: usize,      // Largeur de la grille (ex: 4 pour 4x4)
    pub height: usize,     // Hauteur de la grille (ex: 4 pour 4x4)
    pub distances: Vec<u16>, // Données stockées row-major (ligne par ligne)
}

impl MultiZoneData {
    pub fn new(width: usize, height: usize, distances: Vec<u16>) -> Self {
        assert_eq!(width * height, distances.len(), "Dimensions incompatibles avec les données");
        Self { width, height, distances }
    }
    
    /// Accès à une zone par coordonnées (x, y)
    pub fn get(&self, x: usize, y: usize) -> Option<u16> {
        if x < self.width && y < self.height {
            Some(self.distances[y * self.width + x])
        } else {
            None
        }
    }
    
    /// Modifie une zone par coordonnées (x, y)
    pub fn set(&mut self, x: usize, y: usize, value: u16) -> bool {
        if x < self.width && y < self.height {
            self.distances[y * self.width + x] = value;
            true
        } else {
            false
        }
    }
    
    /// Retourne les dimensions
    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }
    
    /// Retourne toutes les distances sous forme de grille 2D
    pub fn as_2d_vec(&self) -> Vec<Vec<u16>> {
        let mut result = Vec::with_capacity(self.height);
        for row in 0..self.height {
            let mut row_data = Vec::with_capacity(self.width);
            for col in 0..self.width {
                row_data.push(self.distances[row * self.width + col]);
            }
            result.push(row_data);
        }
        result
    }
}

#[derive(Debug, Clone)]
pub enum DistanceData {
    Single(u16),              // Pour VL53L1X - une seule distance
    MultiZone(MultiZoneData), // Pour VL53L5CX - grille 2D de distances
}

impl DistanceData {
    /// Retourne la première distance valide (pour compatibilité avec l'usage simple)
    pub fn first_distance(&self) -> Option<u16> {
        match self {
            DistanceData::Single(distance) => {
                if *distance == u16::MAX { None } else { Some(*distance) }
            }
            DistanceData::MultiZone(multizone) => {
                multizone.distances.iter().find(|&&d| d != u16::MAX).copied()
            }
        }
    }
    
    /// Retourne toutes les distances sous forme de Vec linéaire
    pub fn all_distances(&self) -> Vec<u16> {
        match self {
            DistanceData::Single(distance) => vec![*distance],
            DistanceData::MultiZone(multizone) => multizone.distances.clone(),
        }
    }
    
    /// Retourne le nombre de zones
    pub fn zone_count(&self) -> usize {
        match self {
            DistanceData::Single(_) => 1,
            DistanceData::MultiZone(multizone) => multizone.distances.len(),
        }
    }
    
    /// Retourne les dimensions (largeur, hauteur)
    pub fn dimensions(&self) -> (usize, usize) {
        match self {
            DistanceData::Single(_) => (1, 1),
            DistanceData::MultiZone(multizone) => multizone.dimensions(),
        }
    }
    
    /// Accès à une zone par coordonnées (x, y) - retourne None pour Single si x>0 ou y>0
    pub fn get(&self, x: usize, y: usize) -> Option<u16> {
        match self {
            DistanceData::Single(distance) => {
                if x == 0 && y == 0 { Some(*distance) } else { None }
            }
            DistanceData::MultiZone(multizone) => multizone.get(x, y),
        }
    }
}

pub struct VLX<I2C, const N: usize, EN, RST> 
where
    I2C: Clone + crate::I2c + Send + Sync + 'static,
    I2C::Error: std::error::Error + Send + Sync + 'static,
{
    i2c: I2C,
    configs: [Config<EN, RST>; N],
    sensors: [SensorInstance<I2C>; N],
    _phantom: PhantomData<(EN, RST)>,
}

enum SensorInstance<I2C> {
    L1(l1::VL53L1X<I2C>),
    L5(l5::VL53L5CX<I2C>),
}

impl<I2C, const N: usize, EN, RST> VLX<I2C, N, EN, RST> 
where
    I2C: Clone + crate::I2c + Send + Sync + 'static,
    I2C::Error: std::error::Error + Send + Sync + 'static,
    EN: Fn(bool),
    RST: Fn(bool),
{
    pub fn new(i2c: I2C, configs: [Config<EN, RST>; N]) -> Self {
        let sensors = core::array::from_fn(|i| {
            match configs[i].sensor_type {
                SensorType::L1 => SensorInstance::L1(l1::VL53L1X::new(i2c.clone(), configs[i].i2c_address)),
                SensorType::L5 => SensorInstance::L5(l5::VL53L5CX::new(i2c.clone(), configs[i].i2c_address)),
            }
        });
        
        Self {
            i2c,
            configs,
            sensors,
            _phantom: PhantomData,
        }
    }
    
    pub fn init(&mut self) -> Result<(), VlxError> {
        // Phase 1: Reset all sensors and clear all enables
        for (i, _) in self.sensors.iter().enumerate() {
            // Clear enable first
            if let Some(ref enable_fn) = self.configs[i].enable_fn {
                enable_fn(false);
            }
            
            // Reset sensor
            if let Some(ref reset_fn) = self.configs[i].reset_fn {
                reset_fn(false);  // Assert reset
            }
        }
        
        // Small delay to ensure reset is stable
        for _ in 0..50000 { core::hint::spin_loop(); }
        
        // Phase 2: Release reset for all sensors
        for (i, _) in self.sensors.iter().enumerate() {
            if let Some(ref reset_fn) = self.configs[i].reset_fn {
                reset_fn(true);   // Release reset
            }
        }
        
        // Small delay after reset release
        for _ in 0..50000 { core::hint::spin_loop(); }
        
        // Phase 3: Enable and initialize each sensor sequentially
        for (i, sensor) in self.sensors.iter_mut().enumerate() {
            if let Some(ref enable_fn) = self.configs[i].enable_fn {
                enable_fn(true);
            }
            
            // Small delay after enable
            for _ in 0..10000 { core::hint::spin_loop(); }
            
            match sensor {
                SensorInstance::L1(sensor) => sensor.init().map_err(|_| VlxError::InitError)?,
                SensorInstance::L5(sensor) => sensor.init().map_err(|_| VlxError::InitError)?,
            }
        }
        Ok(())
    }
    
    pub fn get_distance(&mut self, sensor_index: usize) -> Result<DistanceData, VlxError> {
        if sensor_index >= N {
            return Err(VlxError::InvalidSensor);
        }
        
        match &mut self.sensors[sensor_index] {
            SensorInstance::L1(sensor) => sensor.get_distance().map_err(|_| VlxError::ReadError),
            SensorInstance::L5(sensor) => sensor.get_distance().map_err(|_| VlxError::ReadError),
        }
    }
    
    pub fn set_alarm(&mut self, sensor_index: usize, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError> {
        if sensor_index >= N {
            return Err(VlxError::InvalidSensor);
        }
        
        match &mut self.sensors[sensor_index] {
            SensorInstance::L1(sensor) => sensor.set_alarm(low, high, zone).map_err(|_| VlxError::ConfigError),
            SensorInstance::L5(sensor) => sensor.set_alarm(low, high, zone).map_err(|_| VlxError::ConfigError),
        }
    }
    
    pub fn set_multiple_alarms(&mut self, sensor_index: usize, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        if sensor_index >= N {
            return Err(VlxError::InvalidSensor);
        }
        
        match &mut self.sensors[sensor_index] {
            SensorInstance::L1(sensor) => sensor.set_multiple_alarms(alarms).map_err(|_| VlxError::ConfigError),
            SensorInstance::L5(sensor) => sensor.set_multiple_alarms(alarms).map_err(|_| VlxError::ConfigError),
        }
    }
    
    pub fn clear_alarm(&mut self, sensor_index: usize) -> Result<(), VlxError> {
        if sensor_index >= N {
            return Err(VlxError::InvalidSensor);
        }
        
        match &mut self.sensors[sensor_index] {
            SensorInstance::L1(sensor) => sensor.clear_alarm().map_err(|_| VlxError::ConfigError),
            SensorInstance::L5(sensor) => sensor.clear_alarm().map_err(|_| VlxError::ConfigError),
        }
    }
    
}

#[derive(Debug)]
pub enum VlxError {
    InvalidSensor,
    InitError,
    ReadError,
    ConfigError,
    RangingError,
}