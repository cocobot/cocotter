pub mod l1;
pub mod l5;

use std::sync::{Arc, Mutex};
use std::{thread, time::Duration};
use embedded_hal::i2c::I2c;

mod bindings;
use bindings::*;

// Stockage global du SharedI2c pour les fonctions platform C
static GLOBAL_SHARED_I2C: Mutex<Option<Box<dyn I2cWrapper>>> = Mutex::new(None);

// Trait wrapper pour permettre le stockage d'objets I2C hétérogènes
trait I2cWrapper: Send + Sync {
    fn write(&mut self, address: u8, register: u16, data: &[u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>>;
    fn read(&mut self, address: u8, register: u16, data: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>>;
}

// Implémentation du wrapper pour SharedI2c
impl<I2C, E> I2cWrapper for SharedI2c<I2C>
where
    I2C: I2c<Error = E> + Send + Sync,
    E: std::error::Error + Send + Sync + 'static,
{
    fn write(&mut self, address: u8, register: u16, data: &[u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut i2c = self.0.lock().unwrap();
        let reg_bytes = register.to_be_bytes();
        let mut buffer = Vec::with_capacity(reg_bytes.len() + data.len());
        buffer.extend_from_slice(&reg_bytes);
        buffer.extend_from_slice(data);
        i2c.write(address, &buffer).map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)
    }
    
    fn read(&mut self, address: u8, register: u16, data: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut i2c = self.0.lock().unwrap();
        let reg_bytes = register.to_be_bytes();
        i2c.write_read(address, &reg_bytes, data).map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)
    }
}

// Fonction pour enregistrer le SharedI2c global
fn register_global_i2c<I2C, E>(i2c: I2C) 
where
    I2C: I2c<Error = E> + Send + Sync + 'static,
    E: std::error::Error + Send + Sync + 'static,
{
    let shared_i2c = SharedI2c(Arc::new(Mutex::new(i2c)));
    let mut global = GLOBAL_SHARED_I2C.lock().unwrap();
    *global = Some(Box::new(shared_i2c));
}

// Fonctions pour utiliser l'I2C global depuis les platforms
pub unsafe fn call_i2c_write(address: u8, register: u16, data: &[u8]) -> i8 {
    let mut global = match GLOBAL_SHARED_I2C.lock() {
        Ok(guard) => guard,
        Err(_) => return -1, // Erreur de lock
    };
    
    if let Some(ref mut i2c) = *global {
        match i2c.write(address, register, data) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    } else {
        -1 // Pas d'I2C enregistré
    }
}

pub unsafe fn call_i2c_read(address: u8, register: u16, data: &mut [u8]) -> i8 {
    let mut global = match GLOBAL_SHARED_I2C.lock() {
        Ok(guard) => guard,
        Err(_) => return -1, // Erreur de lock
    };
    
    if let Some(ref mut i2c) = *global {
        match i2c.read(address, register, data) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    } else {
        -1 // Pas d'I2C enregistré
    }
}


// Wrapper I2C thread-safe et clonable (usage interne uniquement)
struct SharedI2c<I2C>(Arc<Mutex<I2C>>);
/*
impl<I2C> Clone for SharedI2c<I2C> {
    fn clone(&self) -> Self {
        SharedI2c(self.0.clone())
    }
}

unsafe impl<I2C> Send for SharedI2c<I2C> where I2C: Send {}
unsafe impl<I2C> Sync for SharedI2c<I2C> where I2C: Send {}

impl<I2C, E> Write for SharedI2c<I2C> 
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    type Error = E;
    
    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut i2c = self.0.lock().unwrap();
        i2c.write(address, bytes)
    }
}

impl<I2C, E> WriteRead for SharedI2c<I2C> 
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    type Error = E;
    
    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut i2c = self.0.lock().unwrap();
        i2c.write_read(address, bytes, buffer)
    }
}

impl<I2C, E> I2c for SharedI2c<I2C> 
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    E: Into<Box<dyn std::error::Error + Send + Sync>>,
{
    type Error = Box<dyn std::error::Error + Send + Sync>;
    
    fn write(&mut self, address: u8, register: u16, data: &[u8]) -> Result<(), Self::Error> {
        let mut i2c = self.0.lock().unwrap();
        let reg_bytes = register.to_be_bytes();

        //gen one buffer with register and data
        let mut buffer = Vec::with_capacity(reg_bytes.len() + data.len());
        buffer.extend_from_slice(&reg_bytes);
        buffer.extend_from_slice(data);

        // Use the I2C write method
        i2c.write(address, &buffer).map_err(|e| e.into())
    }
    
    fn read(&mut self, address: u8, register: u16, data: &mut [u8]) -> Result<(), Self::Error> {
        let mut i2c = self.0.lock().unwrap();
        let reg_bytes = register.to_be_bytes();
        i2c.write_read(address, &reg_bytes, data).map_err(|e| e.into())
    }
}
*/


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

enum SensorInstance {
    L1(l1::VL53L1X),
    L5(l5::VL53L5CX),
}

pub struct VLX<I2C, const N: usize>
{
    configs: [Config<Box<dyn FnMut(bool)>, Box<dyn FnMut(bool)>>; N],
    sensors: [SensorInstance; N],
    _phantom: std::marker::PhantomData<I2C>,
}

impl<I2C, E, const N: usize> VLX<I2C, N> 
where
    I2C: I2c<Error = E> + Send + Sync + 'static,
    E: std::error::Error + Send + Sync + 'static,
{
    pub fn new(i2c: I2C, configs: [Config<Box<dyn FnMut(bool)>, Box<dyn FnMut(bool)>>; N]) -> Self {
        register_global_i2c(i2c);

        let sensors = core::array::from_fn(|i| {
            match configs[i].sensor_type {
                SensorType::L1 => SensorInstance::L1(l1::VL53L1X::new(configs[i].i2c_address)),
                SensorType::L5 => SensorInstance::L5(l5::VL53L5CX::new(configs[i].i2c_address)),
            }
        });
        
        Self {
            configs,
            sensors,
            _phantom: std::marker::PhantomData,
        }
    }
    
    pub fn init(&mut self) -> Result<(), VlxError> {
        // Phase 1: Reset all sensors and clear all enables
        for (i, _) in self.sensors.iter().enumerate() {
            // Clear enable first
            if let Some(ref mut enable_fn) = self.configs[i].enable_fn {
                (*enable_fn)(false);
            }
            
            // Reset sensor
            if let Some(ref mut reset_fn) = self.configs[i].reset_fn {
                (*reset_fn)(false);  // Assert reset
            }
        }
        
        // Small delay to ensure reset is stable
        thread::sleep(Duration::from_millis(10));
        
        // Phase 2: Release reset for all sensors
        for (i, _) in self.sensors.iter().enumerate() {
            if let Some(ref mut reset_fn) = self.configs[i].reset_fn {
                (*reset_fn)(true);   // Release reset
            }
        }
        
        // Small delay after reset release
        thread::sleep(Duration::from_millis(10));
        
        let mut res = Ok(());
        // Phase 3: Enable and initialize each sensor sequentially
        for (i, sensor) in self.sensors.iter_mut().enumerate() {
            if let Some(ref mut enable_fn) = self.configs[i].enable_fn {
                (*enable_fn)(true);
            }
            
            // Small delay after enable
            thread::sleep(Duration::from_millis(100));
            
            let r = match sensor {
                SensorInstance::L1(sensor) => sensor.init().map_err(|_| VlxError::InitError),
                SensorInstance::L5(sensor) => sensor.init().map_err(|_| VlxError::InitError),
            };

            if r.is_err() {
                log::error!("Failed to initialize sensor {}: {:?}", i + 1, r);
                res = r;
            }
        }
        res
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
    
    pub fn sensor_count(&self) -> usize {
        N
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