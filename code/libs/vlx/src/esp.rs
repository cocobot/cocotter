/// ESP-specific declarations, for the real stuff
use std::sync::{Arc, Mutex};
use std::{thread, time::Duration};
use embedded_hal::i2c::I2c;
use crate::common::*;
use crate::{l1, l5};


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
        i2c.write(address, &buffer)?;
        Ok(())
    }
    
    fn read(&mut self, address: u8, register: u16, data: &mut [u8]) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut i2c = self.0.lock().unwrap();
        let reg_bytes = register.to_be_bytes();
        i2c.write_read(address, &reg_bytes, data)?;
        Ok(())
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
pub(crate) fn call_i2c_write(address: u8, register: u16, data: &[u8]) -> bool {
    let mut global = match GLOBAL_SHARED_I2C.lock() {
        Ok(guard) => guard,
        Err(_) => return false, // Erreur de lock
    };
    
    if let Some(ref mut i2c) = *global {
        i2c.write(address, register, data).is_ok()
    } else {
        false // Pas d'I2C enregistré
    }
}

pub(crate) fn call_i2c_read(address: u8, register: u16, data: &mut [u8]) -> bool {
    let mut global = match GLOBAL_SHARED_I2C.lock() {
        Ok(guard) => guard,
        Err(_) => return false, // Erreur de lock
    };
    
    if let Some(ref mut i2c) = *global {
        i2c.read(address, register, data).is_ok()
    } else {
        false // Pas d'I2C enregistré
    }
}


// Wrapper I2C thread-safe et clonable (usage interne uniquement)
struct SharedI2c<I2C>(Arc<Mutex<I2C>>);


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
                (*reset_fn)(true);  // Assert reset
            }
        }
        
        // Small delay to ensure reset is stable
        thread::sleep(Duration::from_millis(10));
        
        // Phase 2: Release reset for all sensors
        for (i, _) in self.sensors.iter().enumerate() {
            if let Some(ref mut reset_fn) = self.configs[i].reset_fn {
                (*reset_fn)(false);   // Release reset
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
                SensorInstance::L5(sensor) => sensor.init().map_err(|_| VlxError::InitError),
                SensorInstance::L1(sensor) => sensor.init().map_err(|_| VlxError::InitError),
            };

            if r.is_err() {
                log::error!("Failed to initialize sensor {}: {:?}", i + 1, r);
                res = r;
            }
        }
        res
    }

    fn get_sensor_mut(&mut self, sensor_index: usize) -> Result<&mut dyn Sensor, VlxError> {
        match self.sensors.get_mut(sensor_index).ok_or(VlxError::InvalidSensor)? {
            SensorInstance::L1(sensor) => Ok(sensor),
            SensorInstance::L5(sensor) => Ok(sensor),
        }
    }

    pub fn get_distance(&mut self, sensor_index: usize) -> Result<DistanceData, VlxError> {
        self.get_sensor_mut(sensor_index)?.get_distance()
    }
    
    pub fn set_alarm(&mut self, sensor_index: usize, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError> {
        self.get_sensor_mut(sensor_index)?.set_alarm(low, high, zone)
    }
    
    pub fn set_multiple_alarms(&mut self, sensor_index: usize, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        self.get_sensor_mut(sensor_index)?.set_multiple_alarms(alarms)
    }
    
    pub fn clear_alarm(&mut self, sensor_index: usize) -> Result<(), VlxError> {
        self.get_sensor_mut(sensor_index)?.clear_alarm()
    }
    
    pub const fn sensor_count(&self) -> usize {
        N
    }
    
}

