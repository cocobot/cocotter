use simple_logger::SimpleLogger;
use vlx::{VLX, Config, SensorType, Sensor, DistanceData, MultiZoneData, ZoneAlarm};
use embedded_hal::blocking::i2c::{Write, WriteRead};
use std::collections::HashMap;

pub struct FakeOutputPin;

impl FakeOutputPin {
    pub fn toggle(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Simulate LED toggle
        Ok(())
    }
}

// Fake VLX implementation for simulator
pub struct FakeVlx {
    sensor_type: SensorType,
    address: u8,
    initialized: bool,
    ranging: bool,
    l1_distance: u16,
    l5_distances: Vec<u16>,
    l5_width: usize,
    l5_height: usize,
    alarms: Vec<ZoneAlarm>,
}

impl FakeVlx {
    pub fn new(sensor_type: SensorType, address: u8) -> Self {
        let (l5_distances, l5_width, l5_height) = match sensor_type {
            SensorType::L5 => {
                // Default 4x4 grid with varying distances
                let mut distances = Vec::new();
                for i in 0..16 {
                    distances.push(150 + (i * 10) as u16);
                }
                (distances, 4, 4)
            }
            SensorType::L1 => (vec![], 0, 0),
        };
        
        Self {
            sensor_type,
            address,
            initialized: false,
            ranging: false,
            l1_distance: 100,
            l5_distances,
            l5_width,
            l5_height,
            alarms: Vec::new(),
        }
    }
    
    pub fn set_l1_distance(&mut self, distance: u16) {
        self.l1_distance = distance;
    }
    
    pub fn set_l5_distances(&mut self, distances: Vec<u16>) {
        if distances.len() == self.l5_width * self.l5_height {
            self.l5_distances = distances;
        }
    }
}

#[derive(Debug)]
pub struct FakeVlxError;

impl std::fmt::Display for FakeVlxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Fake VLX error")
    }
}

impl std::error::Error for FakeVlxError {}

impl Sensor for FakeVlx {
    type Error = FakeVlxError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        println!("DEBUG: FakeVlx init - sensor_type: {:?}, address: 0x{:02X}", self.sensor_type, self.address);
        self.initialized = true;
        self.ranging = true; // Automatically start ranging after init
        Ok(())
    }
    
    fn get_distance(&mut self) -> Result<DistanceData, Self::Error> {
        if !self.initialized {
            return Err(FakeVlxError);
        }
        
        if !self.ranging {
            return Err(FakeVlxError);
        }
        
        match self.sensor_type {
            SensorType::L1 => Ok(DistanceData::Single(self.l1_distance)),
            SensorType::L5 => {
                let multizone = MultiZoneData::new(self.l5_width, self.l5_height, self.l5_distances.clone());
                Ok(DistanceData::MultiZone(multizone))
            }
        }
    }
    
    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(FakeVlxError);
        }
        
        let alarm = ZoneAlarm {
            zone: zone.unwrap_or((0, 0)),
            low,
            high,
        };
        self.alarms.push(alarm);
        Ok(())
    }
    
    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(FakeVlxError);
        }
        
        self.alarms.extend_from_slice(alarms);
        Ok(())
    }
    
    fn clear_alarm(&mut self) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(FakeVlxError);
        }
        
        self.alarms.clear();
        Ok(())
    }
}

// Simple I2C implementation for simulator (not used with FakeVlx)
pub struct FakeI2c;

impl FakeI2c {
    pub fn new() -> Self {
        Self
    }
}

#[derive(Debug)]
pub struct FakeI2cError;

impl std::fmt::Display for FakeI2cError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Fake I2C error")
    }
}

impl std::error::Error for FakeI2cError {}

impl Write for FakeI2c {
    type Error = FakeI2cError;
    
    fn write(&mut self, _addr: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl WriteRead for FakeI2c {
    type Error = FakeI2cError;
    
    fn write_read(&mut self, _addr: u8, _bytes: &[u8], _buffer: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}

// VLX wrapper that implements the same interface as the real VLX
pub struct FakeVlxWrapper {
    sensors: Vec<FakeVlx>,
}

impl FakeVlxWrapper {
    pub fn new(sensors: Vec<FakeVlx>) -> Self {
        Self { sensors }
    }
    
    pub fn init(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        for sensor in &mut self.sensors {
            sensor.init()?;
        }
        Ok(())
    }
    
    pub fn sensor_count(&self) -> usize {
        self.sensors.len()
    }
    
    pub fn get_distance(&mut self, index: usize) -> Result<DistanceData, Box<dyn std::error::Error>> {
        if index < self.sensors.len() {
            Ok(self.sensors[index].get_distance()?)
        } else {
            Err("Sensor index out of bounds".into())
        }
    }
}

// Board specific implementations
pub struct BoardPami {
    pub led_heartbeat: Option<FakeOutputPin>,
    pub vlx_sensors: Option<FakeVlxWrapper>,
}

impl BoardPami {
    pub fn new() -> Self {
        init_logging();
        
        let sensors = vec![
            FakeVlx::new(SensorType::L1, 0x30),
            FakeVlx::new(SensorType::L5, 0x31),
        ];
        
        Self {
            led_heartbeat: Some(FakeOutputPin),
            vlx_sensors: Some(FakeVlxWrapper::new(sensors)),
        }
    }
}

pub struct GalipeurBoard {
    pub fake_i2c: FakeI2c,
}

impl GalipeurBoard {
    pub fn new() -> Self {
        Self {
            fake_i2c: FakeI2c::new(),
        }
    }
    
    pub fn init(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("DEBUG: GalipeurBoard initialized successfully");
        Ok(())
    }
}

pub fn init_logging() {
    SimpleLogger::new().init().unwrap();
}