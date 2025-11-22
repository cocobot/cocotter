use simple_logger::SimpleLogger;
use vlx::{VlxSensor, DistanceData, VlxError, ZoneAlarm};
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub mod comm;

// Fake BLE type for drop-in replacement
pub struct FakeBle;
pub struct FakeOutputPin;

impl FakeOutputPin {
    pub fn toggle(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Simulate LED toggle
        Ok(())
    }
}

// Fake VLX implementation for simulator
pub struct FakeVlx {
    address: u8,
    initialized: bool,
    ranging: bool,
    width: usize,
    height: usize,
    distances: Vec<u16>,
    alarms: Vec<ZoneAlarm>,
}

impl FakeVlx {
    pub fn new_l1(address: u8) -> Self {
        Self {
            address,
            initialized: false,
            ranging: false,
            width: 1,
            height: 1,
            distances: vec![100],
            alarms: Vec::new(),
        }
    }

    pub fn new_l5(address: u8) -> Self {
        let distances = (0..16).map(|i| 150 + 10 * i).collect();
        Self {
            address,
            initialized: false,
            ranging: false,
            width: 4,
            height: 4,
            distances,
            alarms: Vec::new(),
        }
    }
}


impl VlxSensor for FakeVlx {
    fn init(&mut self) -> Result<(), VlxError> {
        println!("DEBUG: FakeVlx init - address: 0x{:02X}", self.address);
        self.initialized = true;
        self.ranging = true; // Automatically start ranging after init
        Ok(())
    }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {
        if !self.initialized {
            return Err(VlxError::InitError);
        }

        if !self.ranging {
            return Err(VlxError::RangingError);
        }

        Ok(DistanceData::new(self.width, self.height, self.distances.clone()))
    }

    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError> {
        if !self.initialized {
            return Err(VlxError::InitError);
        }

        let alarm = ZoneAlarm {
            zone: zone.unwrap_or((0, 0)),
            low,
            high,
        };
        self.alarms.push(alarm);
        Ok(())
    }

    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        if !self.initialized {
            return Err(VlxError::InitError);
        }

        self.alarms.extend_from_slice(alarms);
        Ok(())
    }

    fn clear_alarm(&mut self) -> Result<(), VlxError> {
        if !self.initialized {
            return Err(VlxError::InitError);
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


pub struct FakeVlxSensors {
    pub first: FakeVlx,
    pub back: FakeVlx,
    pub front: FakeVlx,
}

// Board specific implementations
pub struct BoardPami {
    pub led_heartbeat: Option<FakeOutputPin>,
    vlx_sensors: Option<FakeVlxSensors>,
}

impl BoardPami {
    pub fn new() -> Self {
        init_logging();

        let sensors = FakeVlxSensors {
            first: FakeVlx::new_l1(0x30),
            back: FakeVlx::new_l5(0x31),
            front: FakeVlx::new_l5(0x32),
        };
        Self {
            led_heartbeat: Some(FakeOutputPin),
            vlx_sensors: Some(sensors),
        }
    }

    pub fn init_vlx_sensors(&mut self) -> Option<FakeVlxSensors> {
        let mut sensors = self.vlx_sensors.take()?;
        sensors.first.init().unwrap();
        sensors.back.init().unwrap();
        sensors.front.init().unwrap();
        Some(sensors)
    }
}

pub struct BoardSabotter {
    pub led_heartbeat: Option<FakeOutputPin>,
    pub ble: Option<FakeBle>,
}

impl BoardSabotter {
    pub fn new() -> Self {
        init_logging();

        Self {
            led_heartbeat: Some(FakeOutputPin),
            ble: Some(FakeBle),
        }
    }
}

pub fn init_logging() {
    SimpleLogger::new().init().unwrap();
}
