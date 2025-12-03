mod i2c;

#[cfg(target_os = "espidf")]
pub mod l1;
#[cfg(target_os = "espidf")]
pub mod l5;
#[cfg(target_os = "espidf")]
mod bindings;

pub use i2c::{VlxI2c, VlxI2cDriver};


/// Alarm configuration for a zone
#[derive(Debug, Clone)]
pub struct ZoneAlarm {
    pub zone: Option<(u8, u8)>,  // Zone coordinates if relevant, `None` for all of them
    pub low: u16,  // Low threshold, in mm
    pub high: u16,  // High threshold, in mm
}

/// VLX sensor interface
pub trait VlxSensor {
    /// Initialize the sensor, must be called once before using it
    fn init(&mut self) -> Result<(), VlxError>;
    /// Get a measure from the sensor and return it
    fn get_distance(&mut self) -> Result<DistanceData, VlxError>;
    /// Configure alarms, all at once; an empty `alarms` clears them
    fn set_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError>;
}


/// Grid of distance data
#[derive(Debug, Clone)]
pub struct DistanceData {
    width: u8,  // Grid width
    height: u8,  // Grid height
    distances: Vec<u16>,  // Distance data, row-major (line by line)
}

impl DistanceData {
    /// Create a new capture data
    ///
    /// `distances.len()` MUST be equal to `width * height`
    pub fn new(width: u8, height: u8, distances: Vec<u16>) -> Self {
        assert_eq!((width * height) as usize, distances.len());
        Self { width, height, distances }
    }

    /// Create data with a single value
    pub fn new_single(distance: u16) -> Self {
        Self { width: 1, height: 1, distances: vec![distance] }
    }

    /// Return the first valid distance (for simple uses)
    pub fn first_distance(&self) -> Option<u16> {
        self.distances.iter().find(|&&d| d != u16::MAX).copied()
    }

    /// Return all distances, in a linear slice
    pub fn all_distances(&self) -> &[u16] {
        &self.distances
    }

    /// Return zone count
    pub fn zone_count(&self) -> u8 {
        self.width * self.height
    }

    /// Return grid dimensions (width, height)
    pub fn dimensions(&self) -> (u8, u8) {
        (self.width, self.height)
    }

    /// Get a zone from its coordinates
    pub fn get(&self, x: u8, y: u8) -> Option<u16> {
        if x < self.width && y < self.height {
            Some(self.distances[(y * self.width + x) as usize])
        } else {
            None
        }
    }
}

#[derive(Debug)]
pub enum VlxError {
    /// Sensor not alive or not found during init
    SensorNotFound,
    /// Timeout error
    TimeoutError,
    /// Other communication error (CRC, corrupted frame, ...)
    CommunicationError,
    /// Invalid parameter
    InvalidParam,
    /// Attempt to configure too many alarms
    TooManyAlarms,
    /// Unknown error of the underlying API
    LowLevelError,
    /// Ranging data not ready
    DataNotReady,
    /// Invaling ranging measure
    InvalidMeasure,
}

