/// Hardware-independent items

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

/// VLX sensor interface
pub trait VlxSensor {
    fn init(&mut self) -> Result<(), VlxError>;
    fn get_distance(&mut self) -> Result<DistanceData, VlxError>;
    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError>;
    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError>;
    fn clear_alarm(&mut self) -> Result<(), VlxError>;
}


#[derive(Debug, Clone)]
/// Grid of distance data
pub struct DistanceData {
    width: usize,  // Grid width
    height: usize,  // Grid height
    distances: Vec<u16>,  // Distance data, row-major (line by line)
}

impl DistanceData {
    pub fn new(width: usize, height: usize, distances: Vec<u16>) -> Self {
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
    pub fn zone_count(&self) -> usize {
        self.width * self.height
    }

    /// Return grid dimensions (width, height)
    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Get a zone from its coordinates
    pub fn get(&self, x: usize, y: usize) -> Option<u16> {
        if x < self.width && y < self.height {
            Some(self.distances[y * self.width + x])
        } else {
            None
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

