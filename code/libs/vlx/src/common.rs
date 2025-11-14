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

pub trait Sensor {
    fn init(&mut self) -> Result<(), VlxError>;
    fn get_distance(&mut self) -> Result<DistanceData, VlxError>;
    fn set_alarm(&mut self, low: u16, high: u16, zone: Option<(usize, usize)>) -> Result<(), VlxError>;
    fn set_multiple_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError>;
    fn clear_alarm(&mut self) -> Result<(), VlxError>;
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

#[derive(Debug)]
pub enum VlxError {
    InvalidSensor,
    InitError,
    ReadError,
    ConfigError,
    RangingError,
}
