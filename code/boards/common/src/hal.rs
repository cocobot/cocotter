//! Traits to abstract hardware-dependent implementations
use crate::BatteryLevel;


/// Read battery voltage
pub trait BatteryReader {
    fn read_vbatt(&mut self) -> BatteryLevel;
}

/// Generic encoder, fetch a value of given type
pub trait Encoder<T> {
    type Error: core::fmt::Debug;

    /// Get encoded value
    fn get_value(&self) -> Result<T, Self::Error>;
}

