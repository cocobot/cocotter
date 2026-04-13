//! Traits to abstract hardware-dependent implementations
use crate::BatteryLevel;


/// Read battery voltage
pub trait BatteryReader {
    fn read_vbatt(&mut self) -> BatteryLevel;
}

/// Generic OTA handler trait. Board code provides implementations.
pub trait OtaHandler: Send + 'static {
    /// Called on OTA START. Returns ready status: 0=ok, 1=busy, 2=no_space
    fn start(&mut self, size: u32, crc: u32) -> u8;
    /// Called on OTA DATA. Returns next expected offset.
    fn data(&mut self, offset: u32, data: &[u8]) -> u32;
    /// Called on OTA FINISH. Returns result status: 0=ok, 1=crc_fail, 2=size_mismatch
    fn finish(&mut self) -> u8;
    /// Called on OTA REBOOT.
    fn reboot(&mut self);
    /// Called on OTA ABORT.
    fn abort(&mut self);
}

/// Generic encoder, fetch a value of given type
pub trait Encoder<T> {
    type Error: core::fmt::Debug;

    /// Get encoded value
    fn get_value(&self) -> Result<T, Self::Error>;
}

