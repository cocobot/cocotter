#![no_std]
#![feature(async_fn_in_trait, impl_trait_projections)]
#![allow(incomplete_features)]
//! embassy-lora is a collection of async radio drivers that integrate with the lorawan-device
//! crate's async LoRaWAN MAC implementation.

pub(crate) mod fmt;
#[cfg(feature = "external-lora-phy")]
/// interface variants required by the external lora crate
pub mod iv;

#[cfg(feature = "stm32wl")]
#[deprecated(note = "use the external LoRa physical layer crate - https://crates.io/crates/lora-phy")]
pub mod stm32wl;
#[cfg(feature = "sx126x")]
#[deprecated(note = "use the external LoRa physical layer crate - https://crates.io/crates/lora-phy")]
pub mod sx126x;
#[cfg(feature = "sx127x")]
#[deprecated(note = "use the external LoRa physical layer crate - https://crates.io/crates/lora-phy")]
pub mod sx127x;

#[cfg(feature = "time")]
use embassy_time::{Duration, Instant, Timer};

/// A convenience timer to use with the LoRaWAN crate
#[cfg(feature = "time")]
pub struct LoraTimer {
    start: Instant,
}

#[cfg(feature = "time")]
impl LoraTimer {
    pub fn new() -> Self {
        Self { start: Instant::now() }
    }
}

#[cfg(feature = "time")]
impl lorawan_device::async_device::radio::Timer for LoraTimer {
    fn reset(&mut self) {
        self.start = Instant::now();
    }

    async fn at(&mut self, millis: u64) {
        Timer::at(self.start + Duration::from_millis(millis)).await
    }

    async fn delay_ms(&mut self, millis: u64) {
        Timer::after(Duration::from_millis(millis)).await
    }
}
