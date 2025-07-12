use esp_idf_svc::hal::{gpio::{Gpio4, Output, PinDriver}, prelude::Peripherals};

pub type LedHeartbeat = PinDriver<'static, Gpio4, Output>;

pub struct BoardPami {
    pub led_heartbeat: Option<LedHeartbeat>,
}

impl BoardPami {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();

        let peripherals = Peripherals::take().unwrap();

        //intialiaze digital output pins
        let led_heartbeat= PinDriver::output(peripherals.pins.gpio4).unwrap();

        Self {
            led_heartbeat: Some(led_heartbeat),
        }
    }
}