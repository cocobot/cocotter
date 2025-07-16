use esp_idf_svc::hal::{gpio::{Gpio4, Output, PinDriver}, prelude::Peripherals, i2c::{I2cConfig, I2cDriver}, units::Hertz};
use shared_bus::{I2cProxy, BusManagerStd};
use vlx::{VLX, Config, SensorType};
use std::sync::Mutex;

pub type LedHeartbeat = PinDriver<'static, Gpio4, Output>;
pub type I2cBusManager = BusManagerStd<I2cDriver<'static>>;
pub type I2cBusProxy = I2cProxy<'static, Mutex<I2cDriver<'static>>>;

// Configuration pour 2 capteurs VLX (exemple)
pub type VlxSensors = VLX<I2cBusProxy, <I2cBusProxy as embedded_hal::blocking::i2c::WriteRead>::Error, 2, fn(bool), fn(bool)>;

pub struct BoardPami {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub vlx_sensors: Option<VlxSensors>,
}

impl BoardPami {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();

        let peripherals = Peripherals::take().unwrap();

        // Initialize digital output pins
        let led_heartbeat = PinDriver::output(peripherals.pins.gpio4).unwrap();

        // Initialize the I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_driver = I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap();
        let i2c_bus_manager = BusManagerStd::new(i2c_driver);
        let i2c_bus_manager_static = Box::leak(Box::new(i2c_bus_manager));
        let i2c_proxy = i2c_bus_manager_static.acquire_i2c();

        // Configuration des capteurs VLX
        let vlx_configs: [Config<fn(bool), fn(bool)>; 2] = [
            Config {
                i2c_address: 0x30,  // Adresse du premier capteur
                sensor_type: SensorType::L1,
                enable_fn: None,
                reset_fn: None,
            },
            Config {
                i2c_address: 0x31,  // Adresse du deuxième capteur
                sensor_type: SensorType::L5,
                enable_fn: None,
                reset_fn: None,
            },
        ];

        let vlx_sensors = VLX::new(i2c_proxy, vlx_configs);

        Self {
            led_heartbeat: Some(led_heartbeat),
            vlx_sensors: Some(vlx_sensors),
        }
    }
}