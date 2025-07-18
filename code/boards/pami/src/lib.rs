use esp_idf_svc::hal::{gpio::{Gpio4, Output, PinDriver}, prelude::Peripherals, i2c::{I2cConfig, I2cDriver}, units::Hertz};
use vlx::{VLX, Config, SensorType};
use std::sync::Mutex;
use embedded_hal_bus::i2c::MutexDevice;

pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;

pub type LedHeartbeat = PinDriver<'static, Gpio4, Output>;
pub type VlxSensors = VLX<I2CType, 3>;

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

        let i2c_driver : Mutex<I2cDriver<'static>> = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap());
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));
        let mut i2c_bus : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_vlx : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
    
        // Configuration des capteurs VLX
        let mut enable_front_tof = PinDriver::output(peripherals.pins.gpio16).unwrap();
        let mut enable_back_tof = PinDriver::output(peripherals.pins.gpio3).unwrap();

        let vlx_configs: [Config<Box<dyn FnMut(bool)>, Box<dyn FnMut(bool)>>; 3] = [
            Config {
                i2c_address: 0x30,  // Adresse du premier capteur
                sensor_type: SensorType::L1,
                enable_fn: None,
                reset_fn: None,
            },
            Config {
                i2c_address: 0x31,  // Adresse du deuxième capteur
                sensor_type: SensorType::L5,
                enable_fn: Some(Box::new(move |enable| {
                    if enable {
                        enable_back_tof.set_high().ok();
                    } else {
                        enable_back_tof.set_low().ok();
                    }                    
                })),
                reset_fn: None,
            },
            Config {
                i2c_address: 0x32,  // Adresse du deuxième capteur
                sensor_type: SensorType::L5,
                enable_fn: Some(Box::new(move |enable| {
                    if enable {
                        enable_front_tof.set_high().ok();
                    } else {
                        enable_front_tof.set_low().ok();
                    }                    
                })),
                reset_fn: None,
            },
        ];

        let vlx_sensors = VLX::new(i2c_bus_vlx, vlx_configs);       

        Self {
            led_heartbeat: Some(led_heartbeat),
            vlx_sensors: Some(vlx_sensors),
        }
    }
}