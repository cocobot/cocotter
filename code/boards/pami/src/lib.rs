use esp_idf_svc::{
    bt::{Ble, BtDriver},
    hal::{
        gpio::{AnyOutputPin, Output, PinDriver},
        i2c::{I2cConfig, I2cDriver},
        prelude::Peripherals,
        units::Hertz,
    },
    nvs::EspDefaultNvsPartition,
};
use tca6408::TCA6408;
use vlx::{VlxI2cDriver, VlxSensor, l1::VL53L1X, l5::VL53L5CX};
use std::sync::{Arc, Mutex};
use embedded_hal_bus::i2c::MutexDevice;
use pwm_pca9685::{Address, Channel, Pca9685};

pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;

pub type LedHeartbeat = PinDriver<'static, AnyOutputPin, Output>;
pub type PwmController = Arc<Mutex<Pca9685<I2CType>>>;
pub type LineSensor = TCA6408<I2CType>;
pub type Buttons = TCA6408<I2CType>;
pub type Bt = BtDriver<'static, Ble>;

pub struct VlxSensors {
    pub first: VL53L1X,
    pub back: VL53L5CX,
    pub front: VL53L5CX,
}

pub const PWM_EXTENDED_CHANNEL_SERVO: [Channel; 4] = [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANEL_VACCUM: Channel = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF: Channel = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF: Channel = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB: [Channel; 3] = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED: Channel = Channel::C11;
pub const PWM_EXTENDED_LED_RGB: [Channel; 3] = [Channel::C12, Channel::C13, Channel::C14];


pub struct BoardPami {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub ble: Option<Bt>,
    pub line_sensor: Option<LineSensor>,
    pub buttons: Option<Buttons>,
    vlx_sensors: Option<VlxSensors>,
    // Data needed to prepare VLX sensors
    pwm_controller: PwmController,
    vlx_back_enable: PinDriver<'static, AnyOutputPin, Output>,
    vlx_front_enable: PinDriver<'static, AnyOutputPin, Output>,
}

impl BoardPami {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let nvs = EspDefaultNvsPartition::take().unwrap();

        let peripherals = Peripherals::take().unwrap();

        // Initialize digital output pins
        let led_heartbeat = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio4)).unwrap();

        // Initialize the I2C bus
        let config = I2cConfig::new().baudrate(Hertz(100_000));
        let i2c_driver = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap());
        // Leak `i2c_driver` to get a static lifetime
        // The I2cDriver must leave until the end of the program, so it's fine
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));

        // Initialize PCA9685 PWM controller
        let pwm_controller = {
            let mut pwm_controller = Pca9685::new(MutexDevice::new(i2c_driver_static), Address::from(0x69)).unwrap();
            pwm_controller.set_prescale(100).unwrap();
            pwm_controller.enable().unwrap();

            pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[0], 4095).unwrap();
            pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[1], 4095).unwrap();
            pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[2], 4095).unwrap();

            pwm_controller.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
            pwm_controller.set_channel_full_on(PWM_EXTENDED_ENABLE_TOF, 1).unwrap();
            pwm_controller.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).unwrap();
            pwm_controller.set_channel_full_on(PWM_EXTENDED_RESET_TOF, 1).unwrap();

            Arc::new(Mutex::new(pwm_controller))
        };

        // VLX sensors
        let vlx_i2c_driver = VlxI2cDriver::register(MutexDevice::new(i2c_driver_static));
        let vlx_sensors = VlxSensors {
            first: VL53L1X::new(&vlx_i2c_driver, 0x30),
            back: VL53L5CX::new(&vlx_i2c_driver, 0x31),
            front: VL53L5CX::new(&vlx_i2c_driver, 0x32),
        };
        let vlx_back_enable = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio3)).unwrap();
        let vlx_front_enable = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio16)).unwrap();

        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        let line_sensor = TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0000);
        let buttons = TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0001);

        Self {
            led_heartbeat: Some(led_heartbeat),
            vlx_sensors: Some(vlx_sensors),
            line_sensor: Some(line_sensor),
            buttons: Some(buttons),
            ble,
            pwm_controller,
            vlx_back_enable,
            vlx_front_enable,
        }
    }

    /// Initialize VLX sensors and return them
    ///
    /// Run the disable/reset/enable procedure, then regular init.
    pub fn init_vlx_sensors(&mut self) -> Option<VlxSensors> {
        let mut vlx_sensors = self.vlx_sensors.take()?;

        // Disable back/front VLX
        self.vlx_back_enable.set_low().unwrap();
        self.vlx_front_enable.set_low().unwrap();

        // Reset first VLX
        {
            let mut pwm = self.pwm_controller.lock().unwrap();
            pwm.set_channel_on_off(PWM_EXTENDED_RESET_TOF, 0, 0).unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            pwm.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).unwrap();
            pwm.set_channel_full_on(PWM_EXTENDED_RESET_TOF, 1).unwrap();

            // Wait for reset to stabilize
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        // Enable back/front VLX
        self.vlx_back_enable.set_high().unwrap();
        self.vlx_front_enable.set_high().unwrap();

        // Enable first VLX
        {
            let mut pwm = self.pwm_controller.lock().unwrap();
            pwm.set_channel_full_off(PWM_EXTENDED_ENABLE_TOF).unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));
            pwm.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
            pwm.set_channel_full_on(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
        }

        // Small delay after enable
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Initialize sensors
        if vlx_sensors.first.init().is_err() {
            log::error!("Failed to initialize first VLX sensor");
        }
        if vlx_sensors.back.init().is_err() {
            log::error!("Failed to initialize back VLX sensor");
        }
        if vlx_sensors.front.init().is_err() {
            log::error!("Failed to initialize front VLX sensor");
        }

        Some(vlx_sensors)
    }
}

