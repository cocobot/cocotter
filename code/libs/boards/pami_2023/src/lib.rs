pub mod adc;
pub mod encoder;

use std::rc::Rc;
use std::sync::Mutex;

use adc::PamiAdc;
use embedded_hal_bus::i2c::MutexDevice;
use encoder::Encoder;
use esp_idf_svc::hal::gpio::{Output, PinDriver};
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::{adc::{attenuation, oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver}}, gpio, i2c::{I2cConfig, I2cDriver}, prelude::Peripherals};
use pwm_pca9685::Channel;
pub use pwm_pca9685::{Address as Pca9685Address, Pca9685};
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use tca6408a::Tca6408a;
pub use vl53l5cx::Vl53l5cx;

pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;
pub type EmergencyStop = PinDriver<'static, gpio::Gpio15, gpio::Input>;
pub type Starter = PinDriver<'static, gpio::Gpio2, gpio::Input>;
pub type MotorPwmType = LedcDriver<'static>;
pub type DisplayType = Ssd1306<ssd1306::prelude::I2CInterface<MutexDevice<'static, I2cDriver<'static>>>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>>;
pub type VlxType = Vl53l5cx<I2CType>;
pub type TCA6408AType = Tca6408a<I2CType>;

pub const PWM_EXTENDED_CHANEL_SERVO: [Channel; 4] =
    [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANEL_VACCUM: Channel = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF: Channel = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF: Channel = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB: [Channel; 3] = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED: Channel = Channel::C11;
pub const PWM_EXTENDED_LED_RGB: [Channel; 3] = [Channel::C12, Channel::C13, Channel::C14];


pub struct Pami2023 {
    pub led_heartbeat: Option<PinDriver<'static, gpio::Gpio4, Output>>,
    pub pwm_extended: Option<Pca9685<I2CType>>,
    pub adc: Option<PamiAdc>,
    pub encoder_left: Option<Encoder<'static>>,
    pub encoder_right: Option<Encoder<'static>>,
    pub left_pwm: Option<(MotorPwmType, MotorPwmType)>,
    pub right_pwm: Option<(MotorPwmType, MotorPwmType)>,
    pub emergency_stop: Option<EmergencyStop>,
    pub starter: Option<Starter>,
    pub display: Option<DisplayType>,
    pub tof: Option<VlxType>,
    pub line_sensor: Option<TCA6408AType>,
    pub buttons: Option<TCA6408AType>,
    pub conf_3_button: Option<PinDriver<'static, gpio::Gpio7, gpio::Input>>,
}

impl Pami2023{
    pub fn new() -> Self {
        let peripherals = Peripherals::take().unwrap();
        
        // Initialize the ADC
        let mut db_11 = AdcChannelConfig::new();
        db_11.attenuation = attenuation::DB_11;
        let mut db_0 = AdcChannelConfig::new();
        db_0.attenuation = attenuation::NONE;
        let adc = Rc::new(AdcDriver::new(peripherals.adc1).unwrap());
        let vbatt_pin = AdcChannelDriver::new(adc.clone(), peripherals.pins.gpio1, &db_11).unwrap();
        let imot_left_pin = AdcChannelDriver::new(adc.clone(), peripherals.pins.gpio9, &db_0).unwrap();
        let imot_right_pin = AdcChannelDriver::new(adc.clone(), peripherals.pins.gpio10, &db_0).unwrap();
        let adc = PamiAdc::new(vbatt_pin, imot_left_pin, imot_right_pin);

        // Initialize the encoders
        let left_encoder = Encoder::new(peripherals.pcnt0,  peripherals.pins.gpio39,  peripherals.pins.gpio40).unwrap();
        let right_encoder = Encoder::new(peripherals.pcnt1, peripherals.pins.gpio41, peripherals.pins.gpio42).unwrap();

        // Initialize the PWM
        let timer_pwm = LedcTimerDriver::new(
                peripherals.ledc.timer0,
                &TimerConfig::new().frequency(25.kHz().into()).resolution(esp_idf_svc::hal::ledc::Resolution::Bits10),
            ).unwrap();
        let left_pwm = (LedcDriver::new(
            peripherals.ledc.channel0,
            &timer_pwm,
            peripherals.pins.gpio21,
        ).unwrap(), LedcDriver::new(
            peripherals.ledc.channel1,
            &timer_pwm,
            peripherals.pins.gpio14,
        ).unwrap());
        let right_pwm = (LedcDriver::new(
            peripherals.ledc.channel2,
            &timer_pwm,
            peripherals.pins.gpio12,
        ).unwrap(), LedcDriver::new(
            peripherals.ledc.channel3,
            &timer_pwm,
            peripherals.pins.gpio13,
        ).unwrap());
        let emergency_stop = PinDriver::input(peripherals.pins.gpio15).unwrap();

        let starter = PinDriver::input(peripherals.pins.gpio2).unwrap();
        let conf_3_button = PinDriver::input(peripherals.pins.gpio7).unwrap();

        // Initialize the I2C bus
        let config = I2cConfig::new().baudrate(400.kHz().into());
        let i2c_driver : Mutex<I2cDriver<'static>> = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap());
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));
        let i2c_bus : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_screen : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_vlx : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_line_sensor : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_buttons : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);

        // Initialize the PCA9685
        let pwm_extended = Pca9685::new(i2c_bus, Pca9685Address::from(0b110_1001))
            .ok()
            .unwrap();

        // Initialize the TCA3108A for the screen
        let mut line_sensor = Tca6408a::new(
            i2c_bus_line_sensor,
            tca6408a::Address::from_pin_state(false),
        );
        let buttons = Tca6408a::new(
            i2c_bus_buttons,
            tca6408a::Address::from_pin_state(true),
        );
        line_sensor.configure_output(0b00000000).ok();

        // Initialize the Vl53l5cx
        let mut enable_front_tof = PinDriver::output(peripherals.pins.gpio3).unwrap();
        let mut enable_back_tof = PinDriver::output(peripherals.pins.gpio16).unwrap();
        enable_front_tof.set_high().ok();
        enable_back_tof.set_low().ok();
        let tof = Vl53l5cx::new(i2c_bus_vlx, None);
        Box::leak(Box::new(enable_back_tof));
        Box::leak(Box::new(enable_front_tof));

        // Initialize the GPIO for the heartbeat LED
        let led_heartbeat = PinDriver::output(peripherals.pins.gpio4).unwrap();

        //Initialize the screen
        let interface = I2CDisplayInterface::new(i2c_bus_screen);
        let mut display  = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        Self {
            led_heartbeat: Some(led_heartbeat),
            adc: Some(adc),
            pwm_extended: Some(pwm_extended),
            encoder_left: Some(left_encoder),
            encoder_right: Some(right_encoder),
            left_pwm: Some(left_pwm),
            right_pwm: Some(right_pwm),
            emergency_stop: Some(emergency_stop),
            starter: Some(starter),
            display: Some(display),
            tof: Some(tof),
            line_sensor: Some(line_sensor),
            buttons: Some(buttons),
            conf_3_button: Some(conf_3_button),
        }
    }
}