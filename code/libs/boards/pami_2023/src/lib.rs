#![no_std]

use esp_hal::{gpio::{Io, Level, Output}, i2c::I2c, peripherals::I2C0, timer::timg::TimerGroup, Async};
use esp_hal::prelude::*;
use pwm_pca9685::{Address as Pca9685Address, Channel, Pca9685};

//declare hardcoded peripheral types
pub type I2C = I2c<'static, I2C0, Async>;

//external
pub const PWM_EXTENDED_CHANEL_SERVO     : [Channel; 4]  = [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANEL_VACCUM    : Channel       = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF        : Channel       = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF       : Channel       = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB         : [Channel; 3]  = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED         : Channel       = Channel::C11;
pub const PWM_EXTENDED_LED_RGB          : [Channel; 3]  = [Channel::C12, Channel::C13, Channel::C14];

pub struct Pami2023 {
    pub led_esp: Option<Output<'static>>,
    pub led_com: Option<Output<'static>>,
    pub pwm_extended: Option<Pca9685<I2C>>,
}

pub fn board_init() -> Pami2023 {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    //init embassy
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    //init peripherals
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let i2c0 = I2c::new_async(peripherals.I2C0, io.pins.gpio18, io.pins.gpio17, 400.kHz());

    //init i2c peripherals
    let pwm_extended = match Pca9685::new(i2c0, Pca9685Address::default()) {
        Ok(pwm) => Some(pwm),
        Err(_) => None,
    };

    Pami2023 { 
        led_esp: Some(Output::new(io.pins.gpio4, Level::High)),
        led_com: Some(Output::new(io.pins.gpio5, Level::High)),
        pwm_extended,
    }
}