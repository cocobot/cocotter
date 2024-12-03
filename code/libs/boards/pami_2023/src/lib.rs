#![no_std]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embedded_hal::digital::PinState;
use esp_hal::{gpio::{Input, Level, Output, Pull}, i2c::master::{Config, I2c}, pcnt::{self, Pcnt}, peripherals::PCNT, timer::timg::TimerGroup, Async};
use esp_hal::prelude::*;
use pwm_pca9685::{Address as Pca9685Address, Channel, Pca9685};
use static_cell::StaticCell;
use tca6408a::Tca6408a;
use core::cmp::min;

//declare hardcoded peripheral types
pub type I2C0PamiDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>;
pub type LeftWheelEncoder = pcnt::unit::Counter<'static, 0>;
pub type RightWheelEncoder = pcnt::unit::Counter<'static, 1>;

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
    pub pwm_extended: Option<Pca9685<I2C0PamiDevice>>,
    pub line_sensor: Option<Tca6408a<I2C0PamiDevice>>,
    pub buttons: Option<Tca6408a<I2C0PamiDevice>>,

    pub left_wheel_counter: Option<LeftWheelEncoder>,
    pub right_wheel_counter: Option<RightWheelEncoder>,

    pub generic_i2c: I2C0PamiDevice,
}

impl Pami2023 {
    fn init_counters(pcnt: PCNT, left_pin_a: Input, left_pin_b: Input, right_pin_a: Input, right_pin_b: Input) -> (pcnt::unit::Counter<'static, 0>, pcnt::unit::Counter<'static, 1>) {
        let pcnt = Pcnt::new(pcnt);

        let u0 = pcnt.unit0;
        u0.set_filter(Some(min(10u16 * 80, 1023u16))).unwrap();
        u0.clear();
    
        let ch0 = &u0.channel0;
        ch0.set_ctrl_signal(left_pin_a.peripheral_input());
        ch0.set_edge_signal(left_pin_b.peripheral_input());
        ch0.set_ctrl_mode(pcnt::channel::CtrlMode::Reverse, pcnt::channel::CtrlMode::Keep);
        ch0.set_input_mode(pcnt::channel::EdgeMode::Increment, pcnt::channel::EdgeMode::Decrement);
    
        let ch1 = &u0.channel1;
        ch1.set_ctrl_signal(left_pin_b.peripheral_input());
        ch1.set_edge_signal(left_pin_a.peripheral_input());
        ch1.set_ctrl_mode(pcnt::channel::CtrlMode::Reverse, pcnt::channel::CtrlMode::Keep);
        ch1.set_input_mode(pcnt::channel::EdgeMode::Decrement, pcnt::channel::EdgeMode::Increment);

        let u1 = pcnt.unit1;
        u1.set_filter(Some(min(10u16 * 80, 1023u16))).unwrap();
        u1.clear();
    
        let ch0 = &u1.channel0;
        ch0.set_ctrl_signal(right_pin_a.peripheral_input());
        ch0.set_edge_signal(right_pin_b.peripheral_input());
        ch0.set_ctrl_mode(pcnt::channel::CtrlMode::Reverse, pcnt::channel::CtrlMode::Keep);
        ch0.set_input_mode(pcnt::channel::EdgeMode::Increment, pcnt::channel::EdgeMode::Decrement);
    
        let ch1 = &u1.channel1;
        ch1.set_ctrl_signal(right_pin_b.peripheral_input());
        ch1.set_edge_signal(right_pin_a.peripheral_input());
        ch1.set_ctrl_mode(pcnt::channel::CtrlMode::Reverse, pcnt::channel::CtrlMode::Keep);
        ch1.set_input_mode(pcnt::channel::EdgeMode::Decrement, pcnt::channel::EdgeMode::Increment);

        u0.listen();
        u0.resume();
        u1.listen();
        u1.resume();
    
        let counter_left = u0.counter.clone();
        let counter_right = u1.counter.clone();
        
        (counter_left, counter_right)
    }

    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        //init embassy
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        //init peripherals
        let i2c0 = I2c::new(peripherals.I2C0, {
            let mut config = Config::default();
            config.frequency = 100.kHz();
            config
        }).with_scl(peripherals.GPIO17).with_sda(peripherals.GPIO18).into_async();
        static I2C0_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> = StaticCell::new();
        let i2c0 = I2C0_BUS.init(Mutex::new(i2c0));

        //init i2c peripherals
        let pwm_extended = match Pca9685::new(I2cDevice::new(i2c0), Pca9685Address::from(0b110_1001)) {
            Ok(pwm) => Some(pwm),
            Err(_) => None,
        };
        let line_sensor = Some(Tca6408a::new(I2cDevice::new(i2c0), tca6408a::Address::from_pin_state(PinState::Low)));
        let buttons = Some(Tca6408a::new(I2cDevice::new(i2c0), tca6408a::Address::from_pin_state(PinState::High)));

        //init motor peripherals
        let left_wheel_pin_a = Input::new(peripherals.GPIO39, Pull::None);
        let left_wheel_pin_b = Input::new(peripherals.GPIO40, Pull::None);
        let right_wheel_pin_a = Input::new(peripherals.GPIO41, Pull::None);
        let right_wheel_pin_b = Input::new(peripherals.GPIO42, Pull::None);
        let (left_wheel_counter, right_wheel_counter) = Self::init_counters(peripherals.PCNT, left_wheel_pin_a, left_wheel_pin_b, right_wheel_pin_a, right_wheel_pin_b);

        Self { 
            led_esp: Some(Output::new(peripherals.GPIO4, Level::High)),
            led_com: Some(Output::new(peripherals.GPIO5, Level::High)),
            pwm_extended,
            line_sensor,
            buttons,

            left_wheel_counter: Some(left_wheel_counter),
            right_wheel_counter: Some(right_wheel_counter),

            generic_i2c: I2cDevice::new(i2c0),
        }
    }
}