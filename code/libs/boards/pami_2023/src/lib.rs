#![no_std]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embedded_hal::digital::PinState;
use esp_hal::{analog::adc::{Adc, AdcConfig, AdcPin, Attenuation}, gpio::{Input, Io, Level, Output, Pull}, i2c::I2c, peripherals::I2C0, timer::timg::TimerGroup, Async};
use esp_hal::prelude::*;
use pwm_pca9685::{Address as Pca9685Address, Channel, Pca9685};
use static_cell::StaticCell;
use tca6408a::Tca6408a;

//declare hardcoded peripheral types
type I2C0PamiDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>;
//type AdcPamiDevice = Adc<'static, 


//external
pub const PWM_EXTENDED_CHANNEL_SERVO    : [Channel; 4]  = [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANNEL_VACCUM   : Channel       = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF        : Channel       = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF       : Channel       = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB         : [Channel; 3]  = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED         : Channel       = Channel::C11;
pub const PWM_EXTENDED_LED_RGB          : [Channel; 3]  = [Channel::C12, Channel::C13, Channel::C14];

/* 
pub struct EspAdc{
    //pin : AdcPin<Adc<'static, ADC1>>,
    pin : AdcPin<Adc<'static, ADC1>>,
    adc : Adc<'static, ADC1>,
}
*/

pub struct PamiAdc{
    pub vbatt       : AdcPin<esp_hal::gpio::GpioPin<1>, esp_hal::peripherals::ADC1>,
    pub i_mot_left  : AdcPin<esp_hal::gpio::GpioPin<9>, esp_hal::peripherals::ADC1>,
    pub i_mot_right : AdcPin<esp_hal::gpio::GpioPin<10>, esp_hal::peripherals::ADC1>,
    pub adc : Adc<'static, esp_hal::peripherals::ADC1>,
}

pub struct Pami2023 {
    pub led_esp: Option<Output<'static>>,
    pub led_com: Option<Output<'static>>,
    pub pwm_extended: Option<Pca9685<I2C0PamiDevice>>,
    pub line_sensor: Option<Tca6408a<I2C0PamiDevice>>,
    pub buttons: Option<Tca6408a<I2C0PamiDevice>>,
    //pub adc_vbatt : Option<AdcPin<esp_hal::gpio::GpioPin<1>, esp_hal::peripherals::ADC1>>,

    //pub adc_vbatt:          Option<Analog<'static, Adc<'static, AdcPin<Adc<'static, AdcConfig<ADC1>>, ADCI>>>>,
    /*pub adc_vbatt:          Option<AdcPin<Adc<'static, ADC1>, ADC1, AdcConfig<ADC1>>>,
    pub adc_i_mot_left:     Option<AdcPin<Adc<'static, ADC1>, ADC1, AdcConfig<ADC1>>>,
    pub adc_i_mot_right:    Option<EspAdc>,*/
    pub adc : Option<PamiAdc>,
    pub start_robot:        Option<Input<'static>>,
    pub emergency_stop:     Option<Input<'static>>,
}

pub struct I2cButtons{
    pub dir_up :bool,
    pub dir_down :bool,
    pub dir_left :bool,
    pub dir_right :bool,
    pub dir_push :bool,
    pub cfg_0 : bool,
    pub cfg_1 : bool,
    pub cfg_2 : bool,
}

impl From<u8> for I2cButtons{
    fn from(data_in: u8) -> Self{
        I2cButtons{
        dir_right :  data_in & 0b0000_0001 !=0,
        dir_down  :  data_in & 0b0000_0010 !=0,
        dir_up    :  data_in & 0b0000_0100 !=0,
        dir_push  :  data_in & 0b0000_1000 !=0,
        dir_left  :  data_in & 0b0001_0000 !=0,
        cfg_2     :  data_in & 0b0010_0000 !=0,
        cfg_1     :  data_in & 0b0100_0000 !=0,
        cfg_0     :  data_in & 0b1000_0000 !=0,
        }
    }
}





pub fn board_init() -> Pami2023 {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    //init embassy
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    //init peripherals
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let i2c0 = I2c::new_async(peripherals.I2C0, io.pins.gpio18, io.pins.gpio17, 400.kHz());
    static I2C0_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, Async>>> = StaticCell::new();
    let i2c0 = I2C0_BUS.init(Mutex::new(i2c0));

    //init i2c peripherals
    let pwm_extended = match Pca9685::new(I2cDevice::new(i2c0), Pca9685Address::from(0b110_1001)) {
        Ok(pwm) => Some(pwm),
        Err(_) => None,
    };
    let line_sensor = Some(Tca6408a::new(I2cDevice::new(i2c0), tca6408a::Address::from_pin_state(PinState::Low)));
    let buttons = Some(Tca6408a::new(I2cDevice::new(i2c0), tca6408a::Address::from_pin_state(PinState::High)));

    /*let mut adc1_config = AdcConfig::new();
    let mut imot_right_adcpin =  adc1_config.enable_pin(io.pins.gpio1.into_analog(), Attenuation::Attenuation11dB);   
    let mut adc1 = Adc::new(Adc::<ADC1>, adc1_config).unwrap();
*/
    let mut adc1_config = AdcConfig::new();

    let adc = Some(PamiAdc{
        vbatt:      adc1_config.enable_pin(io.pins.gpio1,  Attenuation::Attenuation6dB),    // 0-1750mV
        i_mot_left: adc1_config.enable_pin(io.pins.gpio9,  Attenuation::Attenuation11dB),   // 0-3100mV
        i_mot_right:adc1_config.enable_pin(io.pins.gpio10, Attenuation::Attenuation11dB),   // 0-3100mV
        adc : Adc::new(peripherals.ADC1, adc1_config),
    });

    Pami2023 { 
        led_esp: Some(Output::new(io.pins.gpio4, Level::High)),
        led_com: Some(Output::new(io.pins.gpio5, Level::High)),
        pwm_extended,
        line_sensor,
        buttons,
        adc,
        start_robot:    Some(Input::new(io.pins.gpio2,  Pull::None)),     
        emergency_stop: Some(Input::new(io.pins.gpio15, Pull::None)),    
    }
}