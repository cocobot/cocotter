#![no_std]


use core::cmp::min;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_hal::digital::PinState;
use esp_hal::spi::{BitOrder, Mode};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, Level, Output, Pull, GpioPin},
    spi::master::{Config as SpiConfig, Spi},
    i2c::master::{Config, I2c},
    pcnt::{self, Pcnt},
    peripherals::PCNT,
    timer::timg::TimerGroup,
    Async,
    mcpwm::{McPwm, operator::{PwmPinConfig, PwmPin}, PeripheralClockConfig, timer::PwmWorkingMode},
    time::RateExtU32,
};
pub use pwm_pca9685::{Address as Pca9685Address, Channel, Pca9685};
use static_cell::StaticCell;
use tca6408a::Tca6408a;
use lsm6dso32x::{Lsm6dso32x, Lsm6dso32xConfiguration, AccelerometerConfiguration, GyroscopeConfiguration, ODR, AccelerometerFullScale, GyroFullScale, };

//declare hardcoded peripheral types
pub type I2C0PamiDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>;
pub type LeftWheelEncoder = pcnt::unit::Counter<'static, 0>;
pub type RightWheelEncoder = pcnt::unit::Counter<'static, 1>;
pub type LeftMotorPwms = (PwmPin<'static, esp_hal::peripherals::MCPWM0, 0, true>, PwmPin<'static, esp_hal::peripherals::MCPWM0, 0, false>);
pub type RightMotorPwms = (PwmPin<'static, esp_hal::peripherals::MCPWM0, 1, true>, PwmPin<'static, esp_hal::peripherals::MCPWM0, 1, false>); 

//external
pub const PWM_EXTENDED_CHANEL_SERVO: [Channel; 4] =
    [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANEL_VACCUM: Channel = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF: Channel = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF: Channel = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB: [Channel; 3] = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED: Channel = Channel::C11;
pub const PWM_EXTENDED_LED_RGB: [Channel; 3] = [Channel::C12, Channel::C13, Channel::C14];

pub enum PamiAdcChannel {
    VBat,
    IMotLeft,
    IMotRight,
}

pub struct PamiAdc {
    vbatt: AdcPin<esp_hal::gpio::GpioPin<1>, esp_hal::peripherals::ADC1>,
    i_mot_left: AdcPin<esp_hal::gpio::GpioPin<9>, esp_hal::peripherals::ADC1>,
    i_mot_right: AdcPin<esp_hal::gpio::GpioPin<10>, esp_hal::peripherals::ADC1>,

    pub adc: Adc<'static, esp_hal::peripherals::ADC1>,
}

impl PamiAdc {    
    pub async fn read(&mut self, channel: PamiAdcChannel) -> u16 {
        loop {
            //ESP-HAL ADC is not compatible with embassy async yet.
            //TODO: rework this to have an IRQ to signal the executor
            //instead of periodic polling
            const ADC_RES : u16 = 13;// 12 bits data are stored in 13 bits to be compatible with continuous read that returns 13bits resolution data

            let res = match channel {
                PamiAdcChannel::VBat => {
                    let raw_opt = self.adc.read_oneshot(&mut self.vbatt);
                    match raw_opt{
                        Ok(raw) => {
                            const ADC_MAX_V :f32 = 3100.0;
                            let voltage = ((raw as f32)* ADC_MAX_V)/(((1<<ADC_RES) - 1) as f32); 
        
                            const VBATT_RL_KOHMS : f32= 91.0;
                            const VBATT_RH_KOHMS : f32= 91.0;
                            const ADC_INPUT_IMP_KOHMS : f32= 500.0;
                            let rl_kohms = VBATT_RL_KOHMS*ADC_INPUT_IMP_KOHMS/(VBATT_RL_KOHMS+ADC_INPUT_IMP_KOHMS);
                            let battery_voltage = voltage * (1.0 + VBATT_RH_KOHMS/rl_kohms);
                            Ok(battery_voltage as u16)
                        }
                        _=> raw_opt
                    }
                    
                }
                PamiAdcChannel::IMotLeft => self.adc.read_oneshot(&mut self.i_mot_left),
                PamiAdcChannel::IMotRight => self.adc.read_oneshot(&mut self.i_mot_right),
            };

            match res {
                Ok(result) => return result,
                _ => {
                    Timer::after(Duration::from_millis(2)).await;
                }
            }
        }
    }
}


pub struct Pami2023 {
    pub led_esp: Option<Output<'static>>,
    pub led_com: Option<Output<'static>>,

    pub start_robot: Option<Input<'static>>,
    pub emergency_stop: Option<Input<'static>>,

    pub left_bumper_switch: Option<Input<'static>>,
    pub right_bumper_switch: Option<Input<'static>>,

    pub pwm_extended: Option<Pca9685<I2C0PamiDevice>>,
    pub line_sensor: Option<Tca6408a<I2C0PamiDevice>>,
    pub buttons: Option<Tca6408a<I2C0PamiDevice>>,

    pub left_wheel_counter: Option<LeftWheelEncoder>,
    pub right_wheel_counter: Option<RightWheelEncoder>,

    //pub left_wheel_pwm : Option<>,

    pub adc: Option<PamiAdc>,

    pub accelerometer : Option<Lsm6dso32x>,
    pub left_motor_pwm : Option<LeftMotorPwms>,
    pub right_motor_pwm : Option<RightMotorPwms>,
}


impl Pami2023 {
    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        //init embassy
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        //init i2c peripherals
        let i2c0 = I2c::new(peripherals.I2C0, {
            let mut config = Config::default();
            config.frequency = 100.kHz();
            config
        }).unwrap()
        .with_scl(peripherals.GPIO17)
        .with_sda(peripherals.GPIO18);

        let _enable_front_tof = Output::new(peripherals.GPIO3, Level::Low);
        let _enable_back_tof = Output::new(peripherals.GPIO16, Level::High);
        
        
        let i2c0 = i2c0.into_async();
    
        let (pwm_extended, line_sensor, buttons) = Self::init_i2c_devices(i2c0);

        //init motor peripherals
        let left_wheel_pin_a = Input::new(peripherals.GPIO39, Pull::None);
        let left_wheel_pin_b = Input::new(peripherals.GPIO40, Pull::None);
        let right_wheel_pin_a = Input::new(peripherals.GPIO41, Pull::None);
        let right_wheel_pin_b = Input::new(peripherals.GPIO42, Pull::None);
        let (left_wheel_counter, right_wheel_counter) = Self::init_counters(
            peripherals.PCNT,
            left_wheel_pin_a,
            left_wheel_pin_b,
            right_wheel_pin_a,
            right_wheel_pin_b,
        );

        //init adc
        let mut adc1_config = AdcConfig::new();
      
        let adc = PamiAdc {
            vbatt: adc1_config.enable_pin_with_cal(peripherals.GPIO1, Attenuation::_11dB), // 0-3100mV
            i_mot_left: adc1_config.enable_pin_with_cal(peripherals.GPIO9, Attenuation::_11dB), // 0-3100mV
            i_mot_right: adc1_config.enable_pin_with_cal(peripherals.GPIO10, Attenuation::_11dB), // 0-3100mV

            adc: Adc::new(peripherals.ADC1, adc1_config),
        };

        let spi = Spi::new( peripherals.SPI2, 
                                     {  let mut config = SpiConfig::default();
                                        config.frequency = 100.kHz();
                                        config.mode = Mode::_3;
                                        config.read_bit_order = BitOrder::MsbFirst;
                                        config.write_bit_order = BitOrder::MsbFirst;
                                        config
                                    })
                                    .unwrap()
                                    .with_cs(peripherals.GPIO37)
                                    .with_sck(peripherals.GPIO48)
                                    .with_miso(peripherals.GPIO35)
                                    .with_mosi(peripherals.GPIO36);

        let acc_config = Lsm6dso32xConfiguration  {
            accelerometer : { let config = AccelerometerConfiguration {
                odr: ODR::Odr416Hz,
                full_scale : AccelerometerFullScale::Fs2g,
            };
            config},
            gyroscope : {let config =  GyroscopeConfiguration {
                odr: ODR::Odr416Hz,
                full_scale : GyroFullScale::Fs1000dps,
            };
            config},
        };

        let (left_motor_pwm, right_motor_pwm) = Self::init_motor_pwm(peripherals.MCPWM0 ,
                       peripherals.GPIO14, peripherals.GPIO21,
                       peripherals.GPIO12, peripherals.GPIO13);

        Self {
            led_esp: Some(Output::new(peripherals.GPIO4, Level::High)),
            led_com: Some(Output::new(peripherals.GPIO5, Level::High)),

            start_robot: Some(Input::new(peripherals.GPIO2, Pull::None)),
            emergency_stop: Some(Input::new(peripherals.GPIO15, Pull::None)),

            left_bumper_switch: Some(Input::new(peripherals.GPIO47, Pull::None)),
            right_bumper_switch: Some(Input::new(peripherals.GPIO6, Pull::None)),

            pwm_extended: Some(pwm_extended),
            line_sensor: Some(line_sensor),
            buttons: Some(buttons),

            left_wheel_counter: Some(left_wheel_counter),
            right_wheel_counter: Some(right_wheel_counter),

            adc: Some(adc),
            accelerometer: Some(Lsm6dso32x::new(acc_config, spi)),
            left_motor_pwm : Some(left_motor_pwm),
            right_motor_pwm : Some(right_motor_pwm)
        }
    }

    fn init_motor_pwm(mcpwm_dev : esp_hal::peripherals::MCPWM0,
                    dir_left: GpioPin<14>, pwm_left: GpioPin<21>,
                    dir_right: GpioPin<12>, pwm_right: GpioPin<13>) ->
                    (LeftMotorPwms, RightMotorPwms )
                    {
        // initialize peripheral
        let clock_cfg = PeripheralClockConfig::with_frequency(32u32.MHz()).unwrap();
        let mut mcpwm = McPwm::new(mcpwm_dev, clock_cfg);

        // connect operator0 to timer0
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        // connect operator0 to pin
        let left_motor = mcpwm
            .operator0
            .with_pins(dir_left, PwmPinConfig::UP_ACTIVE_HIGH,
                pwm_left, PwmPinConfig::UP_ACTIVE_HIGH);

        let right_motor = mcpwm
            .operator1
            .with_pins(dir_right, PwmPinConfig::UP_ACTIVE_HIGH,
                pwm_right, PwmPinConfig::UP_ACTIVE_HIGH);
        // start timer with timestamp values in the range of 0..=99 and a frequency
        // of 20 kHz
        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(999, PwmWorkingMode::Increase, 20.kHz())
            .unwrap();
        mcpwm.timer0.start(timer_clock_cfg);

        /*left_motor.0.set_timestamp(10);
        left_motor.1.set_timestamp(0);*/
        (left_motor, right_motor)

    }


    fn init_i2c_devices(
        i2c: I2c<'static, Async>,
    ) -> (
        Pca9685<I2C0PamiDevice>,
        Tca6408a<I2C0PamiDevice>,
        Tca6408a<I2C0PamiDevice>,
    ) {
        static I2C0_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
            StaticCell::new();
        let i2c0: &'static mut Mutex<CriticalSectionRawMutex, I2c<'_, Async>> = I2C0_BUS.init(Mutex::new(i2c));

        //init i2c peripherals
        let pwm_extended = Pca9685::new(I2cDevice::new(i2c0), Pca9685Address::from(0b110_1001))
            .ok()
            .unwrap();

        let line_sensor = Tca6408a::new(
            I2cDevice::new(i2c0),
            tca6408a::Address::from_pin_state(PinState::Low),
        );

        let buttons = Tca6408a::new(
            I2cDevice::new(i2c0),
            tca6408a::Address::from_pin_state(PinState::High),
        );

        (pwm_extended, line_sensor, buttons)
    }

    fn init_counters(
        pcnt: PCNT,
        left_pin_a: Input,
        left_pin_b: Input,
        right_pin_a: Input,
        right_pin_b: Input,
    ) -> (
        pcnt::unit::Counter<'static, 0>,
        pcnt::unit::Counter<'static, 1>,
    ) {
        let pcnt = Pcnt::new(pcnt);

        let u0 = pcnt.unit0;
        u0.set_filter(Some(min(10u16 * 80, 1023u16))).unwrap();
        u0.clear();

        let ch0 = &u0.channel0;
        ch0.set_ctrl_signal(left_pin_a.peripheral_input());
        ch0.set_edge_signal(left_pin_b.peripheral_input());
        ch0.set_ctrl_mode(
            pcnt::channel::CtrlMode::Reverse,
            pcnt::channel::CtrlMode::Keep,
        );
        ch0.set_input_mode(
            pcnt::channel::EdgeMode::Decrement,
            pcnt::channel::EdgeMode::Increment,
        );

        let ch1 = &u0.channel1;
        ch1.set_ctrl_signal(left_pin_b.peripheral_input());
        ch1.set_edge_signal(left_pin_a.peripheral_input());
        ch1.set_ctrl_mode(
            pcnt::channel::CtrlMode::Reverse,
            pcnt::channel::CtrlMode::Keep,
        );
        ch1.set_input_mode(
            pcnt::channel::EdgeMode::Increment,
            pcnt::channel::EdgeMode::Decrement,
        );

        let u1 = pcnt.unit1;
        u1.set_filter(Some(min(10u16 * 80, 1023u16))).unwrap();
        u1.clear();

        let ch0 = &u1.channel0;
        ch0.set_ctrl_signal(right_pin_a.peripheral_input());
        ch0.set_edge_signal(right_pin_b.peripheral_input());
        ch0.set_ctrl_mode(
            pcnt::channel::CtrlMode::Reverse,
            pcnt::channel::CtrlMode::Keep,
        );
        ch0.set_input_mode(
            pcnt::channel::EdgeMode::Decrement,
            pcnt::channel::EdgeMode::Increment,
        );

        let ch1 = &u1.channel1;
        ch1.set_ctrl_signal(right_pin_b.peripheral_input());
        ch1.set_edge_signal(right_pin_a.peripheral_input());
        ch1.set_ctrl_mode(
            pcnt::channel::CtrlMode::Reverse,
            pcnt::channel::CtrlMode::Keep,
        );
        ch1.set_input_mode(
            pcnt::channel::EdgeMode::Increment,
            pcnt::channel::EdgeMode::Decrement,
        );

        u0.listen();
        u0.resume();
        u1.listen();
        u1.resume();

        let counter_left = u0.counter.clone();
        let counter_right = u1.counter.clone();

        (counter_left, counter_right)
    }
}
