use std::rc::Rc;
use std::sync::{Arc, Mutex};
use embedded_hal_bus::i2c::MutexDevice;
use esp_idf_svc::{
    bt::{Ble, BtDriver},
    hal::{
        gpio::{AnyOutputPin, Output, PinDriver, Gpio1},
        i2c::{I2cConfig, I2cDriver},
        prelude::Peripherals,
        units::Hertz,
        adc::{
            ADC1,
            attenuation,
            oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver},
        },
    },
    nvs::EspDefaultNvsPartition,
};
use pwm_pca9685::{Address, Channel, Pca9685};
use ssd1306::{
    I2CDisplayInterface, Ssd1306,
    mode::BufferedGraphicsMode,
    prelude::{
        DisplayConfig,
        DisplayRotation,
        DisplaySize128x64,
        I2CInterface as DisplayI2CInterface,
    },
};
use tca6408::TCA6408;
use vlx::{VlxI2cDriver, VlxSensor, l5::VL53L5CX};

pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;

pub type LedHeartbeat = PinDriver<'static, AnyOutputPin, Output>;
pub type PwmController = Arc<Mutex<Pca9685<I2CType>>>;
pub type LineSensor = TCA6408<I2CType>;
pub type Bt = BtDriver<'static, Ble>;
pub type Display = Ssd1306<DisplayI2CInterface<I2CType>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;


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
    pub buttons: Option<PamiButtons>,
    pub display: Option<Display>,
    pub vbatt: Option<Vbatt>,
    vlx_sensor: Option<(VL53L5CX, PinDriver<'static, AnyOutputPin, Output>)>,
    // Data needed to prepare VLX sensors
    pwm_controller: PwmController,
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
        let vlx_sensor = VL53L5CX::new(&vlx_i2c_driver, 0x31);
        let vlx_enable = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio3)).unwrap();

        // ADC
        let adc = Rc::new(AdcDriver::new(peripherals.adc1).unwrap());
        let adc_vbatt = AdcChannelDriver::new(
            adc.clone(),
            peripherals.pins.gpio1,
            &AdcChannelConfig { attenuation: attenuation::DB_11, ..Default::default() },
        ).unwrap();
        let vbatt = Vbatt(adc_vbatt);

        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        let line_sensor = TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0000);
        let buttons = PamiButtons(TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0001));

        // Screen
        let display = {
            let interface = I2CDisplayInterface::new(MutexDevice::new(i2c_driver_static));
            let mut display  = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();
            display.init().unwrap();
            display
        };

        Self {
            led_heartbeat: Some(led_heartbeat),
            ble,
            line_sensor: Some(line_sensor),
            buttons: Some(buttons),
            display: Some(display),
            vbatt: Some(vbatt),
            vlx_sensor: Some((vlx_sensor, vlx_enable)),
            pwm_controller,
        }
    }

    /// Initialize VLX back sensor and return it
    ///
    /// Run the disable/reset/enable procedure, then regular init.
    pub fn init_vlx_sensor(&mut self) -> Option<VL53L5CX> {
        let (mut vlx_sensor, mut vlx_enable) = self.vlx_sensor.take()?;

        // Disable then enable VLX
        vlx_enable.set_low().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        vlx_enable.set_high().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Initialize sensor
        if vlx_sensor.init().is_err() {
            log::error!("Failed to initialize back VLX sensor");
        }

        Some(vlx_sensor)
    }
}


pub struct Vbatt(AdcChannelDriver<'static, Gpio1, Rc<AdcDriver<'static, ADC1>>>);

impl Vbatt {
    /// Read battery voltage, return value in mV and percentage
    pub fn read(&mut self) -> (u16, u8) {
        let raw = self.0.read().unwrap() as f32;
        let mv = Self::raw_to_mv(raw);
        let pct = Self::mv_to_percent(mv);
        (mv as u16, pct)
    }

    const fn raw_to_mv(raw: f32) -> f32 {
        const VBATT_RL_KOHMS: f32 = 91.0;
        const VBATT_RH_KOHMS: f32 = 91.0;
        const ADC_INPUT_IMP_KOHMS: f32 = 500.0;
        const RL_KOHMS: f32 = VBATT_RL_KOHMS * ADC_INPUT_IMP_KOHMS / (VBATT_RL_KOHMS + ADC_INPUT_IMP_KOHMS);
        raw * (1.0 + VBATT_RH_KOHMS/RL_KOHMS)
    }

    fn mv_to_percent(measure_mv: f32) -> u8 {
        // LiFePO4 voltage to percentage lookup table (voltage in mV, percentage points)
        const LIFEPO4_CURVE: [(f32, f32); 11] = [
            (3400.0, 100.0),
            (3350.0, 90.0),
            (3320.0, 80.0),
            (3300.0, 70.0),
            (3270.0, 60.0),
            (3260.0, 50.0),
            (3250.0, 40.0),
            (3220.0, 35.0),
            (3200.0, 20.0),
            (3000.0, 10.0),
            (2500.0, 0.0),
        ];

        match LIFEPO4_CURVE.iter().position(|(mv, _)| measure_mv >= *mv) {
            None => 0,
            Some(0) => 100,
            Some(i) => {
                let (mv0, pct0) = LIFEPO4_CURVE[i];
                let (mv1, pct1) = LIFEPO4_CURVE[i - 1];
                let pct = pct0 + (pct1 - pct0) * (measure_mv - mv0) / (mv1 - mv0);
                pct as u8
            }
        }
    }
}


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DpadState { None, Middle, Up, Down, Left, Right }

impl DpadState {
    pub fn as_char(&self) -> char {
        match self {
            Self::None => '-',
            Self::Middle => 'X',
            Self::Up => '^',
            Self::Down => 'v',
            Self::Left => '<',
            Self::Right => '>',
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PamiButtonsState(pub u8);

impl PamiButtonsState {
    pub fn dpad(&self) -> DpadState {
        // Assume 1 action on the dpad
        match self.0 & 0b11111 {
            0b00001 => DpadState::Right,
            0b00010 => DpadState::Down,
            0b00100 => DpadState::Up,
            0b01000 => DpadState::Middle,
            0b10000 => DpadState::Left,
            _ => DpadState::None,
        }
    }

    pub fn switches(&self) -> [bool; 3] {
        [
            self.0 & 0b001_00000 != 0,
            self.0 & 0b010_00000 != 0,
            self.0 & 0b100_00000 != 0,
        ]
    }
}

pub struct PamiButtons(TCA6408<I2CType>);

impl PamiButtons {
    /// Read new button state
    pub fn read_state(&mut self) -> PamiButtonsState {
        //XXX Errors are ignored
        PamiButtonsState(self.0.read_inputs().unwrap_or(0))
    }
}

