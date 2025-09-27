use esp_idf_svc::{bt::{Ble, BtDriver}, hal::{gpio::{Gpio4, Output, PinDriver}, i2c::{I2cConfig, I2cDriver}, ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver}, prelude::Peripherals, units::Hertz}, nvs::EspDefaultNvsPartition};
use vlx::{VLX, Config, SensorType};
use std::sync::{Arc, Mutex};
use embedded_hal_bus::i2c::MutexDevice;
use pwm_pca9685::{Address, Channel, Pca9685};

pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;

pub type LedHeartbeat = PinDriver<'static, Gpio4, Output>;
pub type VlxSensors = VLX<I2CType, 3>;
pub type PwmController = Arc<Mutex<Pca9685<I2CType>>>;
pub type Bt = BtDriver<'static, Ble>;

pub const PWM_EXTENDED_CHANNEL_SERVO: [Channel; 4] =
    [Channel::C0, Channel::C1, Channel::C2, Channel::C3];
pub const PWM_EXTENDED_CHANEL_VACCUM: Channel = Channel::C5;
pub const PWM_EXTENDED_RESET_TOF: Channel = Channel::C6;
pub const PWM_EXTENDED_ENABLE_TOF: Channel = Channel::C7;
pub const PWM_EXTENDED_VBAT_RGB: [Channel; 3] = [Channel::C10, Channel::C8, Channel::C9];
pub const PWM_EXTENDED_LINE_LED: Channel = Channel::C11;
pub const PWM_EXTENDED_LED_RGB: [Channel; 3] = [Channel::C12, Channel::C13, Channel::C14];

pub type MotorPwmType = LedcDriver<'static>;

pub struct BoardPami {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub vlx_sensors: Option<VlxSensors>,
    pub pwm_controller: Option<PwmController>,
    pub left_pwm: Option<(MotorPwmType, MotorPwmType)>,
    pub right_pwm: Option<(MotorPwmType, MotorPwmType)>,
    pub ble: Option<Bt>,
}

impl BoardPami {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let nvs = EspDefaultNvsPartition::take().unwrap();
        
        let peripherals = Peripherals::take().unwrap();

        // Initialize digital output pins
        let led_heartbeat = PinDriver::output(peripherals.pins.gpio4).unwrap();

        // Initialize the I2C bus
        let config = I2cConfig::new().baudrate(Hertz(100_000));

        let i2c_driver : Mutex<I2cDriver<'static>> = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap());
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));
        let i2c_bus_vlx : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        let i2c_bus_pwm : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_driver_static);
        
        // Initialize PCA9685 PWM controller
        let mut pwm_controller = Pca9685::new(i2c_bus_pwm, Address::from(0x69)).unwrap();
        pwm_controller.set_prescale(100).unwrap();
        pwm_controller.enable().unwrap();

        pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[0], 4095).unwrap();
        pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[1], 4095).unwrap();
        pwm_controller.set_channel_off(PWM_EXTENDED_VBAT_RGB[2], 4095).unwrap();

        pwm_controller.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
        pwm_controller.set_channel_full_on(PWM_EXTENDED_ENABLE_TOF, 1).unwrap();
        pwm_controller.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).unwrap();
        pwm_controller.set_channel_full_on(PWM_EXTENDED_RESET_TOF, 1).unwrap();
        
        let pwm_controller = Arc::new(Mutex::new(pwm_controller));
        let pwm_controller_for_vlx_en = pwm_controller.clone();
        let pwm_controller_for_vlx_rst = pwm_controller.clone();
        let pwm_controller = Some(pwm_controller);
        

        // Configuration des capteurs VLX
        let mut enable_front_tof = PinDriver::output(peripherals.pins.gpio16).unwrap();
        let mut enable_back_tof = PinDriver::output(peripherals.pins.gpio3).unwrap();

       
        let vlx_configs: [Config<Box<dyn FnMut(bool)>, Box<dyn FnMut(bool)>>; 3] = [
            Config {
                i2c_address: 0x30,  // Adresse du premier capteur
                sensor_type: SensorType::L1,
                enable_fn: Some(Box::new(move |enable| {
                    let mut pwm = pwm_controller_for_vlx_en.lock().unwrap();
                    if enable {
                        
                         pwm.set_channel_full_off(PWM_EXTENDED_ENABLE_TOF).unwrap();

                        std::thread::sleep(std::time::Duration::from_millis(100));

                        pwm.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
                        pwm.set_channel_full_on(PWM_EXTENDED_ENABLE_TOF, 0).unwrap();
                    
                    }      
                })),
                reset_fn: Some(Box::new(move |reset| {
                    let mut pwm = pwm_controller_for_vlx_rst.lock().unwrap();
                    if reset {
                        pwm.set_channel_on_off(PWM_EXTENDED_RESET_TOF, 0, 0).unwrap();

                        std::thread::sleep(std::time::Duration::from_millis(100));

                        pwm.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).unwrap();
                        pwm.set_channel_full_on(PWM_EXTENDED_RESET_TOF, 1).unwrap();
                    }
                })),
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

        // Initialize the PWM
        let timer_pwm = LedcTimerDriver::new(
                peripherals.ledc.timer0,
                &TimerConfig::new().frequency(Hertz(25_000)).resolution(esp_idf_svc::hal::ledc::Resolution::Bits10),
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
        
        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        Self {
            led_heartbeat: Some(led_heartbeat),
            vlx_sensors: Some(vlx_sensors),
            pwm_controller,
            left_pwm: Some(left_pwm),
            right_pwm: Some(right_pwm),
            ble,
        }
    }
}