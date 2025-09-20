pub use pca9535;

use esp_idf_svc::{bt::Ble, hal::{
    adc::{oneshot::{AdcChannelDriver, AdcDriver}, ADC1}, can::{CanConfig, CanDriver}, gpio::*, i2c::{I2cConfig, I2cDriver}, ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver}, prelude::*, spi::{config::DriverConfig, SpiConfig, SpiDeviceDriver, SpiDriver}, uart::{UartConfig, UartDriver}, units::Hertz
}};
use esp_idf_svc::{nvs::EspDefaultNvsPartition, bt::BtDriver};
use embedded_hal_bus::i2c::MutexDevice;
use pca9535::Pca9535Immediate;
use vlx::{VLX, Config, SensorType};
use esp32_encoder::Encoder;
use std::{rc::Rc, sync::Mutex};


pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;

// Type aliases
pub type LedHeartbeat = PinDriver<'static, Gpio45, Output>;
// Motor control types
pub type MotorPwm = LedcDriver<'static>;

// VLX sensors configuration
pub type VlxSensors = VLX<I2CType, 1>;

// GPIO expander type
pub type GpioExpander = Pca9535Immediate<I2CType>;

// CAN bus type
pub type CanBus = CanDriver<'static>;

// UART types
pub type UartAsserv = UartDriver<'static>;
pub type UartLidar = UartDriver<'static>;

// ADC type for battery monitoring
pub type BatteryAdcPin = Gpio1;
pub type BatteryAdc = AdcChannelDriver<'static, BatteryAdcPin, Rc<AdcDriver<'static, ADC1>>>;

// SPI IMU driver
pub type ImuSpi = SpiDeviceDriver<'static, SpiDriver<'static>>;

// Motor structure with encoder
pub struct Motor {
    pub pwm: MotorPwm,
    pub dir: MotorPwm,
    pub encoder: Encoder<'static>,
}

// Bluetooth driver
pub type Bt = BtDriver<'static, Ble>;


pub struct BoardSabotter {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub motors: [Option<Motor>; 3],
    pub motor_gpio_expander: [Option<GpioExpander>; 3],
    pub vlx_sensors: Option<VlxSensors>,
    pub gpio_expander: Option<GpioExpander>,
    pub can_bus: Option<CanBus>,
    pub uart_asserv: Option<UartAsserv>,
    pub uart_lidar: Option<UartLidar>,
    pub battery_adc: Option<BatteryAdc>,
    pub imu_spi: Option<ImuSpi>,
    pub mot_ena: Option<PinDriver<'static, Gpio7, Output>>,
    pub lidar_pwm: Option<LedcDriver<'static>>,
    pub ble: Option<Bt>,
}

impl BoardSabotter {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let nvs = EspDefaultNvsPartition::take().unwrap();

        let peripherals = Peripherals::take().unwrap();

        // Initialize heartbeat LED
        let led_heartbeat = PinDriver::output(peripherals.pins.gpio45).ok();

        // Initialize main I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_driver : Mutex<I2cDriver<'static>> = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio40, peripherals.pins.gpio41, &config).unwrap());
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));
        let i2c_gpio_expander = MutexDevice::new(i2c_driver_static);
        let gpio_expander = Some(Pca9535Immediate::new(i2c_gpio_expander, 0b010_0100));
    
        // Initialize ToF sensor I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_vlx_driver : Mutex<I2cDriver<'static>> = Mutex::new(I2cDriver::new(peripherals.i2c1, peripherals.pins.gpio21, peripherals.pins.gpio47, &config).unwrap());
        let i2c_vlx_driver_static = Box::leak(Box::new(i2c_vlx_driver));
        let i2c_vlx_bus_vlx : MutexDevice<'static, I2cDriver<'static>> =  MutexDevice::new(i2c_vlx_driver_static);

        let vlx_configs: [Config<Box<dyn FnMut(bool)>, Box<dyn FnMut(bool)>>; 1] = [
            Config {
                i2c_address: 0x30,  // Adresse du premier capteur
                sensor_type: SensorType::L1,
                enable_fn: None,
                reset_fn: None,
            },           
        ];
        let vlx_sensors = Some(VLX::new(i2c_vlx_bus_vlx, vlx_configs)); 


        // Initialize LEDC timer for motors
        let timer_pwm = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(25.kHz().into()).resolution(esp_idf_svc::hal::ledc::Resolution::Bits10),
        ).unwrap();

        // Initialize motors with encoders
        let mut motors: [Option<Motor>; 3] = [None, None, None];
        let mut motor_gpio_expander: [Option<GpioExpander>; 3] = [None, None, None];
        
        // Motor 0
        if let (Ok(pwm), Ok(dir), Ok(encoder)) = (
            LedcDriver::new(peripherals.ledc.channel0, &timer_pwm, peripherals.pins.gpio17),
            LedcDriver::new(peripherals.ledc.channel1, &timer_pwm, peripherals.pins.gpio18),
            Encoder::new(peripherals.pcnt0, peripherals.pins.gpio11, peripherals.pins.gpio12)
        ) {
            motors[0] = Some(Motor { pwm, dir, encoder });
        }
        // Initialize main I2C bus
        let i2c_gpio_expander_mot_0 = MutexDevice::new(i2c_vlx_driver_static);
        motor_gpio_expander[0] = Some(Pca9535Immediate::new(i2c_gpio_expander_mot_0, 0b010_0000));
    

        // Motor 1  
        if let (Ok(pwm), Ok(dir), Ok(encoder)) = (
            LedcDriver::new(peripherals.ledc.channel2, &timer_pwm, peripherals.pins.gpio3),
            LedcDriver::new(peripherals.ledc.channel3, &timer_pwm, peripherals.pins.gpio8),
            Encoder::new(peripherals.pcnt1, peripherals.pins.gpio15, peripherals.pins.gpio16)
        ) {
            motors[1] = Some(Motor { pwm, dir, encoder });
        }

        // Motor 2
        if let (Ok(pwm), Ok(dir), Ok(encoder)) = (
            LedcDriver::new(peripherals.ledc.channel4, &timer_pwm, peripherals.pins.gpio35),
            LedcDriver::new(peripherals.ledc.channel5, &timer_pwm, peripherals.pins.gpio48),
            Encoder::new(peripherals.pcnt2, peripherals.pins.gpio13, peripherals.pins.gpio14)
        ) {
            motors[2] = Some(Motor { pwm, dir, encoder });
        }

        // Initialize UART for asserv communication
        let uart_asserv = UartDriver::new(
            peripherals.uart1,
            peripherals.pins.gpio6,   // TX
            peripherals.pins.gpio5,   // RX
            Option::<AnyIOPin>::None, // CTS
            Option::<AnyIOPin>::None, // RTS
            &UartConfig::default().baudrate(Hertz(115_200)),
        ).ok();

        //// Initialize UART for lidar (RX only)
        //let uart_lidar = UartDriver::new(
        //    peripherals.uart2,
        //    peripherals.pins.gpio0,   // TX (dummy pin)
        //    peripherals.pins.gpio2,   // RX
        //    Option::<AnyIOPin>::None, // CTS
        //    Option::<AnyIOPin>::None, // RTS
        //    &UartConfig::default().baudrate(Hertz(1_000_000)),
        //).ok();

        //// Initialize CAN bus with correct pins (GPIO9/10)
        //let can_bus = CanDriver::new(
        //    peripherals.can,
        //    peripherals.pins.gpio9,  // TX
        //    peripherals.pins.gpio10,  // RX
        //    &CanConfig::new(),
        //).ok();

        // Initialize SPI for IMU SCH16T
        use esp_idf_svc::hal::spi::Dma;
        let spi_config = SpiConfig::new()
            .baudrate(Hertz(100_000)); // Start with slower speed for debugging

        let spi_driver = SpiDriver::new(
            peripherals.spi2,
            peripherals.pins.gpio39,  // SCK
            peripherals.pins.gpio38,  // MOSI
            Some(peripherals.pins.gpio37),  // MISO
            &DriverConfig::new().dma(Dma::Disabled),
        ).ok();

        let imu_spi = if let Some(spi) = spi_driver {
            // CS on GPIO36, active low (default)
            SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio36), &spi_config).ok()
        } else {
            None
        };

        // Initialize ADC for battery monitoring
        // ADC initialization - simplified for now
        let battery_adc = None;

        // Initialize interrupt pins
        let mot_ena = PinDriver::output(peripherals.pins.gpio7).ok();

        // Initialize Lidar PWM
        let lidar_timer = LedcTimerDriver::new(
            peripherals.ledc.timer1,
            &TimerConfig::default().frequency(1_000.Hz()),
        ).ok();
        
        let lidar_pwm = if let Some(timer) = &lidar_timer {
            LedcDriver::new(peripherals.ledc.channel6, timer, peripherals.pins.gpio42).ok()
        } else {
            None
        };

        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        Self {
            led_heartbeat,
            motors,
            vlx_sensors,
            gpio_expander,
            motor_gpio_expander,
            can_bus: None,
            uart_asserv,
            uart_lidar: None,
            battery_adc,
            imu_spi,
            mot_ena,
            lidar_pwm,
            ble,
        }
    }
}