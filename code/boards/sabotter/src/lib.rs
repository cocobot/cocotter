use esp_idf_svc::hal::{
    adc::{oneshot::{AdcChannelDriver, AdcDriver}, ADC1}, can::{CanConfig, CanDriver}, gpio::*, i2c::{I2cConfig, I2cDriver}, ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver}, prelude::*, spi::{config::DriverConfig, SpiConfig, SpiDeviceDriver, SpiDriver}, uart::{UartConfig, UartDriver}, units::Hertz
};
use shared_bus::{I2cProxy, BusManagerStd};
use vlx::{VLX, Config, SensorType};
use tca6408::TCA6408;
use esp32_encoder::Encoder;
use std::rc::Rc;

// Type aliases
pub type LedHeartbeat = PinDriver<'static, Gpio45, Output>;
pub type I2cMainBusManager = BusManagerStd<I2cDriver<'static>>;
pub type I2cMainBusProxy = I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>;
pub type I2cTofBusManager = BusManagerStd<I2cDriver<'static>>;
pub type I2cTofBusProxy = I2cProxy<'static, std::sync::Mutex<I2cDriver<'static>>>;

// Motor control types
pub type MotorPwm = LedcDriver<'static>;

// VLX sensors configuration
pub type VlxSensors = VLX<I2cTofBusProxy, <I2cTofBusProxy as embedded_hal::blocking::i2c::WriteRead>::Error, 8, fn(bool), fn(bool)>;

// GPIO expander type
pub type GpioExpander = TCA6408<I2cMainBusProxy>;

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

pub struct BoardSabotter {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub motors: [Option<Motor>; 3],
    pub vlx_sensors: Option<VlxSensors>,
    pub gpio_expander: Option<GpioExpander>,
    pub can_bus: Option<CanBus>,
    pub uart_asserv: Option<UartAsserv>,
    pub uart_lidar: Option<UartLidar>,
    pub battery_adc: Option<BatteryAdc>,
    pub imu_spi: Option<ImuSpi>,
    pub tof_irq: Option<PinDriver<'static, Gpio7, Input>>,
    pub lidar_pwm: Option<LedcDriver<'static>>,
}

impl BoardSabotter {
    pub fn new() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();

        let peripherals = Peripherals::take().unwrap();

        // Initialize heartbeat LED
        let led_heartbeat = PinDriver::output(peripherals.pins.gpio45).ok();

        // Initialize main I2C bus (for TCA6408 GPIO expander)
        let i2c_main_config = I2cConfig::new().baudrate(Hertz(100_000));
        let i2c_main_driver = I2cDriver::new(
            peripherals.i2c0,
            peripherals.pins.gpio40,  // SDA
            peripherals.pins.gpio41,  // SCL
            &i2c_main_config,
        ).ok();

        let gpio_expander = if let Some(i2c) = i2c_main_driver {
            let i2c_main_bus_manager = BusManagerStd::new(i2c);
            let i2c_main_bus_manager_static = Box::leak(Box::new(i2c_main_bus_manager));
            let i2c_proxy = i2c_main_bus_manager_static.acquire_i2c();
            
            // Initialize TCA6408 GPIO expander
            let mut expander = TCA6408::new(i2c_proxy, 0x20);
            if expander.init().is_ok() {
                Some(expander)
            } else {
                log::error!("Failed to initialize TCA6408");
                None
            }
        } else {
            None
        };

        // Initialize ToF sensor I2C bus
        let i2c_tof_config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_tof_driver = I2cDriver::new(
            peripherals.i2c1,
            peripherals.pins.gpio21,  // SDA
            peripherals.pins.gpio47,  // SCL
            &i2c_tof_config,
        ).ok();

        let vlx_sensors = if let Some(i2c) = i2c_tof_driver {
            let i2c_tof_bus_manager = BusManagerStd::new(i2c);
            let i2c_tof_bus_manager_static = Box::leak(Box::new(i2c_tof_bus_manager));
            let i2c_proxy = i2c_tof_bus_manager_static.acquire_i2c();
            
            // Configure VLX sensors
            let vlx_configs: [Config<fn(bool), fn(bool)>; 8] = [
                Config { i2c_address: 0x29, sensor_type: SensorType::L1, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2A, sensor_type: SensorType::L1, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2B, sensor_type: SensorType::L1, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2C, sensor_type: SensorType::L1, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2D, sensor_type: SensorType::L5, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2E, sensor_type: SensorType::L5, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x2F, sensor_type: SensorType::L5, enable_fn: None, reset_fn: None },
                Config { i2c_address: 0x30, sensor_type: SensorType::L5, enable_fn: None, reset_fn: None },
            ];
            
            Some(VLX::new(i2c_proxy, vlx_configs))
        } else {
            None
        };

        // Initialize LEDC timer for motors
        let timer_pwm = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(25.kHz().into()).resolution(esp_idf_svc::hal::ledc::Resolution::Bits10),
        ).unwrap();

        // Initialize motors with encoders
        let mut motors: [Option<Motor>; 3] = [None, None, None];
        
        // Motor 0
        if let (Ok(pwm), Ok(dir), Ok(encoder)) = (
            LedcDriver::new(peripherals.ledc.channel0, &timer_pwm, peripherals.pins.gpio17),
            LedcDriver::new(peripherals.ledc.channel1, &timer_pwm, peripherals.pins.gpio18),
            Encoder::new(peripherals.pcnt0, peripherals.pins.gpio11, peripherals.pins.gpio12)
        ) {
            motors[0] = Some(Motor { pwm, dir, encoder });
        }

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

        // Initialize UART for lidar (RX only)
        let uart_lidar = UartDriver::new(
            peripherals.uart2,
            peripherals.pins.gpio0,   // TX (dummy pin)
            peripherals.pins.gpio2,   // RX
            Option::<AnyIOPin>::None, // CTS
            Option::<AnyIOPin>::None, // RTS
            &UartConfig::default().baudrate(Hertz(1_000_000)),
        ).ok();

        // Initialize CAN bus with correct pins (GPIO9/10)
        let can_bus = CanDriver::new(
            peripherals.can,
            peripherals.pins.gpio9,  // TX
            peripherals.pins.gpio10,  // RX
            &CanConfig::new(),
        ).ok();

        // Initialize SPI for IMU
        let spi_config = SpiConfig::default().baudrate(Hertz(1_000_000));
        let spi_driver = SpiDriver::new(
            peripherals.spi2,
            peripherals.pins.gpio39,  // SCK
            peripherals.pins.gpio38,  // MOSI
            Some(peripherals.pins.gpio37),  // MISO
            &DriverConfig::default(),
        ).ok();

        let imu_spi = if let Some(spi) = spi_driver {
            SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio36), &spi_config).ok()
        } else {
            None
        };

        // Initialize ADC for battery monitoring
        // ADC initialization - simplified for now
        let battery_adc = None;

        // Initialize interrupt pins
        let tof_irq = PinDriver::input(peripherals.pins.gpio7).ok();

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

        Self {
            led_heartbeat,
            motors,
            vlx_sensors,
            gpio_expander,
            can_bus,
            uart_asserv,
            uart_lidar,
            battery_adc,
            imu_spi,
            tof_irq,
            lidar_pwm,
        }
    }
}