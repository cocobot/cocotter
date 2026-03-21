use std::sync::Mutex;
use esp_idf_svc::{
    bt::{Ble, BtDriver},
    hal::{
        can::CanDriver,
        gpio::{Output, PinDriver, Gpio7, Gpio45},
        i2c::{I2cConfig, I2cDriver, I2cError},
        ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver},
        prelude::*,
        spi::{self, config::DriverConfig, SpiConfig, SpiDeviceDriver, SpiDriver},
        units::Hertz
    },
    nvs::EspDefaultNvsPartition,
};
use board_common::esp::EspEncoder;
use embedded_hal_bus::i2c::MutexDevice;
use pca9535::Pca9535Immediate;
pub use pca9535;


// Type aliases
pub type I2CType = MutexDevice<'static, I2cDriver<'static>>;
pub type LedHeartbeat = PinDriver<'static, Gpio45, Output>;
pub type MotorPwm = LedcDriver<'static>;
pub type GpioExpander = Pca9535Immediate<I2CType>;
pub type GpioExpanderError = pca9535::ExpanderError<I2cError>;
pub type CanBus = CanDriver<'static>;
pub type ImuSpi = SpiDeviceDriver<'static, SpiDriver<'static>>;


pub struct SabotterMotor {
    pub pwm: MotorPwm,
    pub dir: MotorPwm,
    pub encoder: EspEncoder<'static>,
}

// Bluetooth driver
pub type Bt = BtDriver<'static, Ble>;


pub struct BoardSabotter {
    pub led_heartbeat: Option<LedHeartbeat>,
    pub motors: Option<[SabotterMotor; 3]>,
    pub motor_gpio_expanders: Option<[GpioExpander; 3]>,
    pub gpio_expander: Option<GpioExpander>,
    pub can_bus: Option<CanBus>,
    pub imu_spi: Option<ImuSpi>,
    pub mot_ena: Option<PinDriver<'static, Gpio7, Output>>,
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
        let i2c_driver = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio40, peripherals.pins.gpio41, &config).unwrap());
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));
        let i2c_gpio_expander = MutexDevice::new(i2c_driver_static);
        let gpio_expander = Some(Pca9535Immediate::new(i2c_gpio_expander, 0b010_0100));

        // Initialize ToF sensor I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_vlx_driver = Mutex::new(I2cDriver::new(peripherals.i2c1, peripherals.pins.gpio21, peripherals.pins.gpio47, &config).unwrap());
        let i2c_vlx_driver_static = Box::leak(Box::new(i2c_vlx_driver));

        // Initialize LEDC timer for motors
        let motor_pwm = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(25_000.Hz()).resolution(ledc::Resolution::Bits10),
        ).unwrap();

        // Initialize motors with encoders
        let motors = [
            SabotterMotor {
                pwm: LedcDriver::new(peripherals.ledc.channel0, &motor_pwm, peripherals.pins.gpio17).unwrap(),
                dir: LedcDriver::new(peripherals.ledc.channel1, &motor_pwm, peripherals.pins.gpio18).unwrap(),
                encoder: EspEncoder::new(peripherals.pcnt0, peripherals.pins.gpio12, peripherals.pins.gpio11).unwrap(),
            },
            SabotterMotor {
                pwm: LedcDriver::new(peripherals.ledc.channel2, &motor_pwm, peripherals.pins.gpio3).unwrap(),
                dir: LedcDriver::new(peripherals.ledc.channel3, &motor_pwm, peripherals.pins.gpio8).unwrap(),
                encoder: EspEncoder::new(peripherals.pcnt1, peripherals.pins.gpio16, peripherals.pins.gpio15).unwrap(),
            },
            SabotterMotor {
                pwm: LedcDriver::new(peripherals.ledc.channel4, &motor_pwm, peripherals.pins.gpio35).unwrap(),
                dir: LedcDriver::new(peripherals.ledc.channel5, &motor_pwm, peripherals.pins.gpio48).unwrap(),
                encoder: EspEncoder::new(peripherals.pcnt2, peripherals.pins.gpio14, peripherals.pins.gpio13).unwrap(),
            },
        ];

        // Initialize main I2C bus
        let motor_gpio_expanders = [
            Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0000),
            Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0001),
            Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0010),
        ];

        /*
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
        */

        // Initialize SPI for IMU SCH16T
        let spi_config = SpiConfig::new()
            .baudrate(Hertz(100_000)); // Start with slower speed for debugging

        let spi_driver = SpiDriver::new(
            peripherals.spi2,
            peripherals.pins.gpio39,  // SCK
            peripherals.pins.gpio38,  // MOSI
            Some(peripherals.pins.gpio37),  // MISO
            &DriverConfig::new().dma(spi::Dma::Disabled),
        ).ok();

        let imu_spi = if let Some(spi) = spi_driver {
            // CS on GPIO36, active low (default)
            SpiDeviceDriver::new(spi, Some(peripherals.pins.gpio36), &spi_config).ok()
        } else {
            None
        };

        // Initialize interrupt pins
        let mot_ena = PinDriver::output(peripherals.pins.gpio7).ok();

        /*
        // Initialize Lidar PWM
        let lidar_pwm = LedcTimerDriver::new(peripherals.ledc.timer1, &TimerConfig::default().frequency(1_000.Hz()))
            .and_then(|timer| LedcDriver::new(peripherals.ledc.channel6, timer, peripherals.pins.gpio42))
            .ok();
        */

        let ble = BtDriver::new(peripherals.modem, Some(nvs.clone())).ok();

        Self {
            led_heartbeat,
            motors: Some(motors),
            gpio_expander,
            motor_gpio_expanders: Some(motor_gpio_expanders),
            can_bus: None,
            imu_spi,
            mot_ena,
            ble,
        }
    }
}
