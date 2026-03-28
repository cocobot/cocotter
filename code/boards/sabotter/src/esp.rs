use std::sync::{Arc, Mutex};
use std::time::Duration;
use embedded_can::blocking::Can;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use embedded_hal_bus::i2c::MutexDevice;
use esp_idf_svc::{
    bt::{Ble, BtDriver},
    hal::{
        can::{CanConfig, CanDriver},
        delay::BLOCK,
        gpio::{AnyOutputPin, Output, PinDriver},
        i2c::{I2cConfig, I2cDriver, I2cError},
        ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver},
        prelude::Peripherals,
        spi::{self, SpiConfig, SpiDeviceDriver, SpiDriver},
        units::Hertz,
        sys::{self, ets_delay_us},
    },
    nvs::EspDefaultNvsPartition,
};
use flume::{Receiver, Sender};
use ble::{BleBuilder, RomePeripheral};
use cancaner::CanInterface;
use pca9535::{Pca9535Immediate, ExpanderError, GPIOBank, StandardExpanderInterface};
use esp32_encoder::Encoder as EspEncoder;
use crate::{SabotterBoard, SabotterLeds, SabotterMotor};


type EspI2c = MutexDevice<'static, I2cDriver<'static>>;
type EspOutputPin = PinDriver<'static, AnyOutputPin, Output>;


pub struct EspSabotterBoard {
    com_led: Option<EspOutputPin>,
    imu_spi: Option<SpiDeviceDriver<'static, SpiDriver<'static>>>,
    can: Option<CanDriver<'static>>,
    ble: Option<BtDriver<'static, Ble>>,
    motors: Option<[SabotterMotor<EspEncoder<'static>, LedcDriver<'static>>; 3]>,
    motor_enable: Option<EspOutputPin>,
    gpio_expanders: GpioExpanders,
}

impl EspSabotterBoard {
    fn motor_leds(&self) -> [SharedGpioPin; 3] {
        [
            self.gpio_expanders.motors[0].get_pin(2),
            self.gpio_expanders.motors[1].get_pin(2),
            self.gpio_expanders.motors[2].get_pin(2),
        ]
    }
}

impl SabotterBoard for EspSabotterBoard {
    type I2c = EspI2c;
    type OutputPin = EspOutputPin;
    type ExOutputPin = SharedGpioPin;
    type Can = EspCanInterface;
    type Spi = SpiDeviceDriver<'static, SpiDriver<'static>>;
    type MotorEncoder = EspEncoder<'static>;
    type MotorPwm = LedcDriver<'static>;

    fn init() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let nvs = EspDefaultNvsPartition::take().unwrap();

        let peripherals = Peripherals::take().unwrap();

        // Initialize digital output pins
        let com_led = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio45)).unwrap();

        // Initialize SPI for IMU SCH16T
        let imu_spi = SpiDeviceDriver::new(
            SpiDriver::new(
                peripherals.spi2,
                peripherals.pins.gpio39,  // SCK
                peripherals.pins.gpio38,  // MOSI
                Some(peripherals.pins.gpio37),  // MISO
                &spi::config::DriverConfig::new().dma(spi::Dma::Disabled),
            ).unwrap(),
            // CS on GPIO36, active low (default)
            Some(peripherals.pins.gpio36),
            &SpiConfig::new().baudrate(Hertz(100_000)),
        ).unwrap();

        // Initialize main I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_driver = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio40, peripherals.pins.gpio41, &config).unwrap());
        // Leak `i2c_driver` to get a static lifetime
        // The I2cDriver never be dropped the end of the program, but it's fine
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));

        // Initialize ToF sensor I2C bus
        let config = I2cConfig::new().baudrate(Hertz(400_000));
        let i2c_vlx_driver = Mutex::new(I2cDriver::new(peripherals.i2c1, peripherals.pins.gpio21, peripherals.pins.gpio47, &config).unwrap());
        let i2c_vlx_driver_static = Box::leak(Box::new(i2c_vlx_driver));

        // Initialize LEDC timer for motors
        let motor_pwm = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(Hertz(25_000)).resolution(ledc::Resolution::Bits10),
        ).unwrap();

        // Prepare GPIO expanders
        let gpio_expanders = GpioExpanders {
            main: SharedGpio::new(Pca9535Immediate::new(MutexDevice::new(i2c_driver_static), 0b010_0100)),
            motors: [
                SharedGpio::new(Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0000)),
                SharedGpio::new(Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0001)),
                SharedGpio::new(Pca9535Immediate::new(MutexDevice::new(i2c_vlx_driver_static), 0b010_0010)),
            ],
        };

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
        let motor_enable = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio7)).unwrap();

        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        /* Unused
        let motor_reset = gpio_expander.get_pin(3);
        */

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
        */

        // Initialize CAN bus with correct pins (GPIO9/10)
        let can = CanDriver::new(peripherals.can, peripherals.pins.gpio9, peripherals.pins.gpio10, &CanConfig::new()).unwrap();

        /*
        // Initialize Lidar PWM
        let lidar_pwm = LedcTimerDriver::new(peripherals.ledc.timer1, &TimerConfig::default().frequency(1_000.Hz()))
            .and_then(|timer| LedcDriver::new(peripherals.ledc.channel6, timer, peripherals.pins.gpio42))
            .ok();
        */

        Self {
            com_led: Some(com_led),
            imu_spi: Some(imu_spi),
            can: Some(can),
            ble,
            motors: Some(motors),
            motor_enable: Some(motor_enable),
            gpio_expanders,
        }
    }

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin>> {
        Some(SabotterLeds {
            com: self.com_led.take()?,
            motors: self.motor_leds(),
        })
    }

    fn imu_spi(&mut self) -> Option<Self::Spi> {
        self.imu_spi.take()
    }

    fn can(&mut self) -> Option<Self::Can> {
        let can = self.can.take()?;
        Some(EspCanInterface::new(can))
    }

    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]> {
        // Take it at the beginning, to return None early if method has already been called
        let mut motor_enable = self.motor_enable.take()?;

        log::info!("Motor drivers initialization...");
        //TODO Make leds follow expected nFAULT state for better feedback
        let mut motor_leds = self.motor_leds();
        for pin in &mut motor_leds {
            let _ = pin.set_high();
        }

        // Motor driver startup procedure
        // See DRV8243 §7.7.2.1 HW Variant
        let mut nfaults = [
            self.gpio_expanders.motors[0].get_pin(3),
            self.gpio_expanders.motors[1].get_pin(3),
            self.gpio_expanders.motors[2].get_pin(3),
        ];

        let _ = motor_enable.set_low();
        std::thread::sleep(Duration::from_millis(10));
        let _ = motor_enable.set_high();

        // Assert all expanders nFAULT low
        while !nfaults.iter_mut().all(|pin| pin.is_low().unwrap_or(false)) {
            std::thread::sleep(Duration::from_millis(10));
        }
        std::thread::sleep(Duration::from_millis(10));

        let _ = motor_enable.set_low();
        // A short wait (between 5µs and 10µs) is required; don't replace with a `thread::sleep()`
        unsafe { ets_delay_us(10); }
        let _ = motor_enable.set_high();

        // Assert all expanders nFAULT high
        while !nfaults.iter_mut().all(|pin| pin.is_high().unwrap_or(false)) {
            std::thread::sleep(Duration::from_millis(10));
        }

        log::info!("Motor drivers initialized");
        for pin in &mut motor_leds {
            let _ = pin.set_high();
        }

        self.motors.take()
    }

    fn rome(&mut self, device_name: String) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        let ble = self.ble.take()?;
        // Note: for now, client is not used, so we can easily initialize both server and client
        // and drop the client. But if the client (and `.with_scanner()`) are needed,
        // another approach must be implemented. Maybe by changing the BLE API.
        let (ble_server, _ble_client) = BleBuilder::new(ble).run();
        let RomePeripheral { sender, receiver } = RomePeripheral::run(ble_server, device_name);
        Some((sender, receiver))
    }
}


type GpioExpander = Pca9535Immediate<EspI2c>;
type SharedGpioError = ExpanderError<I2cError>;

struct GpioExpanders {
    main: SharedGpio,
    motors: [SharedGpio; 3],
}

pub struct SharedGpioPin {
    handle: Arc<Mutex<GpioExpander>>,
    pin_number: u8,

    last_written_value: bool,
    is_output: bool,
}

impl SharedGpioPin {
    fn get_bank(&self) -> GPIOBank {
        match self.pin_number / 8 {
            0 => GPIOBank::Bank0,
            1 => GPIOBank::Bank1,
            
            _ => GPIOBank::Bank0, //should not happen
        }
    }

    fn get_pin_index_in_bank(&self) -> u8 {
        self.pin_number % 8
    }

    fn set_output(&mut self) -> Result<(), SharedGpioError> {
        let mut device = self.handle.lock().unwrap();
        device.pin_into_output(self.get_bank(), self.get_pin_index_in_bank())?;
        self.is_output = true;
        Ok(())
    }
}

impl ErrorType for SharedGpioPin {
    type Error = SharedGpioError;
}

impl InputPin for SharedGpioPin {
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_low(self.get_bank(), self.get_pin_index_in_bank())
    }

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_high(self.get_bank(), self.get_pin_index_in_bank())
    }
}

impl OutputPin for SharedGpioPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        if !self.is_output {
            self.set_output()?;
        }
        let mut device = self.handle.lock().unwrap();
        self.last_written_value = true;
        device.pin_set_high(self.get_bank(), self.get_pin_index_in_bank())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        if !self.is_output {
            self.set_output()?;
        }
        let mut device = self.handle.lock().unwrap();
        self.last_written_value = false;
        device.pin_set_low(self.get_bank(), self.get_pin_index_in_bank())
    }
}

impl StatefulOutputPin for SharedGpioPin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.last_written_value)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.last_written_value)
    }
}

struct SharedGpio {
    device : Arc<Mutex<GpioExpander>>,
}

impl SharedGpio {
    fn new(device: GpioExpander) -> SharedGpio {
        SharedGpio {
            device: Arc::new(Mutex::new(device)),
        }
    }

    //TODO Separate input and output pins
    fn get_pin(&self, pin_number: u8) -> SharedGpioPin {
        SharedGpioPin {
            handle: self.device.clone(),
            pin_number,
            last_written_value: false,
            is_output: false,
        }
    }
}


/// CAN interface with alert handling and auto-recovering
///
/// `init()` will start a thread that will run forever.
/// This thread will stop if the driver is uninstalled, which should never because the driver is leaked.
///
/// The `driver` must be static for the `EspCanInterface` to be `Sync + Send`, which is required
/// to then call `can_receive()` from a separate thread.
#[derive(Clone)]
pub struct EspCanInterface {
    driver: &'static CanDriver<'static>,
}

// SAFETY: We only use `driver.transmit()` and `driver.receive()`.
// Both are safe and don't actually use the instance directly.
unsafe impl Send for EspCanInterface {}
unsafe impl Sync for EspCanInterface {}

impl EspCanInterface {
    /// Create and initialize the interface, setup alert monitoring
    pub fn new(mut driver: CanDriver<'static>) -> Self {
        driver.start().expect("TWAI start failed");

        // Note: the `CanDriver` object will concurrently with raw twai calls

        // Enable error alerts for monitoring + auto-recovery
        let alerts_to_enable = sys::TWAI_ALERT_BUS_ERROR
            | sys::TWAI_ALERT_TX_FAILED
            | sys::TWAI_ALERT_RX_QUEUE_FULL
            | sys::TWAI_ALERT_BUS_OFF
            | sys::TWAI_ALERT_ABOVE_ERR_WARN
            | sys::TWAI_ALERT_ERR_PASS
            | sys::TWAI_ALERT_ARB_LOST
            | sys::TWAI_ALERT_BUS_RECOVERED
            | sys::TWAI_ALERT_RX_FIFO_OVERRUN;

        let ret = unsafe { sys::twai_reconfigure_alerts(alerts_to_enable, core::ptr::null_mut()) };
        if ret != 0 {
            log::error!("CAN: failed to configure alerts (err={ret})");
        }

        // Monitor thread: watches for CAN alerts, logs errors, auto-recovers on bus-off
        std::thread::Builder::new()
            .name("can-mon".into())
            .stack_size(4096)
            .spawn(move || loop {
                let mut alerts: u32 = 0;
                match unsafe { sys::twai_read_alerts(&mut alerts, BLOCK) } {
                    0 => {}
                    sys::ESP_ERR_INVALID_STATE => { return; }
                    _ => { continue; }
                }

                if alerts & sys::TWAI_ALERT_BUS_ERROR != 0 {
                    log::warn!("CAN: bus error detected");
                }
                if alerts & sys::TWAI_ALERT_TX_FAILED != 0 {
                    log::warn!("CAN: TX failed");
                }
                if alerts & sys::TWAI_ALERT_ARB_LOST != 0 {
                    log::warn!("CAN: arbitration lost");
                }
                if alerts & sys::TWAI_ALERT_RX_QUEUE_FULL != 0 {
                    log::warn!("CAN: RX queue full, messages lost");
                }
                if alerts & sys::TWAI_ALERT_RX_FIFO_OVERRUN != 0 {
                    log::warn!("CAN: RX FIFO overrun");
                }
                if alerts & sys::TWAI_ALERT_ABOVE_ERR_WARN != 0 {
                    log::warn!("CAN: error counter above warning threshold");
                }
                if alerts & sys::TWAI_ALERT_ERR_PASS != 0 {
                    log::error!("CAN: entered error passive state");
                }

                // Auto-recovery: bus-off → initiate recovery
                if alerts & sys::TWAI_ALERT_BUS_OFF != 0 {
                    log::error!("CAN: bus off! initiating recovery...");
                    let ret = unsafe { sys::twai_initiate_recovery() };
                    if ret != 0 {
                        log::error!("CAN: recovery initiation failed (err={ret})");
                    }
                }

                // After recovery completes, driver is in STOPPED state → restart
                if alerts & sys::TWAI_ALERT_BUS_RECOVERED != 0 {
                    log::info!("CAN: bus recovered, restarting...");
                    let ret = unsafe { sys::twai_start() };
                    if ret != 0 {
                        log::error!("CAN: restart after recovery failed (err={ret})");
                    } else {
                        log::info!("CAN: restarted successfully");
                    }
                }

                // Log detailed status on serious errors
                if alerts
                    & (sys::TWAI_ALERT_BUS_ERROR
                        | sys::TWAI_ALERT_ERR_PASS
                        | sys::TWAI_ALERT_BUS_OFF
                        | sys::TWAI_ALERT_BUS_RECOVERED)
                    != 0
                {
                    Self::log_status();
                }
            })
            .expect("spawn can-mon");

        // Leak the driver to get a static reference
        // SAFETY: the `CanDriver` object will always be valid; not dropping it will just not uninstall it
        let driver = Box::leak(Box::new(driver));
        Self { driver }
    }

    fn log_status() {
        let mut status: sys::twai_status_info_t = Default::default();
        if unsafe { sys::twai_get_status_info(&mut status) } == 0 {
            let state_str = match status.state {
                0 => &"STOPPED",
                1 => &"RUNNING",
                2 => &"BUS_OFF",
                3 => &"RECOVERING",
                _ => &"UNKNOWN",
            };
            log::warn!(
                "CAN status: state={state_str}, tx_err={}, rx_err={}, tx_failed={}, rx_missed={}, bus_err={}",
                status.tx_error_counter,
                status.rx_error_counter,
                status.tx_failed_count,
                status.rx_missed_count,
                status.bus_error_count,
            );
        }
    }
}

impl CanInterface for EspCanInterface {
    type Frame = <CanDriver<'static> as Can>::Frame;
    type Error = <CanDriver<'static> as Can>::Error;

    fn can_transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error> {
        Ok(self.driver.transmit(frame, BLOCK)?)
    }

    fn can_receive(&self) -> Result<Self::Frame, Self::Error> {
        Ok(self.driver.receive(BLOCK)?)
    }
}
