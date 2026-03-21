use std::sync::{Arc, Mutex};
use std::time::Duration;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use embedded_hal_bus::i2c::MutexDevice;
use esp_idf_svc::{
    hal::{
        gpio::{AnyOutputPin, Output, PinDriver},
        i2c::{I2cConfig, I2cDriver, I2cError},
        ledc::{self, config::TimerConfig, LedcDriver, LedcTimerDriver},
        prelude::Peripherals,
        spi::{self, SpiConfig, SpiDeviceDriver, SpiDriver},
        units::Hertz,
        sys::ets_delay_us,
    },
    nvs::EspDefaultNvsPartition,
};
use pca9535::{Pca9535Immediate, ExpanderError, GPIOBank, StandardExpanderInterface};
use esp32_encoder::Encoder as EspEncoder;
use crate::{SabotterBoard, SabotterLeds, SabotterMotor};


type EspI2c = MutexDevice<'static, I2cDriver<'static>>;
type EspOutputPin = PinDriver<'static, AnyOutputPin, Output>;


pub struct EspSabotterBoard<'d> {
    com_led: Option<EspOutputPin>,
    imu_spi: Option<SpiDeviceDriver<'static, SpiDriver<'static>>>,
    motors: Option<[SabotterMotor<EspEncoder<'d>, LedcDriver<'static>>; 3]>,
    motor_enable: Option<EspOutputPin>,
    gpio_expanders: GpioExpanders,
}

impl<'d> EspSabotterBoard<'d> {
    fn motor_leds(&self) -> [SharedGpioPin; 3] {
        [
            self.gpio_expanders.motors[0].get_pin(2),
            self.gpio_expanders.motors[1].get_pin(2),
            self.gpio_expanders.motors[2].get_pin(2),
        ]
    }
}

impl<'d> SabotterBoard for EspSabotterBoard<'d> {
    type I2c = EspI2c;
    type OutputPin = EspOutputPin;
    type ExOutputPin = SharedGpioPin;
    type Spi = SpiDeviceDriver<'static, SpiDriver<'static>>;
    type MotorEncoder = EspEncoder<'d>;
    type MotorPwm = LedcDriver<'static>;

    fn init() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let _nvs = EspDefaultNvsPartition::take().unwrap();

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

        // Initialize CAN bus with correct pins (GPIO9/10)
        let can_bus = CanDriver::new(
            peripherals.can,
            peripherals.pins.gpio9,  // TX
            peripherals.pins.gpio10,  // RX
            &CanConfig::new(),
        ).ok();
        */

        /*
        // Initialize Lidar PWM
        let lidar_pwm = LedcTimerDriver::new(peripherals.ledc.timer1, &TimerConfig::default().frequency(1_000.Hz()))
            .and_then(|timer| LedcDriver::new(peripherals.ledc.channel6, timer, peripherals.pins.gpio42))
            .ok();

        let ble = BtDriver::new(peripherals.modem, Some(nvs.clone())).ok();
        */

        Self {
            com_led: Some(com_led),
            imu_spi: Some(imu_spi),
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
