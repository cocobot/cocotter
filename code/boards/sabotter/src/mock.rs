use embedded_can::blocking::Can;
use embedded_can as can;
use embedded_hal::digital::InputPin;
use embedded_hal_mock::eh1::{
    digital::{Mock as PinMock},
    i2c::{Mock as I2cMock},
    pwm::{Mock as SetDutyCycleMock},
    spi::{Mock as SpiMock},
};
use flume::{Receiver, Sender};
use smart_leds_trait::{SmartLedsWrite, RGB8};
use board_common::mock::{MockBatteryReader, MockEncoder};
use cancaner::CanInterface;
use crate::{OtaHandler, SabotterBoard, SabotterInputs, SabotterLeds, SabotterMotor, SabotterUart};


pub struct MockUartLidar;

impl SabotterUart for MockUartLidar {
    fn read(&self, _buf: &mut [u8]) -> Result<usize, ()> {
        Err(())
    }
}

pub struct MockSabotterBoard;

impl SabotterBoard for MockSabotterBoard {
    type I2c = I2cMock;
    type OutputPin = PinMock;
    type ExOutputPin = PinMock;
    type ExInputPin = MockInputPin;
    type Can = MockCan;
    type Spi = SpiMock<u8>;
    type MotorEncoder = MockEncoder<i32>;
    type MotorPwm = SetDutyCycleMock;
    type SmartLeds = MockSmartLeds;
    type BatteryReader = MockBatteryReader;
    type UartLidar = MockUartLidar;

    fn init() -> Self {
        Self
    }

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin, Self::SmartLeds>> {
        None
    }

    fn inputs(&mut self) -> Option<SabotterInputs<Self::ExInputPin, Self::ExInputPin>> {
        None
    }

    fn imu_spi(&mut self) -> Option<Self::Spi> {
        None
    }

    fn battery_reader(&mut self) -> Option<Self::BatteryReader> {
        None
    }

    fn lidar_uart(&mut self) -> Option<Self::UartLidar> {
        None
    }

    fn can(&mut self) -> Option<Self::Can> {
        None
    }

    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]> {
        None
    }

    fn rome(&mut self, _device_name: String, _other_ota_handlers: Vec<Box<dyn OtaHandler>>) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        None
    }
}

#[derive(Clone)]
pub struct MockCan;

impl Can for MockCan {
    type Frame = MockCanFrame;
    type Error = MockCanError;

    fn transmit(&mut self, _frame: &Self::Frame) -> Result<(), Self::Error> {
        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        Err(Self::Error::default())
    }
}

pub struct MockCanFrame {
    pub id: can::Id,
    pub data: Vec<u8>,
}

impl can::Frame for MockCanFrame {
    fn new(_id: impl Into<can::Id>, _data: &[u8]) -> Option<Self> {
        None
    }

    fn new_remote(_id: impl Into<can::Id>, _dlc: usize) -> Option<Self> {
        None
    }

    fn is_extended(&self) -> bool {
        false 
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> can::Id {
        can::Id::Standard(can::StandardId::ZERO)
    }

    fn dlc(&self) -> usize {
        0
    }

    fn data(&self) -> &[u8] {
        &[]
    }
}

#[derive(Debug, Default)]
pub struct MockCanError;

impl can::Error for MockCanError {
    fn kind(&self) -> can::ErrorKind {
        can::ErrorKind::Other
    }
}

impl CanInterface for MockCan {
    type Frame = MockCanFrame;
    type Error = MockCanError;

    fn can_transmit(&self, _frame: &Self::Frame) -> Result<(), Self::Error> {
        Ok(())
    }

    fn can_receive(&self) -> Result<Self::Frame, Self::Error> {
        Err(Self::Error::default())
    }
}

pub struct MockInputPin;

impl InputPin for MockInputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }
}

impl embedded_hal::digital::ErrorType for MockInputPin {
    type Error = core::convert::Infallible;
}

pub struct MockSmartLeds;

impl SmartLedsWrite for MockSmartLeds {
    type Color = RGB8;
    type Error = core::convert::Infallible;

    fn write<T, I>(&mut self, _iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>,
    {
        Ok(())
    }
}

