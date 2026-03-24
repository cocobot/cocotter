use embedded_hal_mock::eh1::{
    digital::{Mock as PinMock},
    i2c::{Mock as I2cMock},
    pwm::{Mock as SetDutyCycleMock},
    spi::{Mock as SpiMock},
};
use board_common::mock::MockEncoder;
use crate::{SabotterBoard, SabotterLeds, SabotterMotor};


pub struct MockSabotterBoard;

pub type SabotterDisplay = MockDisplay<BinaryColor>;

impl SabotterBoard for MockSabotterBoard {
    type I2c = I2cMock;
    type OutputPin = PinMock;
    type ExOutputPin = PinMock;
    type Spi = SpiMock;
    type MotorEncoder = MockEncoder<i32>;
    type MotorPwm = SetDutyCycleMock;

    fn init() -> Self {
        Self
    }

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin>> {
        None
    }

    fn imu_spi(&mut self) -> Option<Self::Spi> {
        None
    }

    fn can(&mut self) -> Option<Self::Can> {
        None
    }

    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]> {
        None
    }
}
