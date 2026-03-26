#[cfg(target_os = "espidf")]
pub mod esp;
#[cfg(not(target_os = "espidf"))]
pub mod mock;

use std::sync::mpsc::{Receiver, Sender};
use embedded_can::blocking::Can;
use embedded_hal::{
    digital::StatefulOutputPin,
    i2c::I2c,
    pwm::SetDutyCycle,
    spi::SpiDevice,
};
pub use board_common::{BatteryLevel, Color};
pub use board_common::hal::{BatteryReader, Encoder};

#[cfg(target_os = "espidf")]
pub use esp::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
pub use mock::MockSabotterBoard;


pub trait SabotterBoard {
    type I2c: I2c;
    type OutputPin: StatefulOutputPin;
    type ExOutputPin: StatefulOutputPin;
    type Spi: SpiDevice + Send;
    type Can: Can;
    type MotorEncoder: Encoder<i32> + Send;  //TODO exact type
    type MotorPwm: SetDutyCycle + Send;

    /// Initialize the board and return its instance
    ///
    /// Call this method at the very beginning of main.
    fn init() -> Self;

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin>>;
    fn imu_spi(&mut self) -> Option<Self::Spi>;
    fn can(&mut self) -> Option<Self::Can>;
    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]>;

    /// Configure and return ROME interface
    fn rome(&mut self, device_name: String) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)>;
}


pub struct SabotterMotor<E: Encoder<i32>, PWM: SetDutyCycle> {
    pub pwm: PWM,
    pub dir: PWM,
    pub encoder: E,
}


/// Normal leds, driven directly
pub struct SabotterLeds<Led: StatefulOutputPin, MotorLed: StatefulOutputPin> {
    pub com: Led,
    pub motors: [MotorLed; 3],
}

