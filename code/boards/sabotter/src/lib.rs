#[cfg(target_os = "espidf")]
pub mod esp;
#[cfg(not(target_os = "espidf"))]
pub mod mock;

use embedded_hal::{
    digital::{StatefulOutputPin, InputPin},
    i2c::I2c,
    pwm::SetDutyCycle,
    spi::SpiDevice,
};
pub use smart_leds_trait::{SmartLedsWrite, RGB8};
use flume::{Receiver, Sender};
use cancaner::CanInterface;
pub use board_common::{BatteryLevel, Color};
pub use board_common::hal::{BatteryReader, Encoder, OtaHandler};

#[cfg(target_os = "espidf")]
pub use esp::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
pub use mock::MockSabotterBoard;


/// ADC reader for battery voltage (returns raw ADC value in mV)
pub trait SabotterAdc: Send {
    fn read(&mut self) -> Result<u16, ()>;
}

pub trait SabotterBoard {
    type I2c: I2c;
    type OutputPin: StatefulOutputPin + Send;
    type ExOutputPin: StatefulOutputPin + Send;
    type ExInputPin: InputPin + Send;
    type Spi: SpiDevice + Send;
    type Can: CanInterface + 'static;
    type MotorEncoder: Encoder<i32> + Send;  //TODO exact type
    type MotorPwm: SetDutyCycle + Send;
    type SmartLeds: SmartLedsWrite<Color: From<RGB8>> + Send;
    type Adc: SabotterAdc + 'static;


    /// Initialize the board and return its instance
    ///
    /// Call this method at the very beginning of main.
    fn init() -> Self;

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin, Self::SmartLeds>>;
    fn inputs(&mut self) -> Option<SabotterInputs<Self::ExInputPin, Self::ExInputPin>>;
    fn imu_spi(&mut self) -> Option<Self::Spi>;
    fn can(&mut self) -> Option<Self::Can>;
    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]>;
    fn battery_adc(&mut self) -> Option<Self::Adc>;

    /// Configure and return ROME interface
    fn rome(&mut self, device_name: String, other_ota_handlers: Option<Vec<Box<dyn OtaHandler>>>) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)>;
}


pub struct SabotterMotor<E: Encoder<i32>, PWM: SetDutyCycle> {
    pub pwm: PWM,
    pub dir: PWM,
    pub encoder: E,
}


pub struct SabotterLeds<Led: StatefulOutputPin, MotorLed: StatefulOutputPin, SmartLeds: SmartLedsWrite> {
    pub com: MotorLed,
    pub heartbeat: Led,
    pub motors: [MotorLed; 3],
    pub rgba: SmartLeds,
}
pub struct SabotterInputs<Color: InputPin, Starter: InputPin> {
    pub color: Color,
    pub starter: Starter,
}

