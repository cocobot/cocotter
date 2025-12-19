#[cfg(target_os = "espidf")]
pub mod esp;
#[cfg(not(target_os = "espidf"))]
pub mod mock;

use std::sync::mpsc::{Receiver, Sender};
use embedded_hal::{
    digital::StatefulOutputPin,
    i2c::I2c,
    pwm::SetDutyCycle,
};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::BinaryColor,
};
pub use board_common::{BatteryLevel, Encoder};
use tca6408::TCA6408;
use vlx::VlxSensor;

#[cfg(target_os = "espidf")]
pub use esp::EspPamiBoard;
#[cfg(not(target_os = "espidf"))]
pub use mock::MockPamiBoard;


pub trait PamiBoard {
    type BatteryLevel: BatteryLevel;
    type I2c: I2c;
    type Led: StatefulOutputPin;
    type Display: DrawTarget<Color=BinaryColor>;
    type Vlx: VlxSensor;
    type MotorEncoder: Encoder<i32>;
    type MotorPwm: SetDutyCycle;

    /// Initialize the board and return its instance
    ///
    /// Call this method at the very beginning of main.
    fn init() -> Self;
    /// Restart the board
    fn restart();

    /// Return Bluetooth MAC address of the device
    fn bt_mac_address(&self) -> [u8; 6];

    fn battery_level(&mut self) -> Option<Self::BatteryLevel>;
    fn emergency_stop(&mut self) -> Option<Box<dyn FnMut() -> bool>>;
    fn starting_cord(&mut self) -> Option<Box<dyn FnMut() -> bool>>;
    fn heartbeat_led(&mut self) -> Option<Self::Led>;
    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>>;
    fn buttons(&mut self) -> Option<PamiButtons<Self::I2c>>;
    fn display(&mut self) -> Option<Self::Display>;
    fn vlx_sensor(&mut self) -> Option<Self::Vlx>;
    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>>;

    /// Configure and return ROME interface
    fn rome<F: Fn([u8; 6], u32) + Send + Sync +'static>(&mut self, device_name: String, passkey_notifier: F) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)>;
}


pub struct PamiMotor<E: Encoder<i32>, PWM: SetDutyCycle> {
    pub encoder: E,
    pub pwm_forward: PWM,
    pub pwm_backward: PWM,
}

pub struct PamiMotors<E: Encoder<i32>, PWM: SetDutyCycle> {
    pub left: PamiMotor<E, PWM>,
    pub right: PamiMotor<E, PWM>,
}


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DpadState { None, Middle, Up, Down, Left, Right }

impl DpadState {
    pub fn as_char(&self) -> char {
        match self {
            Self::None => '-',
            Self::Middle => 'X',
            Self::Up => '^',
            Self::Down => 'v',
            Self::Left => '<',
            Self::Right => '>',
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PamiButtonsState(pub u8);

impl PamiButtonsState {
    pub fn dpad(&self) -> DpadState {
        // Assume 1 action on the dpad
        match self.0 & 0b11111 {
            0b00001 => DpadState::Right,
            0b00010 => DpadState::Down,
            0b00100 => DpadState::Up,
            0b01000 => DpadState::Middle,
            0b10000 => DpadState::Left,
            _ => DpadState::None,
        }
    }

    pub fn switches(&self) -> [bool; 3] {
        [
            self.0 & 0b100_00000 != 0,
            self.0 & 0b010_00000 != 0,
            self.0 & 0b001_00000 != 0,
        ]
    }
}

pub struct PamiButtons<I2C: I2c>(TCA6408<I2C>);

impl<I2C: I2c> PamiButtons<I2C> {
    /// Read new button state
    pub fn read_state(&mut self) -> PamiButtonsState {
        //XXX Errors are ignored
        PamiButtonsState(self.0.read_inputs().unwrap_or(0))
    }
}

