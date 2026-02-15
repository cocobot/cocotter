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
pub use board_common::{BatteryLevel, Color, Encoder};
use pwm_pca9685::{self, Pca9685};
use pwm_pca9685::{Channel as Pca9685Channel};
use tca6408::TCA6408;
use vlx::VlxSensor;

#[cfg(target_os = "espidf")]
pub use esp::EspPamiBoard;
#[cfg(not(target_os = "espidf"))]
pub use mock::MockPamiBoard;

type Pca9685Error<I2C> = pwm_pca9685::Error<<I2C as embedded_hal::i2c::ErrorType>::Error>;


pub trait PamiBoard {
    type BatteryLevel: BatteryLevel;
    type I2c: I2c;
    type Led: StatefulOutputPin;
    type Buttons: PamiButtons;
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
    fn leds(&mut self) -> Option<PamiLeds<Self::Led>>;
    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>>;
    fn buttons(&mut self) -> Option<Self::Buttons>;
    fn display(&mut self) -> Option<Self::Display>;
    fn vlx_sensor(&mut self) -> Option<Self::Vlx>;
    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>>;
    fn pwm_controller(&mut self) -> Option<PamiPwmController<Self::I2c>>;

    /// Configure and return ROME interface
    fn rome<F: Fn([u8; 6], u32) + Send + Sync +'static>(&mut self, device_name: String, passkey_notifier: F) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)>;
}


/// Normal leds, driven directly
pub struct PamiLeds<Led: StatefulOutputPin> {
    pub esp: Led,
    pub com: Led,
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

impl Default for DpadState {
    fn default() -> Self {
        Self::None
    }
}

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

#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub struct PamiButtonsState {
    // Assume 1 active action on the dpad
    pub dpad: DpadState,
    pub switches: u8,  // Only 4 bits are relevant
}

impl PamiButtonsState {
    pub const fn switch(&self, idx: u8) -> bool {
        self.switches & (1 << idx) != 0
    }
}

pub trait PamiButtons {
    /// Read new button state
    fn read_state(&mut self) -> PamiButtonsState;
}


//const PWM_CONTROLLER_SERVO: [Pca9685Channel; 4] = [Pca9685Channel::C0, Pca9685Channel::C1, Pca9685Channel::C2, Pca9685Channel::C3];
const PWM_CONTROLLER_FAN: Pca9685Channel = Pca9685Channel::C5;
//const PWM_CONTROLLER_RESET_TOF: Pca9685Channel = Pca9685Channel::C6;
//const PWM_CONTROLLER_ENABLE_TOF: Pca9685Channel = Pca9685Channel::C7;
const PWM_CONTROLLER_VBAT_RGB: [Pca9685Channel; 3] = [Pca9685Channel::C10, Pca9685Channel::C8, Pca9685Channel::C9];
const PWM_CONTROLLER_LINE_LED: Pca9685Channel = Pca9685Channel::C11;
const PWM_CONTROLLER_GROUND_RGB: [Pca9685Channel; 3] = [Pca9685Channel::C12, Pca9685Channel::C13, Pca9685Channel::C14];

pub struct PamiPwmController<I2C: I2c>(Pca9685<I2C>);

impl<I2C: I2c> PamiPwmController<I2C> {
    fn new(i2c: I2C, addr: u8) -> Result<Self, Pca9685Error<I2C>> {
        Ok(Self(Pca9685::new(i2c, addr)?))
    }

    pub fn init(&mut self) -> Result<(), Pca9685Error<I2C>> {
        // The driver is not always reset when PAMI is switched on
        self.0.set_prescale(100)?;  // For the servo
        self.0.enable()?;
        // Reset to default state
        self.0.set_channel_full_off(Pca9685Channel::All)?;
        self.0.set_channel_on(Pca9685Channel::All, 0)?;
        Ok(())
    }

    pub fn set_battery_rgb(&mut self, color: Color) {
        self.set_inverted_channel_duty(PWM_CONTROLLER_VBAT_RGB[0], color.r);
        self.set_inverted_channel_duty(PWM_CONTROLLER_VBAT_RGB[1], color.g);
        self.set_inverted_channel_duty(PWM_CONTROLLER_VBAT_RGB[2], color.b);
    }

    pub fn set_ground_rgb(&mut self, color: Color) {
        self.set_inverted_channel_duty(PWM_CONTROLLER_GROUND_RGB[0], color.r);
        self.set_inverted_channel_duty(PWM_CONTROLLER_GROUND_RGB[1], color.g);
        self.set_inverted_channel_duty(PWM_CONTROLLER_GROUND_RGB[2], color.b);
    }

    pub fn set_fan_speed(&mut self, speed: f32) {
        self.set_channel_duty(PWM_CONTROLLER_FAN, speed);
    }

    pub fn set_line_led(&mut self, value: f32) {
        self.set_channel_duty(PWM_CONTROLLER_LINE_LED, value);
    }

    fn set_inverted_channel_duty(&mut self, channel: Pca9685Channel, value: f32) {
        if value == 0.0 {
            let _ = self.0.set_channel_full_on(channel, 0);
        } else {
            let _ = self.0.set_channel_on(channel, ((4095.0 * value) as u16).min(4095));
        }
    }

    fn set_channel_duty(&mut self, channel: Pca9685Channel, value: f32) {
        let _ = self.0.set_channel_on_off(channel, 0, 4095 - ((4095.0 * value) as u16).min(4095));
    }
}
