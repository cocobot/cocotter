use embedded_hal_mock::eh1::{
    digital::{Mock as PinMock},
    i2c::{Mock as I2cMock},
    pwm::{Mock as SetDutyCycleMock},
};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    mock_display::MockDisplay,
};
use flume::{Receiver, Sender};
use board_common::mock::{MockBatteryReader, MockEncoder};
use tca6408::TCA6408;
use vlx::{DistanceData, VlxError, VlxSensor, ZoneAlarm};
use crate::{PamiBoard, PamiButtons, PamiButtonsState, PamiLeds, PamiMotors, PamiPwmController};


pub struct MockPamiBoard;

pub type PamiDisplay = MockDisplay<BinaryColor>;

impl PamiBoard for MockPamiBoard {
    type BatteryReader = MockBatteryReader;
    type I2c = I2cMock;
    type Led = PinMock;
    type Display = PamiDisplay;
    type Buttons = MockPamiButtons;
    type Vlx = MockVlxSensor;
    type MotorEncoder = MockEncoder<i32>;
    type MotorPwm = SetDutyCycleMock;

    fn init() -> Self {
        Self
    }

    fn restart() {
        // Don't actually restart (it's not trivial)
        log::warn!("Restart requested");
    }

    fn bt_mac_address(&self) -> [u8; 6] {
        [0; 6]
    }

    fn battery_reader(&mut self) -> Option<Self::BatteryReader> {
        None
    }

    fn emergency_stop(&mut self) -> Option<Box<dyn FnMut() -> bool>> {
        None
    }

    fn starting_cord(&mut self) -> Option<Box<dyn FnMut() -> bool>> {
        None
    }

    fn leds(&mut self) -> Option<PamiLeds<Self::Led>> {
        None
    }

    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>> {
        None
    }

    fn buttons(&mut self) -> Option<Self::Buttons> {
        None
    }

    fn display(&mut self) -> Option<Self::Display> {
        None
    }

    fn vlx_sensor(&mut self) -> Option<Self::Vlx> {
        None
    }

    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>> {
        None
    }

    fn pwm_controller(&mut self) -> Option<PamiPwmController<Self::I2c>> {
        None
    }

    fn rome<F: Fn([u8; 6], u32) + Send + Sync +'static>(&mut self, _device_name: String, _passkey_notifier: F) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        None
    }
}


pub struct MockVlxSensor;

impl VlxSensor for MockVlxSensor {
    fn init(&mut self) -> Result<(), VlxError> {
        Ok(())
    }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {
        Ok(DistanceData::new_single(100))
    }

    fn set_alarms(&mut self, _alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        Ok(())
    }
}

pub struct MockPamiButtons;

impl PamiButtons for MockPamiButtons {
    fn read_state(&mut self) -> PamiButtonsState {
        PamiButtonsState::default()
    }
}

