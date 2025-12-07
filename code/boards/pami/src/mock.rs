use std::sync::mpsc::{Receiver, Sender};
use embedded_hal_mock::eh1::{
    digital::{Mock as PinMock},
    i2c::{Mock as I2cMock},
    pwm::{Mock as SetDutyCycleMock},
};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    mock_display::MockDisplay,
};
use tca6408::TCA6408;
use vlx::{DistanceData, VlxError, VlxSensor, ZoneAlarm};
use crate::{Encoder, PamiBoard, PamiButtons, PamiMotors, Vbatt};


pub struct MockPamiBoard;

impl PamiBoard for MockPamiBoard {
    type I2c = I2cMock;
    type Led = PinMock;
    type Display = MockDisplay<BinaryColor>;
    type Vbatt = MockVbatt;
    type Vlx = MockVlxSensor;
    type MotorEncoder = MockEncoder;
    type MotorPwm = SetDutyCycleMock;
    type EmergencyStop = PinMock;

    fn init() -> Self {
        Self
    }

    fn bt_mac_address(&self) -> [u8; 6] {
        [0; 6]
    }

    fn heartbeat_led(&mut self) -> Option<Self::Led> {
        None
    }

    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>> {
        None
    }

    fn buttons(&mut self) -> Option<PamiButtons<Self::I2c>> {
        None
    }

    fn display(&mut self) -> Option<Self::Display> {
        None
    }

    fn vbatt(&mut self) -> Option<Self::Vbatt> {
        None
    }

    fn vlx_sensor(&mut self) -> Option<Self::Vlx> {
        None
    }

    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>> {
        None
    }

    fn emergency_stop(&mut self) -> Option<Self::EmergencyStop> {
        None
    }

    fn rome<F: Fn([u8; 6], u32) + Send + Sync +'static>(&mut self, _device_name: String, _passkey_notifier: F) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        None
    }
}


pub struct MockVbatt;

impl Vbatt for MockVbatt {
    fn read_vbatt(&mut self) -> (u16, u8) {
        (1000, 50)
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

pub struct MockEncoder;

impl<T: Default> Encoder<T> for MockEncoder {
    type Error = ();

    fn get_value(&self) -> Result<T, Self::Error> {
        Ok(T::default())
    }
}

