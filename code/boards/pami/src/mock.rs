//! Host-side mock board for PAMI.
//!
//! Connects to the simulator on init and provides non-panicking stubs for
//! every `PamiBoard` getter. The asserv's wheel consigns flow via the
//! `sim/src/pami_asserv.rs` sim-variant module.

use core::convert::Infallible;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use embedded_graphics::mock_display::MockDisplay;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_hal::digital::{ErrorType, OutputPin, StatefulOutputPin};
use embedded_hal_mock::eh1::pwm::Mock as SetDutyCycleMock;
use flume::{Receiver, Sender};

use board_common::mock::{MockBatteryReader, MockEncoder};
use sim_protocol::{Capabilities, RobotKind, SimMsgC2S, DEFAULT_SOCKET_PATH, PROTOCOL_VERSION};
use tca6408::TCA6408;
use vlx::{DistanceData, VlxError, VlxSensor, ZoneAlarm};

use crate::{PamiBoard, PamiButtons, PamiButtonsState, PamiLeds, PamiMotors, PamiPwmController};

// ---------------------------------------------------------------------------
// Simple pins
// ---------------------------------------------------------------------------

/// I2c mock that silently accepts every transaction. Used only to make the
/// PWM controller initialize without I/O.
pub struct NoopI2c;

impl embedded_hal::i2c::ErrorType for NoopI2c {
    type Error = core::convert::Infallible;
}

impl embedded_hal::i2c::I2c for NoopI2c {
    fn transaction(
        &mut self,
        _addr: u8,
        _ops: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct NoopLed;

impl OutputPin for NoopLed {
    fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
    fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
}
impl StatefulOutputPin for NoopLed {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> { Ok(false) }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> { Ok(true) }
}
impl ErrorType for NoopLed {
    type Error = Infallible;
}

// ---------------------------------------------------------------------------
// Board
// ---------------------------------------------------------------------------

pub struct MockPamiBoard;

pub type PamiDisplay = MockDisplay<BinaryColor>;

/// First known PAMI MAC; matches an entry in `pami/src/config.rs` so the
/// process doesn't panic on startup.
const DEFAULT_SIM_PAMI_MAC: [u8; 6] = [0x74, 0x4D, 0xBD, 0x51, 0xCF, 0x8A];

impl PamiBoard for MockPamiBoard {
    type BatteryReader = MockBatteryReader;
    type I2c = NoopI2c;
    type Led = NoopLed;
    type Display = PamiDisplay;
    type Buttons = MockPamiButtons;
    type Vlx = MockVlxSensor;
    type MotorEncoder = MockEncoder<i32>;
    type MotorPwm = SetDutyCycleMock;

    fn init() -> Self {
        env_logger_init_once();
        let socket = std::env::var("MECA_SIM_SOCKET")
            .unwrap_or_else(|_| DEFAULT_SOCKET_PATH.to_string());
        let robot_id = std::env::var("MECA_SIM_ROBOT_ID").unwrap_or_else(|_| "pami".into());
        let hello = SimMsgC2S::Hello {
            version: PROTOCOL_VERSION,
            robot_id,
            kind: RobotKind::Pami,
            caps: Capabilities::VLX | Capabilities::ROME | Capabilities::DIFF2,
            requested_start: None,
        };
        match sim_client::SimClient::connect(&socket, hello) {
            Ok(client) => {
                sim_client::install(client);
                log::info!("pami mock connected to simulator at {socket}");
            }
            Err(e) => {
                log::error!("pami mock could not connect to simulator at {socket}: {e}");
                std::process::exit(1);
            }
        }
        Self
    }

    fn restart() {
        log::warn!("Restart requested");
    }

    fn bt_mac_address(&self) -> [u8; 6] {
        DEFAULT_SIM_PAMI_MAC
    }

    fn battery_reader(&mut self) -> Option<Self::BatteryReader> {
        Some(MockBatteryReader::default())
    }

    fn emergency_stop(&mut self) -> Option<Box<dyn FnMut() -> bool>> {
        Some(Box::new(|| false))
    }

    fn starting_cord(&mut self) -> Option<Box<dyn FnMut() -> bool>> {
        // Time-gated: pretend the cord is inserted for 500 ms, then removed,
        // so the match-start wait unblocks.
        let start = Instant::now();
        Some(Box::new(move || start.elapsed() > Duration::from_millis(500)))
    }

    fn leds(&mut self) -> Option<PamiLeds<Self::Led>> {
        Some(PamiLeds { esp: NoopLed, com: NoopLed })
    }

    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>> { None }

    fn buttons(&mut self) -> Option<Self::Buttons> {
        Some(MockPamiButtons)
    }

    fn display(&mut self) -> Option<Self::Display> {
        let mut d = MockDisplay::new();
        d.set_allow_out_of_bounds_drawing(true);
        d.set_allow_overdraw(true);
        Some(d)
    }

    fn vlx_sensor(&mut self) -> Option<Self::Vlx> {
        Some(MockVlxSensor)
    }

    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>> {
        // The sim-variant of pami_asserv.rs does not use board.motors().
        None
    }

    fn pwm_controller(&mut self) -> Option<PamiPwmController<Self::I2c>> {
        PamiPwmController::new(NoopI2c, 0x40).ok()
    }

    fn rome<F: Fn([u8; 6], u32) + Send + Sync + 'static>(
        &mut self,
        _device_name: String,
        _passkey_notifier: F,
    ) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        // Dummy channels; outgoing drained, incoming never signals.
        let (out_tx, out_rx) = flume::unbounded();
        let (_in_tx, in_rx) = flume::unbounded::<Box<[u8]>>();
        std::thread::Builder::new()
            .name("mock-rome-drain".into())
            .spawn(move || while out_rx.recv().is_ok() {})
            .expect("spawn mock-rome-drain");
        Some((out_tx, in_rx))
    }
}

// ---------------------------------------------------------------------------
// VLX mock — serves the sim's VlxDistance pushes.
// ---------------------------------------------------------------------------

pub struct MockVlxSensor;

impl VlxSensor for MockVlxSensor {
    fn init(&mut self) -> Result<(), VlxError> { Ok(()) }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {
        let sim = sim_client::global();
        let mm = sim
            .vlx_rx
            .try_recv()
            .unwrap_or(0xFFFF); // "no target" sentinel until M6 raycasts populate
        Ok(DistanceData::new_single(mm))
    }

    fn set_alarms(&mut self, _alarms: &[ZoneAlarm]) -> Result<(), VlxError> { Ok(()) }
}

pub struct MockPamiButtons;

impl PamiButtons for MockPamiButtons {
    fn read_state(&mut self) -> PamiButtonsState {
        PamiButtonsState::default()
    }
}

fn env_logger_init_once() {
    static DONE: OnceLock<()> = OnceLock::new();
    DONE.get_or_init(|| {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
            .try_init()
            .ok();
    });
}
