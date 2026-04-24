//! Host-side mock board for Galipeur.
//!
//! This mock connects to the simulator process via `sim_client` and provides
//! non-panicking stubs for every `SabotterBoard` getter so that the real
//! `GalipeurRoutines` can run end-to-end off-target. Hardware interactions
//! that need sim feedback (motor consigns, CAN frames, lidar bytes) will be
//! progressively wired through the sim client in later milestones.

use core::convert::Infallible;
use std::thread;
use std::time::{Duration, Instant};

use embedded_can::blocking::Can;
use embedded_can as can;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use embedded_hal_mock::eh1::{
    i2c::Mock as I2cMock,
    pwm::Mock as SetDutyCycleMock,
    spi::Mock as SpiMock,
};
use flume::{Receiver, Sender};
use smart_leds_trait::{SmartLedsWrite, RGB8};

use board_common::mock::{MockBatteryReader, MockEncoder};
use cancaner::CanInterface;
use sim_protocol::{Capabilities, RobotKind, SimMsgC2S, DEFAULT_SOCKET_PATH, PROTOCOL_VERSION};

use crate::{OtaHandler, SabotterBoard, SabotterInputs, SabotterLeds, SabotterMotor, SabotterUart};

// ---------------------------------------------------------------------------
// UART mock (lidar)
// ---------------------------------------------------------------------------

/// UART lidar backed by the simulator. The sim sends full LD06 packets via
/// `SimMsgS2C::Ld06Bytes`; we buffer them here and serve them byte-by-byte
/// to the galipeur sensors thread, which then runs its usual parser.
pub struct MockUartLidar {
    buffer: std::sync::Mutex<std::collections::VecDeque<u8>>,
}

impl Default for MockUartLidar {
    fn default() -> Self {
        Self { buffer: std::sync::Mutex::new(std::collections::VecDeque::new()) }
    }
}

impl SabotterUart for MockUartLidar {
    fn read(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let sim = sim_client::global();

        // Top up the buffer if it's empty, with a short timeout so the
        // sensors thread can still do its periodic battery check when no
        // packets arrive.
        {
            let internal = self.buffer.lock().map_err(|_| ())?;
            if internal.is_empty() {
                drop(internal);
                match sim.ld06_rx.recv_timeout(Duration::from_millis(100)) {
                    Ok(bytes) => {
                        self.buffer.lock().map_err(|_| ())?.extend(bytes);
                    }
                    Err(_) => return Err(()),
                }
            }
        }

        // Also drain any other packets that arrived while we were processing.
        {
            let mut internal = self.buffer.lock().map_err(|_| ())?;
            while let Ok(more) = sim.ld06_rx.try_recv() {
                internal.extend(more);
            }
            let n = buf.len().min(internal.len());
            for b in buf.iter_mut().take(n) {
                *b = internal.pop_front().unwrap();
            }
            Ok(n)
        }
    }
}

// ---------------------------------------------------------------------------
// Pin mocks
// ---------------------------------------------------------------------------

/// InputPin that reports `is_low=true` for the first 500 ms after creation,
/// then `is_high=true` thereafter.
///
/// This is enough to unblock galipeur's strat, which waits first for the
/// starter cable to be inserted (is_low) and then for it to be removed
/// (is_high) before starting the match.
pub struct MockInputPin {
    created: Instant,
}

impl Default for MockInputPin {
    fn default() -> Self {
        Self { created: Instant::now() }
    }
}

impl InputPin for MockInputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.created.elapsed() > Duration::from_millis(500))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.created.elapsed() <= Duration::from_millis(500))
    }
}
impl ErrorType for MockInputPin {
    type Error = Infallible;
}

pub struct NoopOutputPin;

impl OutputPin for NoopOutputPin {
    fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
    fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
}
impl StatefulOutputPin for NoopOutputPin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> { Ok(false) }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> { Ok(true) }
}
impl ErrorType for NoopOutputPin {
    type Error = Infallible;
}

// ---------------------------------------------------------------------------
// LED mocks
// ---------------------------------------------------------------------------

pub struct MockSmartLeds;

impl SmartLedsWrite for MockSmartLeds {
    type Color = RGB8;
    type Error = Infallible;

    /// Collect every pixel written on the strip and forward it to the
    /// simulator as a single `NeopixelFrame`. The sim then maps the
    /// wire-order strip onto the fixtures declared in its config.
    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>,
    {
        let pixels: Vec<[u8; 3]> = iterator
            .into_iter()
            .map(|p| {
                let c: RGB8 = p.into();
                [c.r, c.g, c.b]
            })
            .collect();
        // `send` errors are non-fatal — losing a frame just skips one
        // visual update. The sim client may not be installed during
        // board self-tests either.
        if let Some(sim) = sim_client::try_global() {
            let _ = sim.send(sim_protocol::SimMsgC2S::NeopixelFrame { pixels });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CAN mock (wired to the simulator via sim_client)
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct MockCan;

fn frame_to_sim(frame: &MockCanFrame) -> Option<(u16, [u8; 8], u8)> {
    let id = match frame.id {
        can::Id::Standard(s) => s.as_raw(),
        can::Id::Extended(_) => return None,
    };
    let mut data = [0u8; 8];
    let len = frame.data.len().min(8);
    data[..len].copy_from_slice(&frame.data[..len]);
    Some((id, data, len as u8))
}

fn sim_to_frame(id: u16, data: [u8; 8], len: u8) -> MockCanFrame {
    let sid = can::StandardId::new(id).unwrap_or(can::StandardId::ZERO);
    MockCanFrame {
        id: can::Id::Standard(sid),
        data: data[..len as usize].to_vec(),
    }
}

impl Can for MockCan {
    type Frame = MockCanFrame;
    type Error = MockCanError;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        <Self as CanInterface>::can_transmit(self, frame)
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        <Self as CanInterface>::can_receive(self)
    }
}

pub struct MockCanFrame {
    pub id: can::Id,
    pub data: Vec<u8>,
}

impl can::Frame for MockCanFrame {
    fn new(id: impl Into<can::Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        Some(Self { id: id.into(), data: data.to_vec() })
    }
    fn new_remote(_id: impl Into<can::Id>, _dlc: usize) -> Option<Self> { None }
    fn is_extended(&self) -> bool { matches!(self.id, can::Id::Extended(_)) }
    fn is_remote_frame(&self) -> bool { false }
    fn id(&self) -> can::Id { self.id }
    fn dlc(&self) -> usize { self.data.len() }
    fn data(&self) -> &[u8] { &self.data }
}

#[derive(Debug, Default)]
pub struct MockCanError;

impl can::Error for MockCanError {
    fn kind(&self) -> can::ErrorKind { can::ErrorKind::Other }
}

impl CanInterface for MockCan {
    type Frame = MockCanFrame;
    type Error = MockCanError;

    fn can_transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error> {
        if let Some((id, data, len)) = frame_to_sim(frame) {
            let sim = sim_client::global();
            let _ = sim.send(SimMsgC2S::CanFrame { id, data, len });
        }
        Ok(())
    }

    fn can_receive(&self) -> Result<Self::Frame, Self::Error> {
        let sim = sim_client::global();
        match sim.can_rx.recv_timeout(Duration::from_millis(1000)) {
            Ok((id, data, len)) => Ok(sim_to_frame(id, data, len)),
            Err(_) => Err(MockCanError),
        }
    }
}

// ---------------------------------------------------------------------------
// Board
// ---------------------------------------------------------------------------

pub struct MockSabotterBoard;

impl SabotterBoard for MockSabotterBoard {
    type I2c = I2cMock;
    type OutputPin = NoopOutputPin;
    type ExOutputPin = NoopOutputPin;
    type ExInputPin = MockInputPin;
    type Can = MockCan;
    type Spi = SpiMock<u8>;
    type MotorEncoder = MockEncoder<i32>;
    type MotorPwm = SetDutyCycleMock;
    type SmartLeds = MockSmartLeds;
    type BatteryReader = MockBatteryReader;
    type UartLidar = MockUartLidar;

    fn init() -> Self {
        env_logger_init_once();

        let socket = std::env::var("MECA_SIM_SOCKET")
            .unwrap_or_else(|_| DEFAULT_SOCKET_PATH.to_string());
        let hello = SimMsgC2S::Hello {
            version: PROTOCOL_VERSION,
            robot_id: "galipeur".into(),
            kind: RobotKind::Galipeur,
            caps: Capabilities::CAN
                | Capabilities::LD06
                | Capabilities::GYRO
                | Capabilities::HOLO3,
            requested_start: None,
        };
        match sim_client::SimClient::connect(&socket, hello) {
            Ok(client) => {
                sim_client::install(client);
                log::info!("connected to simulator at {socket}");
            }
            Err(e) => {
                log::error!("could not connect to simulator at {socket}: {e}");
                log::error!("start the sim binary first (`cargo run -p sim`)");
                std::process::exit(1);
            }
        }
        Self
    }

    fn leds(&mut self) -> Option<SabotterLeds<Self::OutputPin, Self::ExOutputPin, Self::SmartLeds>> {
        Some(SabotterLeds {
            com: NoopOutputPin,
            heartbeat: NoopOutputPin,
            motors: [NoopOutputPin, NoopOutputPin, NoopOutputPin],
            rgba: MockSmartLeds,
        })
    }

    fn inputs(&mut self) -> Option<SabotterInputs<Self::ExInputPin, Self::ExInputPin>> {
        Some(SabotterInputs {
            color: MockInputPin::default(),
            starter: MockInputPin::default(),
        })
    }

    fn imu_spi(&mut self) -> Option<Self::Spi> {
        // Sim variant of MovementLowLevelHardware does not call this.
        None
    }

    fn battery_reader(&mut self) -> Option<Self::BatteryReader> {
        Some(MockBatteryReader::default())
    }

    fn lidar_uart(&mut self) -> Option<Self::UartLidar> {
        Some(MockUartLidar::default())
    }

    fn can(&mut self) -> Option<Self::Can> {
        Some(MockCan)
    }

    fn motors(&mut self) -> Option<[SabotterMotor<Self::MotorEncoder, Self::MotorPwm>; 3]> {
        // Sim variant of MovementLowLevelHardware does not call this.
        None
    }

    fn rome(
        &mut self,
        _device_name: String,
        _other_ota_handlers: Vec<Box<dyn OtaHandler>>,
    ) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        // Outgoing: caller sends, we drop. Incoming: never triggered (for M1).
        let (out_tx, out_rx) = flume::unbounded();
        let (_in_tx, in_rx) = flume::unbounded::<Box<[u8]>>();
        thread::Builder::new()
            .name("mock-rome-drain".into())
            .spawn(move || {
                while out_rx.recv().is_ok() {}
            })
            .expect("spawn mock-rome-drain");
        Some((out_tx, in_rx))
    }
}

fn env_logger_init_once() {
    use std::sync::OnceLock;
    static DONE: OnceLock<()> = OnceLock::new();
    DONE.get_or_init(|| {
        env_logger::Builder::from_env(
            env_logger::Env::default().default_filter_or("info"),
        )
        .try_init()
        .ok();
    });
}
