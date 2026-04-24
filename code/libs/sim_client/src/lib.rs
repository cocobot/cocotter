//! Host-only IPC client for robot mocks.
//!
//! Each robot process connects to the simulator once via a global singleton.
//! The mock board calls [`install`] during its `init()`; subsequent accesses
//! go through [`global`]. The client maintains a background thread that
//! reads incoming [`SimMsgS2C`] frames and dispatches them to typed flume
//! channels.

use std::io::{self, Write};
use std::os::unix::net::UnixStream;
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::Duration;

use flume::{Receiver, Sender};
use sim_protocol::{recv_msg, send_msg, Pose2D, SimMsgC2S, SimMsgS2C};

static GLOBAL: OnceLock<Arc<SimClient>> = OnceLock::new();

/// Install the client as the process-wide singleton. Call once from the
/// mock board's `init()`. Subsequent calls panic.
pub fn install(client: SimClient) -> Arc<SimClient> {
    let arc = Arc::new(client);
    GLOBAL
        .set(arc.clone())
        .map_err(|_| "sim_client already installed")
        .unwrap();
    arc
}

/// Return the singleton. Panics if [`install`] has not been called yet.
pub fn global() -> Arc<SimClient> {
    GLOBAL
        .get()
        .expect("sim_client not installed (call install() from MockBoard::init first)")
        .clone()
}

pub struct SimClient {
    tx: Mutex<UnixStream>,
    pub assigned_start: Pose2D,
    pub sim_tick_ms: u16,

    pub encoder_holo_rx: Receiver<[f32; 3]>,
    pub encoder_diff_rx: Receiver<[f32; 2]>,
    pub gyro_rx: Receiver<f32>,
    pub ld06_rx: Receiver<Vec<u8>>,
    pub can_rx: Receiver<(u16, [u8; 8], u8)>,
    pub vlx_rx: Receiver<u16>,
    pub battery_rx: Receiver<u16>,
    pub rome_rx: Receiver<Vec<u8>>,
    pub tick_rx: Receiver<(u64, u64)>,
}

impl SimClient {
    /// Connect to the simulator, send `hello`, wait for `HelloAck`, spawn
    /// the RX dispatcher thread.
    pub fn connect(socket_path: &str, hello: SimMsgC2S) -> io::Result<Self> {
        let stream = UnixStream::connect(socket_path)?;
        // Blocking reads/writes by default — fine since we have a dedicated RX thread.

        // Send Hello
        {
            let mut w = &stream;
            send_msg(&mut w, &hello)?;
            w.flush()?;
        }

        // Read HelloAck
        let (assigned_start, sim_tick_ms) = {
            let r = &stream;
            let ack: SimMsgS2C = recv_msg(r)?;
            match ack {
                SimMsgS2C::HelloAck { assigned_start, sim_tick_ms } => (assigned_start, sim_tick_ms),
                other => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("expected HelloAck, got {:?}", other),
                    ));
                }
            }
        };

        let (enc_holo_tx, encoder_holo_rx) = flume::unbounded();
        let (enc_diff_tx, encoder_diff_rx) = flume::unbounded();
        let (gyro_tx, gyro_rx) = flume::unbounded();
        let (ld06_tx, ld06_rx) = flume::unbounded();
        let (can_tx, can_rx) = flume::unbounded();
        let (vlx_tx, vlx_rx) = flume::unbounded();
        let (batt_tx, battery_rx) = flume::unbounded();
        let (rome_tx, rome_rx) = flume::unbounded();
        let (tick_tx, tick_rx) = flume::unbounded();

        // Spawn RX thread on a clone of the stream.
        let rx_stream = stream.try_clone()?;
        thread::Builder::new()
            .name("sim-rx".into())
            .spawn(move || {
                rx_loop(
                    rx_stream,
                    RxSenders {
                        enc_holo: enc_holo_tx,
                        enc_diff: enc_diff_tx,
                        gyro: gyro_tx,
                        ld06: ld06_tx,
                        can: can_tx,
                        vlx: vlx_tx,
                        batt: batt_tx,
                        rome: rome_tx,
                        tick: tick_tx,
                    },
                );
            })
            .expect("spawn sim-rx");

        Ok(Self {
            tx: Mutex::new(stream),
            assigned_start,
            sim_tick_ms,
            encoder_holo_rx,
            encoder_diff_rx,
            gyro_rx,
            ld06_rx,
            can_rx,
            vlx_rx,
            battery_rx,
            rome_rx,
            tick_rx,
        })
    }

    /// Send a single message to the simulator.
    pub fn send(&self, msg: SimMsgC2S) -> io::Result<()> {
        let mut guard = self.tx.lock().unwrap();
        send_msg(&mut *guard, &msg)
    }

    /// Send motor consigns for a holonomic robot and wait (with timeout)
    /// for both the encoder delta and the gyro delta responses.
    pub fn tick_holo(&self, consigns: [f32; 3]) -> ([f32; 3], f32) {
        self.send(SimMsgC2S::MotorConsignsHolo { values: consigns })
            .unwrap_or_else(|e| log::error!("sim send error: {e}"));
        let timeout = Duration::from_millis(500);
        let enc = self
            .encoder_holo_rx
            .recv_timeout(timeout)
            .unwrap_or_else(|e| {
                log::error!("sim encoder delta timeout: {e}");
                [0.0, 0.0, 0.0]
            });
        let gyro = self
            .gyro_rx
            .recv_timeout(timeout)
            .unwrap_or_else(|e| {
                log::error!("sim gyro delta timeout: {e}");
                0.0
            });
        (enc, gyro)
    }

    /// Same for differential robots.
    pub fn tick_diff(&self, consigns: [f32; 2]) -> [f32; 2] {
        self.send(SimMsgC2S::MotorConsignsDiff { values: consigns })
            .unwrap_or_else(|e| log::error!("sim send error: {e}"));
        self.encoder_diff_rx
            .recv_timeout(Duration::from_millis(500))
            .unwrap_or_else(|e| {
                log::error!("sim encoder delta timeout: {e}");
                [0.0, 0.0]
            })
    }
}

struct RxSenders {
    enc_holo: Sender<[f32; 3]>,
    enc_diff: Sender<[f32; 2]>,
    gyro: Sender<f32>,
    ld06: Sender<Vec<u8>>,
    can: Sender<(u16, [u8; 8], u8)>,
    vlx: Sender<u16>,
    batt: Sender<u16>,
    rome: Sender<Vec<u8>>,
    tick: Sender<(u64, u64)>,
}

fn rx_loop(stream: UnixStream, s: RxSenders) {
    let mut r = stream;
    loop {
        match recv_msg::<_, SimMsgS2C>(&mut r) {
            Ok(msg) => match msg {
                SimMsgS2C::EncoderDeltaHolo { delta } => {
                    let _ = s.enc_holo.send(delta);
                }
                SimMsgS2C::EncoderDeltaDiff { delta } => {
                    let _ = s.enc_diff.send(delta);
                }
                SimMsgS2C::GyroDelta { d_theta_rad } => {
                    let _ = s.gyro.send(d_theta_rad);
                }
                SimMsgS2C::Ld06Bytes { bytes } => {
                    let _ = s.ld06.send(bytes);
                }
                SimMsgS2C::CanFrame { id, data, len } => {
                    let _ = s.can.send((id, data, len));
                }
                SimMsgS2C::VlxDistance { mm } => {
                    let _ = s.vlx.send(mm);
                }
                SimMsgS2C::BatteryMv { mv } => {
                    let _ = s.batt.send(mv);
                }
                SimMsgS2C::RomeBytes { bytes } => {
                    let _ = s.rome.send(bytes);
                }
                SimMsgS2C::Tick { tick, sim_time_ms } => {
                    let _ = s.tick.send((tick, sim_time_ms));
                }
                SimMsgS2C::HelloAck { .. } => {
                    log::warn!("unexpected HelloAck after handshake");
                }
                SimMsgS2C::Ext { tag, .. } => {
                    log::debug!("sim Ext {tag} ignored");
                }
                SimMsgS2C::Shutdown { reason } => {
                    log::warn!("sim requested shutdown: {reason}");
                    std::process::exit(0);
                }
            },
            Err(e) => {
                // Sim closed the socket (crash, kill-all, etc.). There is
                // no point keeping the robot process alive — every mock
                // driver will just spam errors on a dead channel.
                log::error!("sim connection lost ({e}); exiting");
                std::process::exit(0);
            }
        }
    }
}
