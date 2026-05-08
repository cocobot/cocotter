//! Wire protocol for simulator ↔ robot mock IPC.
//!
//! Length-prefixed bincode frames over a Unix socket. Each direction uses
//! its own enum (`SimMsgC2S` from client to simulator, `SimMsgS2C` from
//! simulator to client). The enums are **append-only** — never reorder
//! existing variants, only add new ones at the end.

use bitflags::bitflags;
use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u16 = 1;
pub const DEFAULT_SOCKET_PATH: &str = "/tmp/cocotter_sim.sock";

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RobotKind {
    Galipeur,
    Pami,
    /// Sim-only opponent robot. Never connects via IPC — the sim drives
    /// it directly from keyboard / gamepad input so the operator can
    /// stress-test the real robots' avoidance behaviour.
    Adversary,
}

/// Which side of the table the robot starts on. Independent of the
/// year-specific team colour mapping (kept in `config.teams` on the
/// sim side); the colour-input pin is set from this.
///
/// Lowercased rename for TOML / CLI ("left", "right"); bincode uses
/// the variant index so the rename doesn't affect the wire format.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    Left,
    Right,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2D {
    pub x_mm: f32,
    pub y_mm: f32,
    pub theta_rad: f32,
}

bitflags! {
    #[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct Capabilities: u32 {
        const CAN   = 1 << 0;
        const LD06  = 1 << 1;
        const GYRO  = 1 << 2;
        const VLX   = 1 << 3;
        const ROME  = 1 << 4;
        const HOLO3 = 1 << 5;
        const DIFF2 = 1 << 6;
    }
}

/// Reference frame for debug volume coordinates.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum DebugVolumeFrame {
    /// Body-relative: parented to the robot entity, follows its pose.
    Body,
    /// Table-absolute: fixed in the world, does not follow the robot.
    Table,
}

/// Translucent debug volume the robot asks the sim to render. Pure
/// visualisation — the sim treats them as overlays toggled by the
/// operator (V key), no physics, no raycast contribution.
///
/// Coordinates are in mm. The `frame` field selects the reference:
/// - `Body`: `+X` forward, `+Y` left, `+Z` up, parented to the robot.
/// - `Table`: world-absolute, `+X`/`+Y` match the table axes.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum DebugVolume {
    /// Oriented box, centre + half-extents + yaw around Z axis.
    Box {
        center_mm: [f32; 3],
        half_size_mm: [f32; 3],
        yaw_rad: f32,
        rgba: [f32; 4],
        frame: DebugVolumeFrame,
    },
    /// Vertical cylinder, base centre + radius + height.
    Cylinder {
        center_mm: [f32; 3],
        radius_mm: f32,
        height_mm: f32,
        rgba: [f32; 4],
        frame: DebugVolumeFrame,
    },
    /// 3D text label at a given position.
    Text {
        position_mm: [f32; 3],
        text: String,
        size_mm: f32,
        rgba: [f32; 4],
        frame: DebugVolumeFrame,
    },
}

impl DebugVolume {
    pub fn frame(&self) -> DebugVolumeFrame {
        match self {
            Self::Box { frame, .. }
            | Self::Cylinder { frame, .. }
            | Self::Text { frame, .. } => *frame,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SimMsgC2S {
    Hello {
        version: u16,
        robot_id: String,
        kind: RobotKind,
        caps: Capabilities,
        requested_start: Option<Pose2D>,
        /// Which side of the table the robot is on. Read from the
        /// `SIDE` env var the launcher (sim chord / CLI) sets. The
        /// sim picks the matching `start_poses.<kind>.<side>` and
        /// echoes the value back via `SimMsgS2C::Side` so the mock
        /// colour pin reflects it.
        requested_side: Option<Side>,
    },
    MotorConsignsHolo { values: [f32; 3] },
    MotorConsignsDiff { values: [f32; 2] },
    MotorsBreak { enable: bool },
    CanFrame { id: u16, data: [u8; 8], len: u8 },
    RomeBytes { bytes: Vec<u8> },
    Ext { tag: String, payload: Vec<u8> },
    /// Full neopixel strip frame in wire order. The sim maps strip
    /// offsets onto the fixtures declared in `[[robot.neopixels]]`.
    /// Component order is the one seen by the host code (RGB after
    /// `smart_leds::RGB8`), not the on-the-wire GRB order.
    NeopixelFrame { pixels: Vec<[u8; 3]> },
    /// Warp the robot's sim pose to the given value. Only meaningful in
    /// the simulator — physical hardware logs a warning and no-ops.
    Teleport { pose: Pose2D },
    /// Push a fresh set of translucent debug volumes for the sim to
    /// render under this robot. Each call replaces the previous list
    /// in its entirety (last-writer-wins). Use an empty `volumes`
    /// vector to clear them.
    DebugVolumes { volumes: Vec<DebugVolume> },
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SimMsgS2C {
    HelloAck {
        assigned_start: Pose2D,
        sim_tick_ms: u16,
    },
    Tick { tick: u64, sim_time_ms: u64 },
    EncoderDeltaHolo { delta: [f32; 3] },
    EncoderDeltaDiff { delta: [f32; 2] },
    GyroDelta { d_theta_rad: f32 },
    Ld06Bytes { bytes: Vec<u8> },
    CanFrame { id: u16, data: [u8; 8], len: u8 },
    VlxDistance { mm: u16 },
    BatteryMv { mv: u16 },
    RomeBytes { bytes: Vec<u8> },
    Ext { tag: String, payload: Vec<u8> },
    /// Sim-requested shutdown. The robot process should log the reason
    /// and exit — don't try to reconnect, don't continue running with a
    /// dead IPC channel.
    Shutdown { reason: String },
    /// State of the simulated starter cable. The robot's mock starter
    /// pin reflects this:
    /// - `inserted: true`  → `is_low()` returns `true` (cable in)
    /// - `inserted: false` → `is_high()` returns `true` (cable out)
    /// The T key in the sim toggles this through Pregame → Inserted →
    /// Running, and the strat reads the pin transitions to advance.
    Starter { inserted: bool },
    /// Side assignment, sent immediately after `HelloAck`. The robot's
    /// mock colour pin reflects this (`is_high` for Left, `is_low` for
    /// Right) so the strat can pick its team without reading any env
    /// var directly.
    Side { side: Side },
}

/// Length-prefixed framing: `u32 LE length` + bincode payload.
pub mod frame {
    use std::io::{self, Read, Write};

    pub fn write_frame<W: Write>(mut w: W, payload: &[u8]) -> io::Result<()> {
        let len = payload.len() as u32;
        w.write_all(&len.to_le_bytes())?;
        w.write_all(payload)?;
        w.flush()
    }

    pub fn read_frame<R: Read>(mut r: R) -> io::Result<Vec<u8>> {
        let mut len_buf = [0u8; 4];
        r.read_exact(&mut len_buf)?;
        let len = u32::from_le_bytes(len_buf) as usize;
        let mut buf = vec![0u8; len];
        r.read_exact(&mut buf)?;
        Ok(buf)
    }
}

/// Convenience: encode a message into a length-prefixed frame on `w`.
pub fn send_msg<W: std::io::Write, M: Serialize>(w: W, msg: &M) -> std::io::Result<()> {
    let payload = bincode::serialize(msg)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    frame::write_frame(w, &payload)
}

/// Convenience: read a length-prefixed frame from `r` and decode into `M`.
pub fn recv_msg<R: std::io::Read, M: for<'de> Deserialize<'de>>(r: R) -> std::io::Result<M> {
    let payload = frame::read_frame(r)?;
    bincode::deserialize(&payload)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_hello() {
        let msg = SimMsgC2S::Hello {
            version: PROTOCOL_VERSION,
            robot_id: "galipeur-test".into(),
            kind: RobotKind::Galipeur,
            caps: Capabilities::CAN | Capabilities::LD06 | Capabilities::GYRO | Capabilities::HOLO3,
            requested_start: Some(Pose2D { x_mm: 200.0, y_mm: 200.0, theta_rad: 0.0 }),
            requested_side: Some(Side::Left),
        };
        let bytes = bincode::serialize(&msg).unwrap();
        let decoded: SimMsgC2S = bincode::deserialize(&bytes).unwrap();
        match decoded {
            SimMsgC2S::Hello { version, kind, .. } => {
                assert_eq!(version, PROTOCOL_VERSION);
                assert_eq!(kind, RobotKind::Galipeur);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn roundtrip_s2c_variants() {
        for msg in [
            SimMsgS2C::HelloAck { assigned_start: Pose2D::default(), sim_tick_ms: 10 },
            SimMsgS2C::EncoderDeltaHolo { delta: [1.0, 2.0, 3.0] },
            SimMsgS2C::GyroDelta { d_theta_rad: 0.01 },
            SimMsgS2C::CanFrame { id: 0x123, data: [0; 8], len: 4 },
        ] {
            let bytes = bincode::serialize(&msg).unwrap();
            let _decoded: SimMsgS2C = bincode::deserialize(&bytes).unwrap();
        }
    }
}
