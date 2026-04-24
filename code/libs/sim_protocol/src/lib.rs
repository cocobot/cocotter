//! Wire protocol for simulator ↔ robot mock IPC.
//!
//! Length-prefixed bincode frames over a Unix socket. Each direction uses
//! its own enum (`SimMsgC2S` from client to simulator, `SimMsgS2C` from
//! simulator to client). The enums are **append-only** — never reorder
//! existing variants, only add new ones at the end.

use bitflags::bitflags;
use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: u16 = 1;
pub const DEFAULT_SOCKET_PATH: &str = "/tmp/meca_sim.sock";

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum RobotKind {
    Galipeur,
    Pami,
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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum SimMsgC2S {
    Hello {
        version: u16,
        robot_id: String,
        kind: RobotKind,
        caps: Capabilities,
        requested_start: Option<Pose2D>,
    },
    MotorConsignsHolo { values: [f32; 3] },
    MotorConsignsDiff { values: [f32; 2] },
    MotorsBreak { enable: bool },
    CanFrame { id: u16, data: [u8; 8], len: u8 },
    RomeBytes { bytes: Vec<u8> },
    Ext { tag: String, payload: Vec<u8> },
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
