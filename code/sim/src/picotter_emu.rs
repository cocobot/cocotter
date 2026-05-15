//! Picotter emulator: stands in for the galipeur peripheral MCU.
//!
//! Receives CAN frames from galipeur, maintains arm/clamp/translation state
//! and replies with *Status frames. Also emits a periodic battery status.
//!
//! Each `Set*` schedules a movement: the shadow position interpolates
//! linearly from its current value toward the requested target over
//! the requested `time_ms` (or `DEFAULT_MOVE_MS` when the strat asks
//! for 0). The reply on `Set*` carries `position_reached=false`,
//! `moving=true`; the completion `Status` (with `position_reached=true`)
//! is fired by `tick()` once the movement reaches the target. The
//! snapshot exposed to the renderer reflects the interpolated
//! position so joints animate smoothly.

use std::time::{Duration, Instant};

use cancaner::{ArmFlags, ArmTarget, CanMessage, ClampTarget, EncodedMessage};
use embedded_can::{Frame, Id, StandardId};
use sim_protocol::SimMsgS2C;

use crate::bridge::ModuleActuators;

const BATTERY_MV: u16 = 16_500; // nominal 4S Li-ion

/// Fallback movement duration when the strat asks for `time_ms = 0`
/// (typical "as fast as possible" request).
const DEFAULT_MOVE_MS: u64 = 400;
/// Floor for any non-zero `time_ms` — the firmware needs at least a
/// few ticks to move, so the sim shouldn't pretend it's instant.
const MIN_MOVE_MS: u64 = 50;

#[derive(Clone, Copy, Debug)]
struct Movement {
    start_pos: u16,
    target_pos: u16,
    started_at: Instant,
    duration: Duration,
}

impl Movement {
    fn new(current: u16, target: u16, time_ms: u16) -> Self {
        let ms = if time_ms == 0 {
            DEFAULT_MOVE_MS
        } else {
            (time_ms as u64).max(MIN_MOVE_MS)
        };
        Self {
            start_pos: current,
            target_pos: target,
            started_at: Instant::now(),
            duration: Duration::from_millis(ms),
        }
    }

    /// `(new_position, finished)`. Linear lerp; `finished` true when
    /// the movement has reached the target (or overshot in time).
    fn sample(&self, now: Instant) -> (u16, bool) {
        let elapsed = now.saturating_duration_since(self.started_at);
        if elapsed >= self.duration {
            return (self.target_pos, true);
        }
        let t = elapsed.as_secs_f32() / self.duration.as_secs_f32();
        let s = self.start_pos as f32;
        let e = self.target_pos as f32;
        let lerped = s + (e - s) * t;
        (lerped.round().clamp(0.0, 65535.0) as u16, false)
    }
}

#[derive(Default)]
pub struct PicotterEmu {
    arms: [[ArmShadow; 4]; 3],
    clamps: [[ClampShadow; 3]; 3],
    translations: [TranslationShadow; 4],
}

#[derive(Default, Clone, Copy)]
struct ArmShadow {
    position: u16,
    movement: Option<Movement>,
    torque_enabled: bool,
    pump: bool,
    valve: bool,
}

#[derive(Default, Clone, Copy)]
struct ClampShadow {
    position: u16,
    movement: Option<Movement>,
    torque_enabled: bool,
}

#[derive(Default, Clone, Copy)]
struct TranslationShadow {
    position: u16,
    movement: Option<Movement>,
}

impl PicotterEmu {
    /// Decode a CAN frame, mutate state, return any replies.
    pub fn on_can_frame(&mut self, id: u16, data: &[u8; 8], len: u8) -> Vec<SimMsgS2C> {
        let frame = SimFrame::new_std(id, &data[..len as usize]);
        let Some(msg) = CanMessage::from_frame(&frame) else {
            log::debug!("[picotter] unknown CAN id=0x{:03X} data={:02X?}", id, &data[..len as usize]);
            return vec![];
        };
        log::debug!("[picotter] RX {:?}", msg);

        let mut out = Vec::new();
        match msg {
            CanMessage::SetArm { target, position, time_ms } => {
                self.set_arm(target, |a| {
                    a.movement = Some(Movement::new(a.position, position, time_ms));
                });
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::SetTorque { target, enable } => {
                self.set_arm(target, |a| a.torque_enabled = enable);
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::SetPump { target, enable } => {
                self.set_arm(target, |a| a.pump = enable);
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::SetValve { target, mode } => {
                self.set_arm(target, |a| a.valve = matches!(mode, cancaner::ValveMode::On | cancaner::ValveMode::Toggle { .. }));
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::RequestArmStatus { target } => {
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::SetClamp { target, position, time_ms } => {
                self.set_clamp(target, |c| {
                    c.movement = Some(Movement::new(c.position, position, time_ms));
                });
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::SetClampTorque { target, enable } => {
                self.set_clamp(target, |c| c.torque_enabled = enable);
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::RequestClampStatus { target } => {
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::SetTranslation { module, position, time_ms } => {
                let start_movement = |t: &mut TranslationShadow| {
                    t.movement = Some(Movement::new(t.position, position, time_ms));
                };
                if module == 0xF {
                    for t in &mut self.translations {
                        start_movement(t);
                    }
                    for m in 0..4 {
                        out.push(translation_status_msg(m, &self.translations[m as usize]));
                    }
                } else if (module as usize) < self.translations.len() {
                    start_movement(&mut self.translations[module as usize]);
                    out.push(translation_status_msg(module, &self.translations[module as usize]));
                }
            }
            CanMessage::RequestTranslationStatus { module } => {
                if (module as usize) < self.translations.len() {
                    out.push(translation_status_msg(module, &self.translations[module as usize]));
                }
            }
            // Lidar enable / ground threshold: state-only, no auto reply.
            CanMessage::SetLidarEnable { .. }
            | CanMessage::SetGroundThreshold { .. }
            | CanMessage::SetColorThreshold { .. }
            | CanMessage::SetColorLedPwm { .. }
            | CanMessage::Ping { .. } => {}
            _ => {}
        }
        out
    }

    /// Advance every active movement to `now`. Returns the
    /// `*Status` frames to send back to galipeur for actuators that
    /// just *finished* moving (so the strat sees `position_reached=true`).
    /// Intermediate positions are kept internal — they only feed the
    /// renderer through `snapshot()`.
    pub fn tick(&mut self, now: Instant) -> Vec<SimMsgS2C> {
        let mut out = Vec::new();
        for m in 0..3 {
            for a in 0..4 {
                let arm = &mut self.arms[m][a];
                if let Some(mv) = arm.movement {
                    let (pos, finished) = mv.sample(now);
                    arm.position = pos;
                    if finished {
                        arm.movement = None;
                        out.push(arm_status_msg(
                            ArmTarget::new(m as u8, a as u8),
                            arm,
                        ));
                    }
                }
            }
            for c in 0..3 {
                let clamp = &mut self.clamps[m][c];
                if let Some(mv) = clamp.movement {
                    let (pos, finished) = mv.sample(now);
                    clamp.position = pos;
                    if finished {
                        clamp.movement = None;
                        out.push(clamp_status_msg(
                            ClampTarget::from_raw(m as u8, c as u8),
                            clamp,
                        ));
                    }
                }
            }
            let t = &mut self.translations[m];
            if let Some(mv) = t.movement {
                let (pos, finished) = mv.sample(now);
                t.position = pos;
                if finished {
                    t.movement = None;
                    out.push(translation_status_msg(m as u8, t));
                }
            }
        }
        // Taquet (translation index 3, not part of arm modules)
        let t = &mut self.translations[3];
        if let Some(mv) = t.movement {
            let (pos, finished) = mv.sample(now);
            t.position = pos;
            if finished {
                t.movement = None;
                out.push(translation_status_msg(3, t));
            }
        }
        out
    }

    /// Snapshot the current shadow positions, indexed Left=0, Back=1,
    /// Right=2 (= `RobotSide::module()`). Used by the renderer to
    /// drive the joint hierarchy.
    pub fn snapshot(&self) -> [ModuleActuators; 3] {
        let mut out = [ModuleActuators::default(); 3];
        for m in 0..3 {
            out[m].translation_position = self.translations[m].position;
            for a in 0..4 {
                out[m].arm_positions[a] = self.arms[m][a].position;
            }
            for c in 0..3 {
                out[m].clamp_positions[c] = self.clamps[m][c].position;
            }
        }
        out
    }

    pub fn battery_status_msg() -> SimMsgS2C {
        let encoded = CanMessage::BatteryStatus {
            voltage_mv: BATTERY_MV,
            modules_mask: 0b111,
        }
        .encode();
        encoded_to_sim(&encoded)
    }

    /// Report all three ground sensors as detecting the table. Without
    /// this the galipeur's LED ring stays red (sensor 0/1/2 not seen)
    /// because its initial assumption is "not on ground".
    pub fn ground_status_msg() -> SimMsgS2C {
        let encoded = CanMessage::GroundStatus {
            detection_mask: 0b111,
        }
        .encode();
        encoded_to_sim(&encoded)
    }

    /// Encode a `LidarStatus` frame carrying two distance-mm
    /// measurements for a module plus a sequence counter.
    pub fn lidar_status_msg(module: u8, distance_0: u16, distance_1: u16, seq: u8) -> SimMsgS2C {
        let encoded = CanMessage::LidarStatus {
            module,
            distance_0,
            distance_1,
            seq,
        }
        .encode();
        encoded_to_sim(&encoded)
    }

    // ---- state mutation helpers ----

    fn set_arm<F: FnMut(&mut ArmShadow)>(&mut self, target: ArmTarget, mut f: F) {
        for m in 0..3 {
            for a in 0..4 {
                if target.matches(m, a) {
                    f(&mut self.arms[m as usize][a as usize]);
                }
            }
        }
    }

    fn emit_arm_status_for(&self, target: ArmTarget, out: &mut Vec<SimMsgS2C>) {
        for m in 0..3 {
            for a in 0..4 {
                if target.matches(m, a) {
                    let s = &self.arms[m as usize][a as usize];
                    out.push(arm_status_msg(ArmTarget::new(m, a), s));
                }
            }
        }
    }

    fn set_clamp<F: FnMut(&mut ClampShadow)>(&mut self, target: ClampTarget, mut f: F) {
        for m in 0..3 {
            for s in 0..3 {
                if target.matches(m, s) {
                    f(&mut self.clamps[m as usize][s as usize]);
                }
            }
        }
    }

    fn emit_clamp_status_for(&self, target: ClampTarget, out: &mut Vec<SimMsgS2C>) {
        for m in 0..3 {
            for s in 0..3 {
                if target.matches(m, s) {
                    let c = &self.clamps[m as usize][s as usize];
                    out.push(clamp_status_msg(ClampTarget::from_raw(m, s), c));
                }
            }
        }
    }
}

fn arm_status_msg(target: ArmTarget, s: &ArmShadow) -> SimMsgS2C {
    let moving = s.movement.is_some();
    let flags = ArmFlags {
        torque_enabled: s.torque_enabled,
        moving,
        position_reached: !moving,
    };
    let encoded = CanMessage::ArmStatus {
        target,
        position: s.position,
        color: 0,
        pump: s.pump,
        valve: s.valve,
        error: 0,
        flags,
        pump_current: 0,
    }
    .encode();
    encoded_to_sim(&encoded)
}

fn clamp_status_msg(target: ClampTarget, c: &ClampShadow) -> SimMsgS2C {
    let moving = c.movement.is_some();
    let flags = ArmFlags {
        torque_enabled: c.torque_enabled,
        moving,
        position_reached: !moving,
    };
    let encoded = CanMessage::ClampStatus {
        target,
        position: c.position,
        error: 0,
        flags,
    }
    .encode();
    encoded_to_sim(&encoded)
}

fn translation_status_msg(module: u8, t: &TranslationShadow) -> SimMsgS2C {
    let moving = t.movement.is_some();
    let flags = ArmFlags {
        torque_enabled: true,
        moving,
        position_reached: !moving,
    };
    let encoded = CanMessage::TranslationStatus {
        module,
        position: t.position,
        error: 0,
        flags,
    }
    .encode();
    encoded_to_sim(&encoded)
}

fn encoded_to_sim(e: &EncodedMessage) -> SimMsgS2C {
    let mut data = [0u8; 8];
    let len = e.len.min(8);
    data[..len].copy_from_slice(&e.data[..len]);
    SimMsgS2C::CanFrame {
        id: e.id.as_raw(),
        data,
        len: len as u8,
    }
}

/// Minimal `embedded_can::Frame` impl used only for decoding incoming CAN
/// payloads.
#[derive(Debug)]
struct SimFrame {
    id: Id,
    data: Vec<u8>,
}

impl SimFrame {
    fn new_std(id_raw: u16, data: &[u8]) -> Self {
        let sid = StandardId::new(id_raw).unwrap_or(StandardId::ZERO);
        Self { id: Id::Standard(sid), data: data.to_vec() }
    }
}

impl Frame for SimFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        Some(Self { id: id.into(), data: data.to_vec() })
    }
    fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> { None }
    fn is_extended(&self) -> bool { matches!(self.id, Id::Extended(_)) }
    fn is_remote_frame(&self) -> bool { false }
    fn id(&self) -> Id { self.id }
    fn dlc(&self) -> usize { self.data.len() }
    fn data(&self) -> &[u8] { &self.data }
}
