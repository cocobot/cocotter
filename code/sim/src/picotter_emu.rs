//! Picotter emulator: stands in for the galipeur peripheral MCU.
//!
//! Receives CAN frames from galipeur, maintains arm/clamp/translation state
//! and replies with *Status frames. Also emits a periodic battery status.
//! No time-delayed responses for now (M4 first pass): each `Set*` produces
//! its `*Status` synchronously with `position_reached=true`.

use cancaner::{ArmFlags, ArmTarget, CanMessage, ClampTarget, EncodedMessage};
use embedded_can::{Frame, Id, StandardId};
use sim_protocol::SimMsgS2C;

const BATTERY_MV: u16 = 16_500; // nominal 4S Li-ion

#[derive(Default)]
pub struct PicotterEmu {
    arms: [[ArmShadow; 4]; 3],
    clamps: [[ClampShadow; 3]; 3],
    translations: [TranslationShadow; 3],
}

#[derive(Default, Clone, Copy)]
struct ArmShadow {
    position: u16,
    torque_enabled: bool,
    pump: bool,
    valve: bool,
}

#[derive(Default, Clone, Copy)]
struct ClampShadow {
    position: u16,
    torque_enabled: bool,
}

#[derive(Default, Clone, Copy)]
struct TranslationShadow {
    position: u16,
}

impl PicotterEmu {
    /// Decode a CAN frame, mutate state, return any replies.
    pub fn on_can_frame(&mut self, id: u16, data: &[u8; 8], len: u8) -> Vec<SimMsgS2C> {
        let frame = SimFrame::new_std(id, &data[..len as usize]);
        let Some(msg) = CanMessage::from_frame(&frame) else {
            return vec![];
        };

        let mut out = Vec::new();
        match msg {
            CanMessage::SetArm { target, position, time_ms: _ } => {
                self.set_arm(target, |a| a.position = position);
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
            CanMessage::SetValve { target, enable } => {
                self.set_arm(target, |a| a.valve = enable);
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::RequestArmStatus { target } => {
                self.emit_arm_status_for(target, &mut out);
            }
            CanMessage::SetClamp { target, position, time_ms: _ } => {
                self.set_clamp(target, |c| c.position = position);
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::SetClampTorque { target, enable } => {
                self.set_clamp(target, |c| c.torque_enabled = enable);
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::RequestClampStatus { target } => {
                self.emit_clamp_status_for(target, &mut out);
            }
            CanMessage::SetTranslation { module, position, time_ms: _ } => {
                if module == 0xF {
                    for t in &mut self.translations {
                        t.position = position;
                    }
                    for m in 0..3 {
                        out.push(translation_status_msg(m, &self.translations[m as usize]));
                    }
                } else if (module as usize) < self.translations.len() {
                    self.translations[module as usize].position = position;
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

    pub fn battery_status_msg() -> SimMsgS2C {
        let encoded = CanMessage::BatteryStatus {
            voltage_mv: BATTERY_MV,
            modules_mask: 0b111,
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
    let flags = ArmFlags {
        torque_enabled: s.torque_enabled,
        moving: false,
        position_reached: true,
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
    let flags = ArmFlags {
        torque_enabled: c.torque_enabled,
        moving: false,
        position_reached: true,
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
    let flags = ArmFlags {
        torque_enabled: true,
        moving: false,
        position_reached: true,
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
