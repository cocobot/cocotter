use std::sync::{Arc, Mutex};

use cancaner::{ArmFlags, ArmTarget, CanMessage, Color, ARMS_PER_MODULE};

use crate::can::CanInterface;

#[derive(Clone, Default, Debug)]
pub struct ArmState {
    pub position: u16,
    pub color: Color,
    pub pump: bool,
    pub valve: bool,
    pub error: u8,
    pub flags: ArmFlags,
    pub pump_current: u8,
}

#[derive(Clone, Default, Debug)]
pub struct TranslationState {
    pub position: u16,
    pub error: u8,
}

#[derive(Clone, Default, Debug)]
pub struct MecaState {
    pub arms: [[ArmState; ARMS_PER_MODULE]; 3],
    pub translations: [TranslationState; 3],
}

/// Low-level proxy: read CAN-updated state + send direct commands
#[derive(Clone)]
pub struct MecaProxy {
    pub(super) can: CanInterface,
    pub(super) state: Arc<Mutex<MecaState>>,
}

impl MecaProxy {
    pub(super) fn new(can: &CanInterface) -> Self {
        let state = Arc::new(Mutex::new(MecaState::default()));
        let s = state.clone();

        can.add_callback(move |msg| match msg {
            CanMessage::ArmStatus {
                target,
                position,
                color,
                pump,
                valve,
                error,
                flags,
                pump_current,
            } => {
                let mut st = s.lock().unwrap();
                if let Some(arm) = st
                    .arms
                    .get_mut(target.module as usize)
                    .and_then(|m| m.get_mut(target.arm as usize))
                {
                    arm.position = *position;
                    arm.color = *color;
                    arm.pump = *pump;
                    arm.valve = *valve;
                    arm.error = *error;
                    arm.flags = *flags;
                    arm.pump_current = *pump_current;
                }
            }
            CanMessage::TranslationStatus {
                module,
                position,
                error,
            } => {
                let mut st = s.lock().unwrap();
                if let Some(t) = st.translations.get_mut(*module as usize) {
                    t.position = *position;
                    t.error = *error;
                }
            }
            _ => {}
        });

        Self {
            can: can.clone(),
            state,
        }
    }

    // --- State reads (updated by CAN callbacks) ---

    pub fn get_state(&self) -> MecaState {
        self.state.lock().unwrap().clone()
    }

    pub fn arm_state(&self, module: u8, arm: u8) -> Option<ArmState> {
        self.state
            .lock()
            .unwrap()
            .arms
            .get(module as usize)
            .and_then(|m| m.get(arm as usize))
            .cloned()
    }

    pub fn translation_state(&self, module: u8) -> Option<TranslationState> {
        self.state
            .lock()
            .unwrap()
            .translations
            .get(module as usize)
            .cloned()
    }

    // --- Direct CAN commands ---

    pub fn set_arm(
        &self,
        module: u8,
        arm: u8,
        position: u16,
        time_ms: u16,
        pump: bool,
        valve: bool,
    ) {
        self.can.send(&CanMessage::SetArm {
            target: ArmTarget::new(module, arm),
            position,
            time_ms,
            pump,
            valve,
        });
    }

    pub fn set_translation(&self, module: u8, position: u16, time_ms: u16) {
        self.can
            .send(&CanMessage::SetTranslation { module, position, time_ms });
    }

    pub fn set_pump(&self, module: u8, arm: u8, enable: bool) {
        self.can.send(&CanMessage::SetPump {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }

    pub fn set_valve(&self, module: u8, arm: u8, enable: bool) {
        self.can.send(&CanMessage::SetValve {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }

    pub fn set_torque(&self, module: u8, arm: u8, enable: bool) {
        self.can.send(&CanMessage::SetTorque {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }
}
