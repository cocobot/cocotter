use std::sync::{Arc, Mutex};
use embedded_can::Frame;
use board_sabotter::SabotterBoard;
use cancaner::{ArmFlags, ArmTarget, CanInterface, CanMessage, Color, LogDecoder, ARMS_PER_MODULE};


const MAX_TX_TRIES: u32 = 3;

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
pub struct MecaProxy<B: SabotterBoard> {
    can: B::Can,
    state: Arc<Mutex<MecaState>>,
}

impl<B: SabotterBoard> MecaProxy<B> {
    pub(super) fn new(can: B::Can) -> Self {
        let state = Arc::new(Mutex::new(MecaState::default()));
        Self { can, state }
    }

    /// Start RX thread
    pub fn init_rx(&mut self) {
        let state_clone = self.state.clone();
        let can_clone = self.can.clone();
        std::thread::Builder::new()
            .name("can-rx".into())
            .stack_size(4096)
            .spawn(move || loop {
                let mut log_decoder = LogDecoder::new();
                match can_clone.can_receive() {
                    Ok(frame) => {
                        if let Some(message) = CanMessage::from_frame(&frame) {
                            if !log_decoder.process_and_log("picotter", &message) {
                                Self::on_other_message(&state_clone, &message);
                            }
                        } else {
                            log::warn!("CAN: unknown or invalid frame: id={:?} len={}", frame.id(), frame.data().len());
                        }
                    }
                    Err(e) => {
                        log::warn!("CAN: receive error: {:?}", e);
                    }
                }
            })
            .expect("spawn can-rx");
    }

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

    pub fn set_arm(
        &self,
        module: u8,
        arm: u8,
        position: u16,
        time_ms: u16,
        pump: bool,
        valve: bool,
    ) {
        self.send_message(&CanMessage::SetArm {
            target: ArmTarget::new(module, arm),
            position,
            time_ms,
            pump,
            valve,
        });
    }

    pub fn set_translation(&self, module: u8, position: u16, time_ms: u16) {
        self.send_message(&CanMessage::SetTranslation { module, position, time_ms });
    }

    pub fn set_pump(&self, module: u8, arm: u8, enable: bool) {
        self.send_message(&CanMessage::SetPump {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }

    pub fn set_valve(&self, module: u8, arm: u8, enable: bool) {
        self.send_message(&CanMessage::SetValve {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }

    pub fn set_torque(&self, module: u8, arm: u8, enable: bool) {
        self.send_message(&CanMessage::SetTorque {
            target: ArmTarget::new(module, arm),
            enable,
        });
    }


    /// Send a message, retry if needed
    fn send_message(&self, msg: &CanMessage) {
        let frame = msg.to_frame();
        for attempt in 0..MAX_TX_TRIES {
            match self.can.can_transmit(&frame) {
                Ok(_) => return,
                Err(e) => log::warn!("CAN: transmit failed ({}/{}): {:?}", attempt + 1, MAX_TX_TRIES, e),
            }
        }
        log::error!("CAN: TX failed after {MAX_TX_TRIES} retries");
    }

    fn on_other_message(state: &Arc<Mutex<MecaState>>, msg: &CanMessage) {
        match msg {
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
                let mut st = state.lock().unwrap();
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
                let mut st = state.lock().unwrap();
                if let Some(t) = st.translations.get_mut(*module as usize) {
                    t.position = *position;
                    t.error = *error;
                }
            }
            _ => {}
        }
    }
}
