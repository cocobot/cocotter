use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cancaner::{ArmFlags, ArmTarget, CanMessage, Stage2Target, ARMS_PER_MODULE, STAGE2_SERVOS_PER_MODULE};

use crate::can::CanInterface;

#[derive(Clone, Default, Debug)]
pub struct ArmState {
    pub position: u16,
    pub color: u8,
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
pub struct Stage2State {
    pub position: u16,
    pub error: u8,
    pub flags: ArmFlags,
}

#[derive(Clone, Default, Debug)]
pub struct ColorSensorRawState {
    pub clear: u16,
    pub red: u16,
    pub green: u16,
    pub blue: u16,
}

#[derive(Clone, Default, Debug)]
pub struct MecaState {
    pub arms: [[ArmState; ARMS_PER_MODULE]; 3],
    pub translations: [TranslationState; 3],
    pub stage2: [[Stage2State; STAGE2_SERVOS_PER_MODULE]; 3],
    pub color_raw: [[ColorSensorRawState; ARMS_PER_MODULE]; 3],
}

/// Low-level proxy: read CAN-updated state + send direct commands
#[derive(Clone)]
pub struct MecaProxy {
    pub(super) can: CanInterface,
    pub(super) state: Arc<Mutex<MecaState>>,
    last_ping: Arc<AtomicU8>,
}

impl MecaProxy {
    pub(super) fn new(can: &CanInterface) -> Self {
        let state = Arc::new(Mutex::new(MecaState::default()));
        let last_ping = Arc::new(AtomicU8::new(0));
        let s = state.clone();
        let lp = last_ping.clone();

        can.add_callback(move |msg| match msg {
            CanMessage::Ping { value } => {
                lp.store(*value, Ordering::Relaxed);
            }
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
            CanMessage::Stage2Status {
                target,
                position,
                error,
                flags,
            } => {
                let mut st = s.lock().unwrap();
                if let Some(servo) = st
                    .stage2
                    .get_mut(target.module as usize)
                    .and_then(|m| m.get_mut(target.servo as usize))
                {
                    servo.position = *position;
                    servo.error = *error;
                    servo.flags = *flags;
                }
            }
            CanMessage::ColorSensorRaw {
                target,
                clear,
                red,
                green,
                blue,
            } => {
                let mut st = s.lock().unwrap();
                if let Some(raw) = st
                    .color_raw
                    .get_mut(target.module as usize)
                    .and_then(|m| m.get_mut(target.arm as usize))
                {
                    raw.clear = *clear;
                    raw.red = *red;
                    raw.green = *green;
                    raw.blue = *blue;
                }
            }
            _ => {}
        });

        Self {
            can: can.clone(),
            state,
            last_ping,
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

    pub fn stage2_state(&self, module: u8, servo: u8) -> Option<Stage2State> {
        self.state
            .lock()
            .unwrap()
            .stage2
            .get(module as usize)
            .and_then(|m| m.get(servo as usize))
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
        self.can.send(&CanMessage::SetTranslation { module, position, time_ms });
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

    pub fn set_color_config(
        &self,
        module: u8,
        arm: u8,
        color_id: u8,
        channel: u8,
        min: u16,
        max: u16,
    ) {
        self.can.send(&CanMessage::SetColorConfig {
            target: ArmTarget::new(module, arm),
            color_id,
            channel,
            min,
            max,
        });
    }

    pub fn set_color_sensor_config(&self, module: u8, arm: u8, integration_time: u8, gain: u8) {
        self.can.send(&CanMessage::SetColorSensorConfig {
            target: ArmTarget::new(module, arm),
            integration_time,
            gain,
        });
    }

    pub fn request_color_sensor_raw(&self, module: u8, arm: u8) {
        self.can.send(&CanMessage::RequestColorSensorRaw {
            target: ArmTarget::new(module, arm),
        });
    }

    pub fn set_color_led_pwm(&self, duty: u8) {
        self.can.send(&CanMessage::SetColorLedPwm { duty });
    }

    pub fn set_stage2(&self, module: u8, servo: u8, position: u16, time_ms: u16) {
        self.can.send(&CanMessage::SetStage2 {
            target: Stage2Target::new(module, servo),
            position,
            time_ms,
        });
    }

    pub fn set_stage2_torque(&self, module: u8, servo: u8, enable: bool) {
        self.can.send(&CanMessage::SetStage2Torque {
            target: Stage2Target::new(module, servo),
            enable,
        });
    }

    /// Send a ping and wait for the response (value + 1), with timeout.
    /// Returns true if pong received, false on timeout.
    pub fn ping(&self, timeout: Duration) -> bool {    
        static COUNTER: AtomicU8 = AtomicU8::new(42);
        let value = COUNTER.fetch_add(1, Ordering::Relaxed);
        let expected = value.wrapping_add(1);
        self.last_ping.store(0, Ordering::Relaxed);
        self.can.send(&CanMessage::Ping { value });

        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.last_ping.load(Ordering::Relaxed) == expected {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        false
    }
}
