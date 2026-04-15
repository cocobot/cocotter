use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};
use cancaner::{ARMS_PER_MODULE, ArmFlags, ArmTarget, CanMessage, LogDecoder, STAGE2_SERVOS_PER_MODULE, Stage2Target};

use crate::can::GalipeurCan;
use board_sabotter::SabotterBoard;


#[derive(Clone, Default, Debug)]
pub struct ArmState {
    pub position: u16,
    pub hue: u16,
    pub color_detected: bool,
    pub pump: bool,
    pub valve: bool,
    pub error: u8,
    pub flags: ArmFlags,
    pub pump_current: u8,
    /// Incremented on each status update from CAN
    pub update_seq: u64,
}

#[derive(Clone, Default, Debug)]
pub struct TranslationState {
    pub position: u16,
    pub error: u8,
    pub flags: ArmFlags,
    pub update_seq: u64,
}

#[derive(Clone, Default, Debug)]
pub struct Stage2State {
    pub position: u16,
    pub error: u8,
    pub flags: ArmFlags,
    pub update_seq: u64,
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
    pub battery_voltage_mv: Option<u16>,
    pub battery_modules_mask: u8,
}

/// Low-level proxy: read CAN-updated state + send direct commands
#[derive(Clone)]
pub struct MecaProxy<B: SabotterBoard> {
    can: GalipeurCan<B>,
    state: Arc<Mutex<MecaState>>,
    state_changed: Arc<Condvar>,
    last_ping: Arc<AtomicU8>,
}

impl<B: SabotterBoard> MecaProxy<B> {
    pub fn new(can: GalipeurCan<B>) -> Self {
        let state = Arc::new(Mutex::new(MecaState::default()));
        let state_changed = Arc::new(Condvar::new());
        let last_ping = Arc::new(AtomicU8::new(0));
        let s = state.clone();
        let sc = state_changed.clone();
        let lp = last_ping.clone();

        let mut log_decoder = LogDecoder::new();
        can.add_callback(move |msg| {
            if log_decoder.process_and_log("picotter", msg) {
                return;
            }
            match msg {
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
                        arm.hue = (color & 0x7F) as u16 * 360 / 128;
                        arm.color_detected = color & 0x80 != 0;
                        arm.pump = *pump;
                        arm.valve = *valve;
                        arm.error = *error;
                        arm.flags = *flags;
                        arm.pump_current = *pump_current;
                        arm.update_seq += 1;
                    }
                    drop(st);
                    sc.notify_all();
                }
                CanMessage::TranslationStatus {
                    module,
                    position,
                    error,
                    flags,
                } => {
                    let mut st = s.lock().unwrap();
                    if let Some(t) = st.translations.get_mut(*module as usize) {
                        t.position = *position;
                        t.error = *error;
                        t.flags = *flags;
                        t.update_seq += 1;
                    }
                    drop(st);
                    sc.notify_all();
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
                        servo.update_seq += 1;
                    }
                    drop(st);
                    sc.notify_all();
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
                CanMessage::BatteryStatus {
                    voltage_mv,
                    modules_mask,
                } => {
                    let mut st = s.lock().unwrap();
                    st.battery_voltage_mv = Some(*voltage_mv);
                    st.battery_modules_mask = *modules_mask;
                }
                _ => {}
            }
        });

        Self {
            can,
            state,
            state_changed,
            last_ping,
        }
    }

    pub fn send_message(&self, msg: &CanMessage) {
        self.can.send(msg);
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

    pub fn battery_voltage_mv(&self) -> Option<u16> {
        self.state.lock().unwrap().battery_voltage_mv
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
        self.send_message(&CanMessage::SetArm {
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

    pub fn set_color_threshold(&self, threshold: u16) {
        self.can.send(&CanMessage::SetColorThreshold { threshold });
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

    pub fn set_servo_id(&self, bus: cancaner::ServoBus, origin_id: u8, new_id: u8) {
        self.can.send(&CanMessage::SetServoId { bus, origin_id, new_id });
    }

    pub fn scan_bus(&self, bus: cancaner::ServoBus) {
        self.can.send(&CanMessage::ScanBus { bus });
    }

    /// Wait until a fresh status update (seq > seq_before) shows not moving, with timeout.
    /// `seq_before` must be captured before sending the command.
    fn wait_not_moving(
        &self,
        timeout: Duration,
        seq_before: u64,
        check: impl Fn(&MecaState) -> Option<(u64, bool)>, // returns (update_seq, moving)
    ) -> bool {
        let start = Instant::now();
        let mut st = self.state.lock().unwrap();
        loop {
            if let Some((seq, moving)) = check(&st) {
                if seq > seq_before && !moving {
                    return true;
                }
            }
            let remaining = timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                return false;
            }
            let (guard, _) = self.state_changed.wait_timeout(st, remaining).unwrap();
            st = guard;
        }
    }

    pub fn wait_arm(&self, module: u8, arm: u8, seq_before: u64, timeout: Duration) -> bool {
        self.wait_not_moving(timeout, seq_before, |st| {
            st.arms.get(module as usize)
                .and_then(|m| m.get(arm as usize))
                .map(|a| (a.update_seq, a.flags.moving))
        })
    }

    pub fn wait_arms(&self, module: u8, seq_before: u64, timeout: Duration) -> bool {
        self.wait_not_moving(timeout, seq_before, |st| {
            st.arms.get(module as usize).map(|arms| {
                // Skip arm 0 (buggy)
                let min_seq = arms.iter().map(|a| a.update_seq).min().unwrap_or(0);
                let any_moving = arms.iter().any(|a| a.flags.moving);
                (min_seq, any_moving)
            })
        })
    }

    pub fn wait_translation(&self, module: u8, seq_before: u64, timeout: Duration) -> bool {
        self.wait_not_moving(timeout, seq_before, |st| {
            st.translations.get(module as usize)
                .map(|t| (t.update_seq, t.flags.moving))
        })
    }

    pub fn wait_stage2(&self, module: u8, servo: u8, seq_before: u64, timeout: Duration) -> bool {
        self.wait_not_moving(timeout, seq_before, |st| {
            st.stage2.get(module as usize)
                .and_then(|m| m.get(servo as usize))
                .map(|s| (s.update_seq, s.flags.moving))
        })
    }

    /// Get current update_seq for an arm
    pub fn arm_seq(&self, module: u8, arm: u8) -> u64 {
        self.state.lock().unwrap().arms
            .get(module as usize)
            .and_then(|m| m.get(arm as usize))
            .map_or(0, |a| a.update_seq)
    }

    /// Get min update_seq across arms 1-3 of a module (skip arm 0)
    pub fn arms_seq(&self, module: u8) -> u64 {
        self.state.lock().unwrap().arms
            .get(module as usize)
            .map_or(0, |arms| arms[1..].iter().map(|a| a.update_seq).min().unwrap_or(0))
    }

    /// Get current update_seq for a translation
    pub fn translation_seq(&self, module: u8) -> u64 {
        self.state.lock().unwrap().translations
            .get(module as usize)
            .map_or(0, |t| t.update_seq)
    }

    /// Get current update_seq for a stage2 servo
    pub fn stage2_seq(&self, module: u8, servo: u8) -> u64 {
        self.state.lock().unwrap().stage2
            .get(module as usize)
            .and_then(|m| m.get(servo as usize))
            .map_or(0, |s| s.update_seq)
    }

    /// Get min update_seq across both stage2 servos of a module
    pub fn stage2s_seq(&self, module: u8) -> u64 {
        self.state.lock().unwrap().stage2
            .get(module as usize)
            .map_or(0, |servos| servos.iter().map(|s| s.update_seq).min().unwrap_or(0))
    }

    /// Wait for both stage2 servos of a module to stop moving
    pub fn wait_stage2s(&self, module: u8, seq_before: u64, timeout: Duration) -> bool {
        self.wait_not_moving(timeout, seq_before, |st| {
            st.stage2.get(module as usize).map(|servos| {
                let min_seq = servos.iter().map(|s| s.update_seq).min().unwrap_or(0);
                let any_moving = servos.iter().any(|s| s.flags.moving);
                (min_seq, any_moving)
            })
        })
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
