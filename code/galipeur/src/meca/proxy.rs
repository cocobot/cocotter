use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};
use cancaner::{ArmFlags, ArmTarget, CanMessage, ClampServo, ClampTarget, LogDecoder};

use crate::can::GalipeurCan;
use board_sabotter::SabotterBoard;

// ==================== Watcher ====================

struct WatcherInner<T> {
    value: T,
    seq: u64,
}

/// A per-element value with change notification.
///
/// `set()` replaces the value, increments an internal sequence number and
/// wakes any thread waiting in `wait_until()`. `wait_until()` blocks until
/// both (a) a set has happened since the captured `seq_before` and (b) the
/// predicate matches the current value.
pub struct Watcher<T: Clone> {
    inner: Arc<(Mutex<WatcherInner<T>>, Condvar)>,
}

impl<T: Clone> Clone for Watcher<T> {
    fn clone(&self) -> Self {
        Self { inner: self.inner.clone() }
    }
}

impl<T: Clone> Watcher<T> {
    pub fn new(initial: T) -> Self {
        Self {
            inner: Arc::new((
                Mutex::new(WatcherInner { value: initial, seq: 0 }),
                Condvar::new(),
            )),
        }
    }

    /// Clone the current value.
    pub fn get(&self) -> T {
        self.inner.0.lock().unwrap().value.clone()
    }

    /// Return the current sequence number without reading the value.
    pub fn seq(&self) -> u64 {
        self.inner.0.lock().unwrap().seq
    }

    /// Replace the value, bump the sequence and notify waiters.
    pub fn set(&self, value: T) {
        let (lock, cvar) = &*self.inner;
        let mut inner = lock.lock().unwrap();
        inner.value = value;
        inner.seq = inner.seq.wrapping_add(1);
        drop(inner);
        cvar.notify_all();
    }

    /// Block until a new value arrived after `seq_before` AND it matches `predicate`.
    /// Returns `true` if matched, `false` on timeout.
    pub fn wait_until<P>(&self, seq_before: u64, predicate: P, timeout: Duration) -> bool
    where
        P: Fn(&T) -> bool,
    {
        let (lock, cvar) = &*self.inner;
        let start = Instant::now();
        let mut inner = lock.lock().unwrap();
        loop {
            if inner.seq > seq_before && predicate(&inner.value) {
                return true;
            }
            let remaining = timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                return false;
            }
            let (guard, _) = cvar.wait_timeout(inner, remaining).unwrap();
            inner = guard;
        }
    }
}

// ==================== State types ====================

#[derive(Clone, Default, Debug)]
pub struct ArmStatus {
    pub position: u16,
    pub hue: u16,
    pub color_detected: bool,
    pub pump: bool,
    pub valve: bool,
    pub error: u8,
    pub flags: ArmFlags,
    pub pump_current: u8,
}

#[derive(Clone, Default, Debug)]
pub struct ClampStatus {
    pub position: u16,
    pub error: u8,
    pub flags: ArmFlags,
}

#[derive(Clone, Default, Debug)]
pub struct TranslationStatus {
    pub position: u16,
    pub error: u8,
    pub flags: ArmFlags,
}

#[derive(Clone, Default, Debug)]
pub struct ColorRawStatus {
    pub clear: u16,
    pub red: u16,
    pub green: u16,
    pub blue: u16,
}

// ==================== Proxy ====================

/// Low-level proxy: per-servo watchers updated by CAN callbacks + raw command senders.
pub struct MecaProxy<B: SabotterBoard> {
    can: GalipeurCan<B>,

    arms: [[Watcher<ArmStatus>; 4]; 3],
    clamps: [[Watcher<ClampStatus>; 3]; 3],
    translations: [Watcher<TranslationStatus>; 3],
    color_raw: [[Watcher<ColorRawStatus>; 4]; 3],
    battery_voltage_mv: Watcher<Option<u16>>,

    last_ping: Arc<AtomicU8>,
}

impl<B: SabotterBoard> Clone for MecaProxy<B> {
    fn clone(&self) -> Self {
        Self {
            can: self.can.clone(),
            arms: self.arms.clone(),
            clamps: self.clamps.clone(),
            translations: self.translations.clone(),
            color_raw: self.color_raw.clone(),
            battery_voltage_mv: self.battery_voltage_mv.clone(),
            last_ping: self.last_ping.clone(),
        }
    }
}

fn new_arm_watchers() -> [[Watcher<ArmStatus>; 4]; 3] {
    std::array::from_fn(|_| std::array::from_fn(|_| Watcher::new(ArmStatus::default())))
}

fn new_clamp_watchers() -> [[Watcher<ClampStatus>; 3]; 3] {
    std::array::from_fn(|_| std::array::from_fn(|_| Watcher::new(ClampStatus::default())))
}

fn new_translation_watchers() -> [Watcher<TranslationStatus>; 3] {
    std::array::from_fn(|_| Watcher::new(TranslationStatus::default()))
}

fn new_color_raw_watchers() -> [[Watcher<ColorRawStatus>; 4]; 3] {
    std::array::from_fn(|_| std::array::from_fn(|_| Watcher::new(ColorRawStatus::default())))
}

impl<B: SabotterBoard> MecaProxy<B> {
    pub fn new(can: GalipeurCan<B>) -> Self {
        let arms = new_arm_watchers();
        let clamps = new_clamp_watchers();
        let translations = new_translation_watchers();
        let color_raw = new_color_raw_watchers();
        let battery_voltage_mv = Watcher::new(None);
        let last_ping = Arc::new(AtomicU8::new(0));

        // Clone handles for the callback.
        let cb_arms = arms.clone();
        let cb_clamps = clamps.clone();
        let cb_translations = translations.clone();
        let cb_color_raw = color_raw.clone();
        let cb_battery = battery_voltage_mv.clone();
        let cb_last_ping = last_ping.clone();

        let mut log_decoder = LogDecoder::new();
        can.add_callback(move |msg| {
            if log_decoder.process_and_log("picotter", msg) {
                return;
            }
            match msg {
            CanMessage::Ping { value } => {
                cb_last_ping.store(*value, Ordering::Relaxed);
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
                if let Some(w) = cb_arms
                    .get(target.module as usize)
                    .and_then(|m| m.get(target.arm as usize))
                {
                    w.set(ArmStatus {
                        position: *position,
                        hue: (color & 0x7F) as u16 * 360 / 128,
                        color_detected: color & 0x80 != 0,
                        pump: *pump,
                        valve: *valve,
                        error: *error,
                        flags: *flags,
                        pump_current: *pump_current,
                    });
                }
            }
            CanMessage::TranslationStatus {
                module,
                position,
                error,
                flags,
            } => {
                if let Some(w) = cb_translations.get(*module as usize) {
                    w.set(TranslationStatus {
                        position: *position,
                        error: *error,
                        flags: *flags,
                    });
                }
            }
            CanMessage::ClampStatus {
                target,
                position,
                error,
                flags,
            } => {
                if let Some(w) = cb_clamps
                    .get(target.module as usize)
                    .and_then(|m| m.get(target.servo as usize))
                {
                    w.set(ClampStatus {
                        position: *position,
                        error: *error,
                        flags: *flags,
                    });
                }
            }
            CanMessage::ColorSensorRaw {
                target,
                clear,
                red,
                green,
                blue,
            } => {
                if let Some(w) = cb_color_raw
                    .get(target.module as usize)
                    .and_then(|m| m.get(target.arm as usize))
                {
                    w.set(ColorRawStatus {
                        clear: *clear,
                        red: *red,
                        green: *green,
                        blue: *blue,
                    });
                }
            }
            CanMessage::BatteryStatus { voltage_mv, .. } => {
                cb_battery.set(Some(*voltage_mv));
            }
            _ => {}
            }
        });

        Self {
            can,
            arms,
            clamps,
            translations,
            color_raw,
            battery_voltage_mv,
            last_ping,
        }
    }

    // --- Watcher accessors ---

    pub fn arm_watcher(&self, module: u8, arm: u8) -> &Watcher<ArmStatus> {
        &self.arms[module as usize][arm as usize]
    }

    pub fn clamp_watcher(&self, module: u8, servo: ClampServo) -> &Watcher<ClampStatus> {
        &self.clamps[module as usize][servo as usize]
    }

    pub fn translation_watcher(&self, module: u8) -> &Watcher<TranslationStatus> {
        &self.translations[module as usize]
    }

    pub fn color_raw_watcher(&self, module: u8, arm: u8) -> &Watcher<ColorRawStatus> {
        &self.color_raw[module as usize][arm as usize]
    }

    pub fn battery_voltage_mv(&self) -> Option<u16> {
        self.battery_voltage_mv.get()
    }

    // --- Raw CAN commands ---

    pub fn send_message(&self, msg: &CanMessage) {
        self.can.send(msg);
    }

    pub fn set_arm_position(&self, module: u8, arm: u8, position: u16, time_ms: u16) {
        self.can.send(&CanMessage::SetArm {
            target: ArmTarget::new(module, arm),
            position,
            time_ms,
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

    pub fn set_clamp_position(&self, module: u8, servo: ClampServo, position: u16, time_ms: u16) {
        self.can.send(&CanMessage::SetClamp {
            target: ClampTarget::new(module, servo),
            position,
            time_ms,
        });
    }

    pub fn set_clamp_torque(&self, module: u8, servo: ClampServo, enable: bool) {
        self.can.send(&CanMessage::SetClampTorque {
            target: ClampTarget::new(module, servo),
            enable,
        });
    }

    pub fn set_servo_id(&self, bus: cancaner::ServoBus, origin_id: u8, new_id: u8) {
        self.can.send(&CanMessage::SetServoId { bus, origin_id, new_id });
    }

    pub fn scan_bus(&self, bus: cancaner::ServoBus) {
        self.can.send(&CanMessage::ScanBus { bus });
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
