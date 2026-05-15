//! Mid-level movement primitives for the meca.
//!
//! Contains all the hard-coded servo positions (calibration data) and
//! self-waiting primitives that send a command and block until the
//! corresponding watcher reports movement completion.
//!
//! The `mod.rs` layer must only use these primitives — no raw positions
//! should leak out of this file.

use std::time::Duration;

use board_common::Team;
use board_sabotter::SabotterBoard;
use cancaner::{ClampServo, ValveMode};

use super::proxy::{ArmStatus, ClampStatus, MecaProxy, TranslationStatus, Watcher};

// Reference hues for team colors (degrees, 0..360).
const HUE_YELLOW: u16 = 60;
const HUE_BLUE: u16 = 240;
pub const ALL_ARMS : &[u8] = &[0, 1, 2, 3];

fn hue_distance(a: u16, b: u16) -> u16 {
    let d = a.abs_diff(b) % 360;
    d.min(360 - d)
}

// ==================== Calibration constants ====================
//
// Values are grouped **per servo**: each block lists every preset of one
// physical servo so calibration can proceed servo-by-servo. Positions are
// all zeroed and must be tuned on hardware.

/// All presets for a single arm servo.
struct ArmCalib {
    pre_grab: u16,
    cleat_up: u16,
    down: u16,
    up: u16, // also serves as the rest/idle position
}

/// Presets for the clamp rotate servo (one per module).
struct RotateCalib {
    pickup: u16,
    hold: u16,
}

/// Presets for a clamp grip servo (left or right).
struct GripCalib {
    open: u16,
    close: u16,
}

/// All clamp servo presets for one module.
struct ClampCalib {
    rotate: RotateCalib,
    left: GripCalib,
    right: GripCalib,
}

/// Presets for a translation servo (one per module).
struct TranslationCalib {
    spread: u16,
    close: u16,
}

const ARMS: [[ArmCalib; 4]; 3] = [
    // Module 0
    [
        ArmCalib { cleat_up: 420, pre_grab: 360, down: 340, up: 746 },
        ArmCalib { cleat_up: 417, pre_grab: 357, down: 337, up: 743 },
        ArmCalib { cleat_up: 420, pre_grab: 360, down: 340, up: 746 },
        ArmCalib { cleat_up: 408, pre_grab: 348, down: 328, up: 740 },
    ],
    // Module 1
    [
        ArmCalib { cleat_up: 405, pre_grab: 355, down: 335, up: 741 },
        ArmCalib { cleat_up: 380, pre_grab: 380, down: 360, up: 766 },
        ArmCalib { cleat_up: 340, pre_grab: 340, down: 320, up: 726 },
        ArmCalib { cleat_up: 375, pre_grab: 325, down: 305, up: 700 },
    ],
    // Module 2
    [
        ArmCalib { cleat_up: 420, pre_grab: 360, down: 340, up: 740 },
        ArmCalib { cleat_up: 420, pre_grab: 370, down: 340, up: 735 },
        ArmCalib { cleat_up: 420, pre_grab: 375, down: 355, up: 760 },
        ArmCalib { cleat_up: 415, pre_grab: 355, down: 335, up: 746 },
    ],
];

const CLAMPS: [ClampCalib; 3] = [
    // Module 0
    ClampCalib {
        rotate: RotateCalib { pickup: 830, hold: 650 - 40 },  //hold = horizontal - 40
        left:  GripCalib { open: 316 - 40, close: 660 },
        right: GripCalib { open: 618 + 40, close: 275 },
    },
    // Module 1 — TODO calibrer
    ClampCalib {
        rotate: RotateCalib { pickup: 870, hold: 670 - 40 },
        left:  GripCalib { open: 420 - 40, close: 654 },
        right: GripCalib { open: 540 + 40, close: 280 },
    },
    // Module 2
    ClampCalib {
        rotate: RotateCalib { pickup: 840, hold: 660 - 40 },
        left:  GripCalib { open: 375 - 40, close: 590 },
        right: GripCalib { open: 535 + 40, close: 300 },
    },
];

const TRANSLATIONS: [TranslationCalib; 3] = [
    TranslationCalib { spread: 640, close: 910 }, // Module 0
    TranslationCalib { spread: 640, close: 940 }, // Module 1
    TranslationCalib { spread: 340, close: 710 }, // Module 2
];

const MOVE_TIME_MS: u16 = 50;
const SLOW_MOVE_TIME_MS: u16 = 500;
const WAIT_TIMEOUT: Duration = Duration::from_millis(250);

// ==================== MecaPrimitives ====================

/// Mid-level self-waiting movement primitives.
pub struct MecaPrimitives<B: SabotterBoard> {
    proxy: MecaProxy<B>,
}

impl<B: SabotterBoard> Clone for MecaPrimitives<B> {
    fn clone(&self) -> Self {
        Self { proxy: self.proxy.clone() }
    }
}

impl<B: SabotterBoard> MecaPrimitives<B> {

    pub fn new(proxy: MecaProxy<B>) -> Self {
        Self { proxy }
    }

    // ---------- internal helpers ----------

    fn wait_not_moving_arm(w: &Watcher<ArmStatus>, seq_before: u64, target: u16) -> bool {
        w.wait_until(seq_before, |s| {
            !s.flags.moving || s.position.abs_diff(target) < 25 
        }, WAIT_TIMEOUT)
    }

    fn wait_not_moving_clamp(w: &Watcher<ClampStatus>, seq_before: u64, target: u16) -> bool {
        w.wait_until(seq_before, |s| {
            !s.flags.moving || s.position.abs_diff(target) < 25
        }, WAIT_TIMEOUT)
    }

    fn wait_not_moving_translation(w: &Watcher<TranslationStatus>, seq_before: u64, target: u16) -> bool {
        w.wait_until(seq_before, |s| {
            !s.flags.moving || s.position.abs_diff(target) < 25
        }, WAIT_TIMEOUT)
    }

    // ---------- Arm movements ----------

    fn arm_move(&self, module: u8, arm: u8, position: u16) {
        let w = self.proxy.arm_watcher(module, arm);
        let seq = w.seq();
        self.proxy.set_arm_position(module, arm, position, MOVE_TIME_MS);
        if !Self::wait_not_moving_arm(w, seq, position) {
            log::warn!("arm_move timeout: module={} arm={}", module, arm);
        }
    }

    pub fn arm_pre_grab(&self, module: u8, arm: u8) {
        self.arm_move(module, arm, ARMS[module as usize][arm as usize].pre_grab);
    }

    pub fn arm_down(&self, module: u8, arm: u8) {
        self.arm_move(module, arm, ARMS[module as usize][arm as usize].down);
    }

    pub fn arm_up(&self, module: u8, arm: u8) {
        self.arm_move(module, arm, ARMS[module as usize][arm as usize].up);
    }

    /// Move a subset of arms to the same preset in parallel, then wait on each.
    fn arms_move<F>(&self, module: u8, arms: &[u8], pos_for: F)
    where
        F: Fn(u8) -> u16,
    {
        // Capture seq + send commands first, then wait — true parallelism.
        let mut seqs = [0u64; 4];
        for &arm in arms {
            let w = self.proxy.arm_watcher(module, arm);
            seqs[arm as usize] = w.seq();
            self.proxy.set_arm_position(module, arm, pos_for(arm), MOVE_TIME_MS);
        }
        for &arm in arms {
            let w = self.proxy.arm_watcher(module, arm);
            if !Self::wait_not_moving_arm(w, seqs[arm as usize], pos_for(arm)) {
                log::warn!("arms_move timeout: module={} arm={} target={}", module, arm, pos_for(arm));
            }
        }
    }

    fn arms_move_slow<F>(&self, module: u8, arms: &[u8], pos_for: F)
    where
        F: Fn(u8) -> u16,
    {
        // Capture seq + send commands first, then wait — true parallelism.
        let mut seqs = [0u64; 4];
        for &arm in arms {
            let w = self.proxy.arm_watcher(module, arm);
            seqs[arm as usize] = w.seq();
            self.proxy.set_arm_position(module, arm, pos_for(arm), SLOW_MOVE_TIME_MS);
        }
        for &arm in arms {
            let w = self.proxy.arm_watcher(module, arm);
            if !Self::wait_not_moving_arm(w, seqs[arm as usize], pos_for(arm)) {
                log::warn!("arms_move timeout: module={} arm={} target={}", module, arm, pos_for(arm));
            }
        }
    }

    pub fn arms_pre_grab(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].pre_grab);
    }

    pub fn arms_give_space_from_clamp_rotation(&self, module: u8, arms: &[u8]) {
        self.arms_move_slow(module, arms, |a| ARMS[module as usize][a as usize].pre_grab + 300);
    }

    pub fn arms_pre_release_good_color(&self, module: u8, arms: &[u8]) {
        self.arms_move_slow(module, arms, |a| ARMS[module as usize][a as usize].pre_grab + 100);
    }

    pub fn arms_pre_release_bad_color(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].up + 30);
    }
    
    pub fn arms_cleat_up(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].cleat_up);
    }

    pub fn arms_down(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].down);
    }

    pub fn arms_up(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].up);
    }

    pub fn arms_up_for_clamp_release(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].up + 10);
    }


    // ---------- Clamp movements ----------

    fn clamp_rotate_move(&self, module: u8, position: u16) {
        let w = self.proxy.clamp_watcher(module, ClampServo::Rotate);
        let seq = w.seq();
        self.proxy
            .set_clamp_position(module, ClampServo::Rotate, position, MOVE_TIME_MS);
        if !Self::wait_not_moving_clamp(w, seq, position) {
            log::warn!("clamp_rotate timeout: module={}", module);
        }
    }

    pub fn clamp_rotate_pickup(&self, module: u8) {
        self.clamp_rotate_move(module, CLAMPS[module as usize].rotate.pickup);
    }

    pub fn clamp_rotate_hold(&self, module: u8) {
        self.clamp_rotate_move(module, CLAMPS[module as usize].rotate.hold);
    }

    /// Move both grip servos in parallel to the same preset (open or close).
    fn clamp_grip_move(&self, module: u8, left_pos: u16, right_pos: u16) {
        let wl = self.proxy.clamp_watcher(module, ClampServo::Left);
        let wr = self.proxy.clamp_watcher(module, ClampServo::Right);
        let seq_l = wl.seq();
        let seq_r = wr.seq();
        self.proxy
            .set_clamp_position(module, ClampServo::Left, left_pos, MOVE_TIME_MS);
        self.proxy
            .set_clamp_position(module, ClampServo::Right, right_pos, MOVE_TIME_MS);
        if !Self::wait_not_moving_clamp(wl, seq_l, left_pos) {
            log::warn!("clamp_left timeout: module={}", module);
        }
        if !Self::wait_not_moving_clamp(wr, seq_r, right_pos) {
            log::warn!("clamp_right timeout: module={}", module);
        }
    }

    pub fn clamp_open(&self, module: u8) {
        let c = &CLAMPS[module as usize];
        self.clamp_grip_move(module, c.left.open, c.right.open);
    }

    pub fn clamp_close(&self, module: u8) {
        let c = &CLAMPS[module as usize];
        self.clamp_grip_move(module, c.left.close, c.right.close);
    }

    // ---------- Translation movements ----------

    fn translation_move(&self, module: u8, position: u16) {
        let w = self.proxy.translation_watcher(module);
        let seq = w.seq();
        self.proxy.set_translation(module, position, MOVE_TIME_MS);
        if !Self::wait_not_moving_translation(w, seq, position) {
            log::warn!("translation timeout: module={}", module);
        }
    }

    pub fn translation_spread(&self, module: u8) {
        self.translation_move(module, TRANSLATIONS[module as usize].spread);
    }

    pub fn translation_close(&self, module: u8) {
        self.translation_move(module, TRANSLATIONS[module as usize].close);
    }

    // ---------- Pump / valve (instant, no servo wait) ----------

    /// Enable pump, close valve.
    pub fn grab(&self, module: u8, arm: u8) {
        self.proxy.set_pump(module, arm, true);
        self.proxy.set_valve(module, arm, ValveMode::Off);
    }

    /// Open valve, disable pump.
    pub fn release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, ValveMode::On);
        self.proxy.set_pump(module, arm, false);
    }

    /// Close valve after release — call ~250 ms after `release`.
    pub fn end_release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, ValveMode::Off);
    }

    pub fn grabs(&self, module: u8, arms: &[u8]) {
        for &arm in arms {
            self.grab(module, arm);
        }
    }

    /// PWM-toggle valves (5 ms ON / 15 ms OFF, ~25% duty) for `duration`, then fully release.
    pub fn slow_releases(&self, module: u8, arms: &[u8], duration: Duration) {
        for &arm in arms {
            self.proxy.set_valve(module, arm, ValveMode::Toggle { on_ms: 5, off_ms: 15 });
        }
        std::thread::sleep(duration);
        self.releases(module, arms);
    }

    pub fn inf_slow_releases(&self, module: u8, arms: &[u8]) {
        for &arm in arms {
            self.proxy.set_valve(module, arm, ValveMode::Toggle { on_ms: 5, off_ms: 15 });
        }
    }

    pub fn releases(&self, module: u8, arms: &[u8]) {
        for &arm in arms {
            self.release(module, arm);
        }
    }

    pub fn end_releases(&self, module: u8, arms: &[u8]) {
        for &arm in arms {
            self.end_release(module, arm);
        }
    }

    // ---------- Reset to neutral position ----------

    /// Test all actuators across all modules, then move to neutral position.
    pub fn init_all_modules(&self) {
        // Test all pumps
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_pump(module, arm, true);
                std::thread::sleep(Duration::from_millis(50));
                self.proxy.set_pump(module, arm, false);
                std::thread::sleep(Duration::from_millis(50));
            }
        }

        // Test all valves
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_valve(module, arm, ValveMode::On);
                std::thread::sleep(Duration::from_millis(20));
                self.proxy.set_valve(module, arm, ValveMode::Off);
                std::thread::sleep(Duration::from_millis(20));
            }
        }

        // Wiggle arms down slightly
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_arm_position(module, arm, ARMS[module as usize][arm as usize].up - 100, 50);
            }
            self.proxy.set_translation(module, TRANSLATIONS[module as usize].close, 50);
            self.proxy.set_clamp_position(module, ClampServo::Rotate, CLAMPS[module as usize].rotate.hold, 50);
            self.proxy.set_clamp_position(module, ClampServo::Left, CLAMPS[module as usize].left.close, 50);
            self.proxy.set_clamp_position(module, ClampServo::Right, CLAMPS[module as usize].right.close, 50);
        }
        std::thread::sleep(Duration::from_millis(500));

        // Move all modules to neutral
        for module in 0..3 {
            self.reset_module(module);
        }
    }

    pub fn reset_module(&self, module: u8) {
        self.arms_up(module, ALL_ARMS);
        self.clamp_rotate_pickup(module);
        self.clamp_open(module);
        self.translation_spread(module);
    }

    /// Drop everything in the lower stage (arms): release vacuum and move arms up (no wait).
    pub fn drop_lower_stage(&self, module: u8) {
        self.inf_slow_releases(module, ALL_ARMS);
        for arm in 0..4u8 {
            self.proxy.set_arm_position(module, arm, ARMS[module as usize][arm as usize].up, MOVE_TIME_MS);
        }
    }

    /// Drop everything in the upper stage (clamp): rotate to pickup and open (no wait).
    pub fn drop_upper_stage(&self, module: u8) {
        let c = &CLAMPS[module as usize];
        self.proxy.set_clamp_position(module, ClampServo::Rotate, c.rotate.pickup, MOVE_TIME_MS);
        self.proxy.set_clamp_position(module, ClampServo::Left, c.left.open, MOVE_TIME_MS);
        self.proxy.set_clamp_position(module, ClampServo::Right, c.right.open, MOVE_TIME_MS);
    }

    // ---------- Color classification ----------

    /// Read the arm's current hue and classify it as a `Team`.
    /// Otherwise picks the closest reference hue (Left=yellow, Right=blue).
    pub fn read_arm_team(&self, module: u8, arm: u8) -> Team {
        let s = self.proxy.arm_watcher(module, arm).get();
        let h = s.hue % 360;
        if hue_distance(h, HUE_YELLOW) <= hue_distance(h, HUE_BLUE) {
            Team::Left
        } else {
            Team::Right
        }
    }

    /// Read the team classification for all 4 arms of a module.
    pub fn read_arms_teams(&self, module: u8) -> [Team; 4] {
        std::array::from_fn(|a| self.read_arm_team(module, a as u8))
    }
}
