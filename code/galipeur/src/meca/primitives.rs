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
use cancaner::ClampServo;

use super::proxy::{ArmStatus, ClampStatus, MecaProxy, TranslationStatus, Watcher};

// Reference hues for team colors (degrees, 0..360).
const HUE_YELLOW: u16 = 60;
const HUE_BLUE: u16 = 240;

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
    // Module 0 — TODO calibrer
    [
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
    ],
    // Module 1 — TODO calibrer
    [
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
    ],
    // Module 2 — TODO calibrer
    [
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
        ArmCalib { pre_grab: 360, down: 340, up: 746 },
    ],
];

const CLAMPS: [ClampCalib; 3] = [
    // Module 0 — TODO calibrer
    ClampCalib {
        rotate: RotateCalib { pickup: 820, hold: 610 },
        left:  GripCalib { open: 316, close: 654 },
        right: GripCalib { open: 618, close: 280 },
    },
    // Module 1 — TODO calibrer
    ClampCalib {
        rotate: RotateCalib { pickup: 820, hold: 610 },
        left:  GripCalib { open: 316, close: 654 },
        right: GripCalib { open: 618, close: 280 },
    },
    // Module 2 — TODO calibrer
    ClampCalib {
        rotate: RotateCalib { pickup: 820, hold: 610 },
        left:  GripCalib { open: 316, close: 654 },
        right: GripCalib { open: 618, close: 280 },
    },
];

const TRANSLATIONS: [TranslationCalib; 3] = [
    TranslationCalib { spread: 640, close: 910 }, // Module 0 
    TranslationCalib { spread: 640, close: 910 }, // Module 1
    TranslationCalib { spread: 640, close: 910 }, // Module 2 
];

const MOVE_TIME_MS: u16 = 50;
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

    fn wait_not_moving_arm(w: &Watcher<ArmStatus>, seq_before: u64) -> bool {
        w.wait_until(seq_before, |s| !s.flags.moving, WAIT_TIMEOUT)
    }

    fn wait_not_moving_clamp(w: &Watcher<ClampStatus>, seq_before: u64) -> bool {
        w.wait_until(seq_before, |s| !s.flags.moving, WAIT_TIMEOUT)
    }

    fn wait_not_moving_translation(w: &Watcher<TranslationStatus>, seq_before: u64) -> bool {
        w.wait_until(seq_before, |s| !s.flags.moving, WAIT_TIMEOUT)
    }

    // ---------- Arm movements ----------

    fn arm_move(&self, module: u8, arm: u8, position: u16) {
        let w = self.proxy.arm_watcher(module, arm);
        let seq = w.seq();
        self.proxy.set_arm_position(module, arm, position, MOVE_TIME_MS);
        if !Self::wait_not_moving_arm(w, seq) {
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
            if !Self::wait_not_moving_arm(w, seqs[arm as usize]) {
                log::warn!("arms_move timeout: module={} arm={}", module, arm);
            }
        }
    }

    pub fn arms_pre_grab(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].pre_grab);
    }

    pub fn arms_down(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].down);
    }

    pub fn arms_up(&self, module: u8, arms: &[u8]) {
        self.arms_move(module, arms, |a| ARMS[module as usize][a as usize].up);
    }

    // ---------- Clamp movements ----------

    fn clamp_rotate_move(&self, module: u8, position: u16) {
        let w = self.proxy.clamp_watcher(module, ClampServo::Rotate);
        let seq = w.seq();
        self.proxy
            .set_clamp_position(module, ClampServo::Rotate, position, MOVE_TIME_MS);
        if !Self::wait_not_moving_clamp(w, seq) {
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
        if !Self::wait_not_moving_clamp(wl, seq_l) {
            log::warn!("clamp_left timeout: module={}", module);
        }
        if !Self::wait_not_moving_clamp(wr, seq_r) {
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
        if !Self::wait_not_moving_translation(w, seq) {
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
        self.proxy.set_valve(module, arm, false);
    }

    /// Open valve, disable pump.
    pub fn release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, true);
        self.proxy.set_pump(module, arm, false);
    }

    /// Close valve after release — call ~250 ms after `release`.
    pub fn end_release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, false);
    }

    pub fn grabs(&self, module: u8, arms: &[u8]) {
        for &arm in arms {
            self.grab(module, arm);
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

    // ---------- Color classification ----------

    /// Read the arm's current hue and classify it as a `Team`.
    /// Returns `Team::None` if the color sensor reports no detection.
    /// Otherwise picks the closest reference hue (Left=yellow, Right=blue).
    pub fn read_arm_team(&self, module: u8, arm: u8) -> Team {
        let s = self.proxy.arm_watcher(module, arm).get();
        if !s.color_detected {
            return Team::None;
        }
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
