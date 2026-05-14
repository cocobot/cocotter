//! Ground sensor state management
//!
//! Each module has a VCNL4040 proximity sensor for line detection.
//! The actual I2C communication is handled by `I2cDevices`.

use cancaner::GroundThresholdMode;
use rtt_target::rprintln;

/// Default threshold for detection
pub const DEFAULT_THRESHOLD: u16 = 5;

/// Ground sensor state
#[derive(Debug, Clone, Copy)]
pub struct GroundSensorState {
    /// Current proximity value
    pub value: u16,
    /// Detection threshold
    pub threshold: u16,
    /// Is above threshold (detected)
    pub detected: bool,
    /// Threshold mode
    mode: GroundThresholdMode,
    /// Reference value captured when entering Delta mode
    reference: u16,
    /// Latching flag for delta mode (once detected, stays detected)
    latched: bool,
}

impl Default for GroundSensorState {
    fn default() -> Self {
        Self::new()
    }
}

impl GroundSensorState {
    pub fn new() -> Self {
        Self {
            value: 0,
            threshold: DEFAULT_THRESHOLD,
            detected: false,
            mode: GroundThresholdMode::Raw,
            reference: 0,
            latched: false,
        }
    }

    /// Update state with new value
    pub fn update(&mut self, value: u16) {
        self.value = value;
        //rprintln!("UPD {:?} {:?} {:?}", value, self.mode, self.reference);
        match self.mode {
            GroundThresholdMode::Raw => {
                self.detected = value >= self.threshold;
            }
            GroundThresholdMode::Delta => {
                if self.latched {
                    return;
                }
                let floor = self.reference.saturating_sub(self.threshold);
               
                self.detected  = value > floor;
            }
        }
    }

    /// Set mode and threshold, resetting delta state
    pub fn set_mode_and_threshold(&mut self, mode: GroundThresholdMode, threshold: u16) {
        self.threshold = threshold;
        self.latched = false;
        match mode {
            GroundThresholdMode::Raw => {
                self.reference = 0;
                self.detected = self.value >= threshold;
            }
            GroundThresholdMode::Delta => {
                rprintln!("Set ref {}", self.reference);
                if self.mode != mode {
                    self.reference = self.value;
                    self.detected = false;
                }
            }
        }
        self.mode = mode;

    }
}
