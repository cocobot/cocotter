//! Ground sensor state management
//!
//! Each module has a VCNL4040 proximity sensor for line detection.
//! The actual I2C communication is handled by `I2cDevices`.

/// Default threshold for detection
pub const DEFAULT_THRESHOLD: u16 = 100;

/// Ground sensor state
#[derive(Debug, Clone, Copy, Default)]
pub struct GroundSensorState {
    /// Current proximity value
    pub value: u16,
    /// Detection threshold
    pub threshold: u16,
    /// Is above threshold (detected)
    pub detected: bool,
}

impl GroundSensorState {
    pub fn new() -> Self {
        Self {
            value: 0,
            threshold: DEFAULT_THRESHOLD,
            detected: false,
        }
    }

    /// Update state with new value
    pub fn update(&mut self, value: u16) {
        self.value = value;
        self.detected = value >= self.threshold;
    }

    /// Set threshold and re-evaluate detection
    pub fn set_threshold(&mut self, threshold: u16) {
        self.threshold = threshold;
        self.detected = self.value >= threshold;
    }
}
