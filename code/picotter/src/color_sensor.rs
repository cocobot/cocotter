<<<<<<< HEAD
//! Color sensor: TCS3472 color matching logic
//!
//! Stores per-arm color decoding tables. Each entry maps a color_id (1-254)
//! to min/max ranges for each of the 4 channels (Clear, R, G, B).
//!
//! Return values:
//! - 255 = config incomplete (sensor or table not fully configured)
//! - 0 = config OK but no color matches
//! - 1-254 = matched color_id

/// Maximum number of color entries per arm
pub const MAX_COLOR_ENTRIES: usize = 8;

/// Number of channels (Clear, Red, Green, Blue)
const NUM_CHANNELS: usize = 4;

/// A single color entry with per-channel ranges
#[derive(Debug, Clone, Copy)]
pub struct ColorEntry {
    pub color_id: u8,
    pub ranges: [(u16, u16); NUM_CHANNELS], // (min, max) for C, R, G, B
    /// Bitmask tracking which channels have been configured (bit 0=C, 1=R, 2=G, 3=B)
    pub channels_set: u8,
}

impl ColorEntry {
    const fn new() -> Self {
        Self {
            color_id: 0,
            ranges: [(0, 0); NUM_CHANNELS],
            channels_set: 0,
        }
    }

    /// Returns true if all 4 channels have been configured
    pub fn is_complete(&self) -> bool {
        self.channels_set == 0x0F
    }
}

/// Per-arm color matching table
#[derive(Debug, Clone)]
pub struct ColorTable {
    entries: [ColorEntry; MAX_COLOR_ENTRIES],
    count: usize,
    /// True once SetColorSensorConfig has been received for this arm
    pub sensor_configured: bool,
}

impl ColorTable {
    pub const fn new() -> Self {
        Self {
            entries: [ColorEntry::new(); MAX_COLOR_ENTRIES],
            count: 0,
            sensor_configured: false,
        }
    }

    /// Clear all entries and reset sensor_configured
    pub fn clear(&mut self) {
        self.count = 0;
        self.sensor_configured = false;
    }

    /// Check if config is fully complete:
    /// - sensor_configured is true
    /// - at least one entry exists
    /// - all entries have all 4 channels set
    pub fn is_fully_configured(&self) -> bool {
        self.sensor_configured
            && self.count > 0
            && self.entries[..self.count].iter().all(|e| e.is_complete())
    }

    /// Set range for a specific color_id and channel.
    /// color_id=0 clears the entire table.
    pub fn set_range(&mut self, color_id: u8, channel: u8, min: u16, max: u16) {
        if color_id == 0 {
            self.count = 0;
            return;
        }
        if channel as usize >= NUM_CHANNELS {
            return;
        }

        // Find existing entry or add new one
        if let Some(entry) = self.entries[..self.count]
            .iter_mut()
            .find(|e| e.color_id == color_id)
        {
            entry.ranges[channel as usize] = (min, max);
            entry.channels_set |= 1 << channel;
        } else if self.count < MAX_COLOR_ENTRIES {
            let entry = &mut self.entries[self.count];
            entry.color_id = color_id;
            entry.ranges = [(0, u16::MAX); NUM_CHANNELS];
            entry.ranges[channel as usize] = (min, max);
            entry.channels_set = 1 << channel;
            self.count += 1;
        }
    }

    /// Match CRGB values against table.
    /// Returns 255 if config incomplete, 0 if no match, or first matching color_id.
    pub fn match_color(&self, crgb: &[u16; 4]) -> u8 {
        if !self.is_fully_configured() {
            return 255;
        }

        for entry in &self.entries[..self.count] {
            let mut all_match = true;
            for ch in 0..NUM_CHANNELS {
                let (min, max) = entry.ranges[ch];
                if crgb[ch] < min || crgb[ch] > max {
                    all_match = false;
                    break;
                }
            }
            if all_match {
                return entry.color_id;
            }
        }
        0
=======
//! Color sensor driver (placeholder)

use cancaner::Color;
use crate::arm::ArmError;
use crate::module::ColorSensor;

/// Dummy color sensor that always returns Unknown
pub struct DummyColorSensor;

impl DummyColorSensor {
    pub fn new() -> Self {
        Self
    }
}

impl ColorSensor for DummyColorSensor {
    async fn read_color(&mut self, _arm: u8) -> Result<Color, ArmError> {
        Ok(Color::Unknown)
>>>>>>> origin/bry-dev
    }
}
