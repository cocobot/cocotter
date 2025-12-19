///! Common elements shared by all boards
#[cfg(not(target_os = "espidf"))]
pub mod mock;

use std::time::{Duration, Instant};


/// Read battery voltage
pub trait BatteryLevel {
    /// Read battery voltage, return value in mV and as percentage
    fn read_vbatt(&mut self) -> (u16, u8);
}

/// Generic encoder, fetch a value of given type
pub trait Encoder<T> {
    type Error: core::fmt::Debug;

    /// Get encoded value
    fn get_value(&self) -> Result<T, Self::Error>;
}


/// Robot team
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Team {
    None,
    Left,
    Right,
}

impl Default for Team {
    fn default() -> Self {
        Self::None
    }
}

impl Team {
    /// Return full name, lowercase
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Left => "jaune",
            Self::Right => "bleu",

        }
    }

    /// Return full name, uppercase
    pub const fn name_upper(self) -> &'static str {
        match self {
            Self::None => "NONE",
            Self::Left => "JAUNE",
            Self::Right => "BLUE",

        }
    }

    /// Return name as a single letter
    pub const fn letter(self) -> char {
        match self {
            Self::None => '?',
            Self::Left => 'J',
            Self::Right => 'B',

        }
    }
}


/// Helper for periodic events
///
/// Call `update()` with the the current date to check if the event must be triggered.
/// After an update, the next update is scheduled based on the current date, not the previous
/// scheduled date. As a result error may propagate.
pub struct Periodicity {
    next: Instant,
    period: Duration,
}

impl Periodicity {
    pub fn new(period: Duration) -> Self {
        Self {
            next: Instant::now(),
            period,
        }
    }

    /// Update periodicity, return `true` if a new period is reached
    pub fn update(&mut self, now: &Instant) -> bool {
        if now >= &self.next {
            self.next = *now + self.period;
            true
        } else {
            false
        }
    }

    pub fn next(&self) -> &Instant {
        &self.next
    }

    pub fn period(&self) -> &Duration {
        &self.period
    }
}


