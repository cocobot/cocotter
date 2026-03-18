///! Common elements shared by all boards
#[cfg(not(target_os = "espidf"))]
pub mod mock;

use std::time::{Duration, Instant};


/// Battery level, read from board
#[derive(Clone, Copy, Default)]
pub struct BatteryLevel {
    pub mv: u16,
    pub percent: u8,
}

/// Read battery voltage
pub trait BatteryReader {
    fn read_vbatt(&mut self) -> BatteryLevel;
}

/// Generic encoder, fetch a value of given type
pub trait Encoder<T> {
    type Error: core::fmt::Debug;

    /// Get encoded value
    fn get_value(&self) -> Result<T, Self::Error>;
}


/// Robot team
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum Team {
    #[default]
    None,
    Left,
    Right,
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

    /// Return team color
    pub const fn color(self) -> Color {
        match self {
            Self::None => Color::new(0.5, 0.5, 0.5),
            Self::Left => Color::new(0.8, 0.8, 0.0),
            Self::Right => Color::new(0.0, 0.0, 1.0),
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

    /// Reset periodicity to given instance
    pub fn reset(&mut self, now: &Instant) {
        self.next = *now + self.period;
    }

    pub fn next(&self) -> &Instant {
        &self.next
    }

    pub fn period(&self) -> &Duration {
        &self.period
    }
}


/// RGB Color
#[derive(Clone, Debug)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32
}

impl Color {
    pub const BLACK: Self = Self::new(0.0, 0.0, 0.0);
    pub const WHITE: Self = Self::new(1.0, 1.0, 1.0);
    pub const RED: Self = Self::new(1.0, 0.0, 0.0);
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0);
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0);
    pub const CYAN: Self = Self::new(0.0, 1.0, 1.0);
    pub const MAGENTA: Self = Self::new(1.0, 0.0, 1.0);
    pub const YELLOW: Self = Self::new(1.0, 1.0, 0.0);

    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    pub const fn rgb8(r: u8, g: u8, b: u8) -> Self {
        Self {
            r: r as f32 / 255.0,
            g: g as f32 / 255.0,
            b: b as f32 / 255.0,
        }
    }
}

