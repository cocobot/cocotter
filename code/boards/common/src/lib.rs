///! Common elements shared by all boards
#[cfg(not(target_os = "espidf"))]
pub mod mock;


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

