//! Common elements specific to Eurobot contest
use amatheur::XY;
use crate::Color;


/// Table size in millimeters
pub const TABLE_SIZE: XY = XY::new(3000.0, 2000.0);


/// Robot team
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
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
            Self::None => Color::new(0.0, 0.0, 0.0),
            Self::Left => Color::new(0.8, 0.8, 0.0),
            Self::Right => Color::new(0.0, 0.0, 1.0),
        }
    }
}

impl std::fmt::Display for Team {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

