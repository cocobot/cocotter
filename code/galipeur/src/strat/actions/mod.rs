mod take_crate;
mod release;
mod end_of_match;

pub use take_crate::TakeCrateAction;
pub use release::ReleaseAction;
pub use end_of_match::EndOfMatchAction;

/// Max crates the robot can hold simultaneously
pub const MAX_CRATES: isize = 16;
/// Counter index for total crates held
pub const CTR_HELD: usize = 0;
/// Bit index for end of match trigger
pub const END_OF_MATCH: u8 = 63;