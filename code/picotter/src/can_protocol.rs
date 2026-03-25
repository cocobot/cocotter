//! CAN Protocol wrapper for picotter (Embassy/STM32)
//!
//! Provides Frame conversion on top of the shared cancaner protocol.

use embassy_stm32::can::Frame;
use embassy_stm32::can::frame::Header;
use embedded_can::Id;

// Re-export everything from cancaner
pub use cancaner::*;

/// Build a ping response frame (for fast path in CAN handler)
pub fn ping_response_frame(value: u8) -> Frame {
    ping_response(value).to_frame()
}
