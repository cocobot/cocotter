//! CAN Protocol wrapper for picotter (Embassy/STM32)
//!
//! Provides Frame conversion on top of the shared cancaner protocol.

use embassy_stm32::can::Frame;
use embassy_stm32::can::frame::Header;
use embedded_can::Id;

// Re-export everything from cancaner
pub use cancaner::*;

<<<<<<< HEAD
/// Extension trait to convert CanMessage to/from Embassy Frame
pub trait CanMessageFrameExt {
    /// Parse a CAN message from an Embassy Frame
    fn from_frame(frame: &Frame) -> Option<CanMessage>;

    /// Convert a CAN message to an Embassy Frame
    fn to_frame(&self) -> Frame;
}

impl CanMessageFrameExt for CanMessage {
    fn from_frame(frame: &Frame) -> Option<CanMessage> {
        let id = match frame.header().id() {
            Id::Standard(id) => id.as_raw(),
            Id::Extended(_) => return None,
        };
        CanMessage::parse(id, frame.data())
    }

    fn to_frame(&self) -> Frame {
        let encoded = self.encode();
        Frame::new(
            Header::new(Id::Standard(encoded.id), encoded.len as u8, false),
            &encoded.data[..encoded.len],
        )
        .unwrap()
    }
}

=======
>>>>>>> origin/bry-dev
/// Build a ping response frame (for fast path in CAN handler)
pub fn ping_response_frame(value: u8) -> Frame {
    ping_response(value).to_frame()
}
