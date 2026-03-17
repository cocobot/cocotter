//! Log transmission helpers for CAN bus
//!
//! Logs are transmitted in multi-frame format:
//! - LOG_MSG: First frame with seq, level, and up to 6 bytes of payload
//! - LOG_CONT: Continuation frames with seq and up to 7 bytes of payload
//! - LOG_END: Final frame with seq, total_len, and remaining payload
//!
//! Short logs (≤6 bytes) can be sent as LOG_MSG + LOG_END combined.

use crate::message::CanMessage;
use crate::protocol::{LOG_MSG_PAYLOAD_SIZE, LOG_CONT_PAYLOAD_SIZE, LOG_END_PAYLOAD_SIZE};
use crate::types::LogLevel;

/// Log message encoder
///
/// Encodes a log message into a sequence of CAN frames.
pub struct LogEncoder {
    seq_counter: u8,
}

impl LogEncoder {
    /// Create a new log encoder
    pub const fn new() -> Self {
        Self { seq_counter: 0 }
    }

    /// Encode a log message into CAN frames
    ///
    /// Returns an iterator of CanMessage that should be sent in order.
    /// The caller must ensure all frames are sent before starting a new message.
    pub fn encode<'a>(
        &'a mut self,
        level: LogLevel,
        message: &'a [u8],
    ) -> LogFrameIterator<'a> {
        let seq = self.seq_counter;
        self.seq_counter = self.seq_counter.wrapping_add(1);

        LogFrameIterator {
            seq,
            level,
            message,
            offset: 0,
            sent_first: false,
            done: false,
        }
    }
}

impl Default for LogEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator that yields CAN frames for a log message
pub struct LogFrameIterator<'a> {
    seq: u8,
    level: LogLevel,
    message: &'a [u8],
    offset: usize,
    sent_first: bool,
    done: bool,
}

impl<'a> Iterator for LogFrameIterator<'a> {
    type Item = CanMessage;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        let remaining = self.message.len() - self.offset;

        if !self.sent_first {
            // First frame: LOG_MSG
            self.sent_first = true;
            let payload_len = remaining.min(LOG_MSG_PAYLOAD_SIZE);
            let mut payload = [0u8; 6];
            payload[..payload_len].copy_from_slice(&self.message[self.offset..self.offset + payload_len]);
            self.offset += payload_len;

            // If this is the only frame needed, mark as done and return LOG_END instead
            if self.offset >= self.message.len() {
                self.done = true;
                return Some(CanMessage::LogEnd {
                    seq: self.seq,
                    total_len: self.message.len() as u8,
                    payload,
                    payload_len: payload_len as u8,
                });
            }

            return Some(CanMessage::LogMsg {
                seq: self.seq,
                level: self.level,
                payload,
                payload_len: payload_len as u8,
            });
        }

        // Check if this is the last frame
        if remaining <= LOG_END_PAYLOAD_SIZE {
            self.done = true;
            let mut payload = [0u8; 6];
            payload[..remaining].copy_from_slice(&self.message[self.offset..]);
            return Some(CanMessage::LogEnd {
                seq: self.seq,
                total_len: self.message.len() as u8,
                payload,
                payload_len: remaining as u8,
            });
        }

        // Continuation frame: LOG_CONT
        let payload_len = remaining.min(LOG_CONT_PAYLOAD_SIZE);
        let mut payload = [0u8; 7];
        payload[..payload_len].copy_from_slice(&self.message[self.offset..self.offset + payload_len]);
        self.offset += payload_len;

        Some(CanMessage::LogCont {
            seq: self.seq,
            payload,
            payload_len: payload_len as u8,
        })
    }
}

/// Log message decoder
///
/// Reassembles log messages from CAN frames.
pub struct LogDecoder {
    buffer: [u8; 256],
    buffer_len: usize,
    current_seq: Option<u8>,
    current_level: LogLevel,
}

impl LogDecoder {
    /// Create a new log decoder
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; 256],
            buffer_len: 0,
            current_seq: None,
            current_level: LogLevel::Info,
        }
    }

    /// Reset the decoder state
    pub fn reset(&mut self) {
        self.buffer_len = 0;
        self.current_seq = None;
    }

    /// Process a LOG_MSG frame
    ///
    /// Returns None (message started, wait for more frames)
    pub fn process_msg(&mut self, seq: u8, level: LogLevel, payload: &[u8]) -> Option<()> {
        self.reset();
        self.current_seq = Some(seq);
        self.current_level = level;

        let len = payload.len().min(self.buffer.len());
        self.buffer[..len].copy_from_slice(&payload[..len]);
        self.buffer_len = len;
        None
    }

    /// Process a LOG_CONT frame
    ///
    /// Returns None if successful, resets on sequence mismatch
    pub fn process_cont(&mut self, seq: u8, payload: &[u8]) -> Option<()> {
        if self.current_seq != Some(seq) {
            self.reset();
            return None;
        }

        let space_left = self.buffer.len() - self.buffer_len;
        let len = payload.len().min(space_left);
        self.buffer[self.buffer_len..self.buffer_len + len].copy_from_slice(&payload[..len]);
        self.buffer_len += len;
        None
    }

    /// Process a LOG_END frame
    ///
    /// Returns the complete message if successful
    pub fn process_end(&mut self, seq: u8, _total_len: u8, payload: &[u8]) -> Option<(LogLevel, &[u8])> {
        // If no current sequence, this is a single-frame message
        if self.current_seq.is_none() {
            self.current_seq = Some(seq);
            self.current_level = LogLevel::Info; // Will be overwritten by caller if needed
        }

        if self.current_seq != Some(seq) {
            self.reset();
            return None;
        }

        let space_left = self.buffer.len() - self.buffer_len;
        let len = payload.len().min(space_left);
        self.buffer[self.buffer_len..self.buffer_len + len].copy_from_slice(&payload[..len]);
        self.buffer_len += len;

        let level = self.current_level;
        Some((level, &self.buffer[..self.buffer_len]))
    }

    /// Get the current level (for single-frame LOG_END messages)
    pub fn set_level(&mut self, level: LogLevel) {
        self.current_level = level;
    }
}

impl Default for LogDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// Tests require std for Vec - run with: cargo test -p cancaner --features std
#[cfg(all(test, feature = "std"))]
mod tests {
    extern crate std;
    use std::vec::Vec;
    use super::*;

    #[test]
    fn test_short_message() {
        let mut encoder = LogEncoder::new();
        let message = b"Hello";
        let frames: Vec<_> = encoder.encode(LogLevel::Info, message).collect();

        assert_eq!(frames.len(), 1);
        match &frames[0] {
            CanMessage::LogEnd { seq, total_len, payload, payload_len } => {
                assert_eq!(seq, &0);
                assert_eq!(total_len, &5);
                assert_eq!(payload_len, &5);
                assert_eq!(&payload[..*payload_len as usize], b"Hello");
            }
            _ => panic!("Expected LogEnd"),
        }
    }

    #[test]
    fn test_medium_message() {
        let mut encoder = LogEncoder::new();
        // 12 bytes: LOG_MSG (6) + LOG_END (6) = 2 frames
        let message = b"Hello World!"; // 12 bytes
        let frames: Vec<_> = encoder.encode(LogLevel::Warn, message).collect();

        assert_eq!(frames.len(), 2);
        match &frames[0] {
            CanMessage::LogMsg { seq, level, payload_len, .. } => {
                assert_eq!(seq, &0);
                assert_eq!(level, &LogLevel::Warn);
                assert_eq!(payload_len, &6);
            }
            _ => panic!("Expected LogMsg"),
        }
        match &frames[1] {
            CanMessage::LogEnd { seq, total_len, payload_len, .. } => {
                assert_eq!(seq, &0);
                assert_eq!(total_len, &12);
                assert_eq!(payload_len, &6);
            }
            _ => panic!("Expected LogEnd"),
        }
    }

    #[test]
    fn test_long_message() {
        let mut encoder = LogEncoder::new();
        let message = b"This is a longer message that needs continuation frames"; // 55 bytes
        let frames: Vec<_> = encoder.encode(LogLevel::Debug, message).collect();

        // First frame has 6 bytes, subsequent frames have 7 bytes (LOG_CONT) or 6 (LOG_END)
        assert!(frames.len() >= 2);

        // First should be LogMsg
        assert!(matches!(frames[0], CanMessage::LogMsg { .. }));

        // Last should be LogEnd
        assert!(matches!(frames.last().unwrap(), CanMessage::LogEnd { .. }));
    }

    #[test]
    fn test_seq_increments() {
        let mut encoder = LogEncoder::new();

        let frames1: Vec<_> = encoder.encode(LogLevel::Info, b"A").collect();
        let frames2: Vec<_> = encoder.encode(LogLevel::Info, b"B").collect();

        let seq1 = match &frames1[0] {
            CanMessage::LogEnd { seq, .. } => *seq,
            _ => panic!("Expected LogEnd"),
        };
        let seq2 = match &frames2[0] {
            CanMessage::LogEnd { seq, .. } => *seq,
            _ => panic!("Expected LogEnd"),
        };

        assert_eq!(seq1, 0);
        assert_eq!(seq2, 1);
    }

    // ==================== Roundtrip encoder → decoder ====================

    /// Feed encoder frames into decoder, return decoded message
    fn roundtrip_log(level: LogLevel, message: &[u8]) -> (LogLevel, Vec<u8>) {
        let mut encoder = LogEncoder::new();
        let mut decoder = LogDecoder::new();

        let frames: Vec<_> = encoder.encode(level, message).collect();

        for frame in &frames {
            match frame {
                CanMessage::LogMsg { seq, level, payload, payload_len } => {
                    decoder.process_msg(*seq, *level, &payload[..*payload_len as usize]);
                }
                CanMessage::LogCont { seq, payload, payload_len } => {
                    decoder.process_cont(*seq, &payload[..*payload_len as usize]);
                }
                CanMessage::LogEnd { seq, total_len, payload, payload_len } => {
                    let result = decoder.process_end(*seq, *total_len, &payload[..*payload_len as usize]);
                    if let Some((lvl, data)) = result {
                        return (lvl, data.to_vec());
                    }
                }
                _ => panic!("Unexpected frame type"),
            }
        }
        panic!("Decoder did not produce a result");
    }

    #[test]
    fn roundtrip_short() {
        let (level, data) = roundtrip_log(LogLevel::Error, b"Hi");
        assert_eq!(data, b"Hi");
        // Note: single-frame messages (LogEnd only) don't carry level in the frame,
        // so the decoder defaults to Info unless set_level() was called.
        // This is a known API limitation.
        assert_eq!(level, LogLevel::Info);
    }

    #[test]
    fn roundtrip_medium() {
        let (level, data) = roundtrip_log(LogLevel::Warn, b"Hello World!");
        assert_eq!(level, LogLevel::Warn);
        assert_eq!(data, b"Hello World!");
    }

    #[test]
    fn roundtrip_long() {
        let msg = b"This is a longer message that needs continuation frames!!";
        let (level, data) = roundtrip_log(LogLevel::Debug, msg);
        assert_eq!(level, LogLevel::Debug);
        assert_eq!(data, msg.as_slice());
    }

    #[test]
    fn roundtrip_exact_6_bytes() {
        // Exactly LOG_MSG_PAYLOAD_SIZE bytes - single LogEnd frame
        let (_, data) = roundtrip_log(LogLevel::Info, b"123456");
        assert_eq!(data, b"123456");
    }

    #[test]
    fn roundtrip_7_bytes() {
        // 7 bytes: LogMsg (6) + LogEnd (1)
        let (level, data) = roundtrip_log(LogLevel::Error, b"1234567");
        assert_eq!(level, LogLevel::Error);
        assert_eq!(data, b"1234567");
    }

    #[test]
    fn decoder_seq_mismatch_resets() {
        let mut decoder = LogDecoder::new();

        // Start with seq 0
        decoder.process_msg(0, LogLevel::Info, b"Hello");

        // Continuation with wrong seq - should reset
        decoder.process_cont(1, b" World");

        // End with seq 0 - decoder was reset so current_seq is None.
        // process_end treats this as a single-frame message (only the end payload).
        let result = decoder.process_end(0, 1, b"!");
        assert!(result.is_some());
        let (_, data) = result.unwrap();
        // Only the end payload survives, "Hello" was lost
        assert_eq!(data, b"!");
    }

    #[test]
    fn decoder_seq_mismatch_end_rejected() {
        let mut decoder = LogDecoder::new();

        // Start with seq 0
        decoder.process_msg(0, LogLevel::Info, b"Hello");

        // End with wrong seq - should fail
        let result = decoder.process_end(1, 5, b"");
        assert!(result.is_none());
    }
}
