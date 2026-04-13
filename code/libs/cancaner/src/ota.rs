//! OTA (Over-The-Air) firmware update helpers for CAN bus
//!
//! The OTA protocol allows updating firmware via CAN bus with flow control:
//!
//! 1. Sender sends OTA_START with firmware size and CRC32
//! 2. Receiver responds OTA_READY (ok/busy/no_space)
//! 3. Sender sends OTA_DATA chunks (6 bytes each, indexed)
//! 4. Receiver responds OTA_ACK for each chunk (flow control)
//! 5. Sender sends OTA_FINISH when complete
//! 6. Receiver verifies CRC and responds OTA_RESULT
//! 7. If OK, sender sends OTA_REBOOT to apply update

use crate::message::CanMessage;
use crate::types::{OtaReadyStatus, OtaResultStatus};
use crate::protocol::OTA_CHUNK_SIZE;

/// OTA sender state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaSenderState {
    /// Idle, ready to start new transfer
    Idle,
    /// Waiting for OTA_READY response
    WaitingReady,
    /// Sending data chunks
    Sending { next_chunk: u16, total_chunks: u16 },
    /// Waiting for ACK for a specific chunk
    WaitingAck { chunk_idx: u16, total_chunks: u16 },
    /// Waiting for OTA_RESULT after sending OTA_FINISH
    WaitingResult,
    /// Transfer complete, ready to reboot
    Complete,
    /// Error occurred
    Error,
}

/// OTA sender for transmitting firmware updates
pub struct OtaSender<'a> {
    firmware: &'a [u8],
    fw_crc32: u32,
    state: OtaSenderState,
}

impl<'a> OtaSender<'a> {
    /// Create a new OTA sender with firmware data
    ///
    /// The CRC32 should be pre-computed by the caller using a CRC32 algorithm
    /// (e.g., crc32fast crate or similar).
    pub fn new(firmware: &'a [u8], fw_crc32: u32) -> Self {
        Self {
            firmware,
            fw_crc32,
            state: OtaSenderState::Idle,
        }
    }

    /// Get current state
    pub fn state(&self) -> OtaSenderState {
        self.state
    }

    /// Start OTA transfer - returns OTA_START message to send
    pub fn start(&mut self) -> CanMessage {
        self.state = OtaSenderState::WaitingReady;
        CanMessage::OtaStart {
            fw_size: self.firmware.len() as u32,
            fw_crc32: self.fw_crc32,
        }
    }

    /// Process OTA_READY response
    ///
    /// Returns the first data chunk to send, or None on error
    pub fn handle_ready(&mut self, status: OtaReadyStatus) -> Option<CanMessage> {
        if self.state != OtaSenderState::WaitingReady {
            return None;
        }

        match status {
            OtaReadyStatus::Ok => {
                let total_chunks = self.total_chunks();
                if total_chunks == 0 {
                    // Empty firmware
                    self.state = OtaSenderState::WaitingResult;
                    return Some(CanMessage::OtaFinish);
                }
                self.state = OtaSenderState::WaitingAck {
                    chunk_idx: 0,
                    total_chunks,
                };
                Some(self.build_chunk(0))
            }
            OtaReadyStatus::Busy | OtaReadyStatus::NoSpace => {
                self.state = OtaSenderState::Error;
                None
            }
        }
    }

    /// Process OTA_ACK response
    ///
    /// Returns the next chunk to send, OTA_FINISH, or None if waiting/error
    pub fn handle_ack(&mut self, chunk_idx: u16, status: u8) -> Option<CanMessage> {
        match self.state {
            OtaSenderState::WaitingAck {
                chunk_idx: expected,
                total_chunks,
            } => {
                if chunk_idx != expected {
                    // Out of order ACK, ignore
                    return None;
                }
                if status != 0 {
                    // Error from receiver
                    self.state = OtaSenderState::Error;
                    return None;
                }

                let next = expected + 1;
                if next >= total_chunks {
                    // All chunks sent
                    self.state = OtaSenderState::WaitingResult;
                    Some(CanMessage::OtaFinish)
                } else {
                    self.state = OtaSenderState::WaitingAck {
                        chunk_idx: next,
                        total_chunks,
                    };
                    Some(self.build_chunk(next))
                }
            }
            _ => None,
        }
    }

    /// Process OTA_RESULT response
    ///
    /// Returns OTA_REBOOT message on success, None on error
    pub fn handle_result(&mut self, status: OtaResultStatus) -> Option<CanMessage> {
        if self.state != OtaSenderState::WaitingResult {
            return None;
        }

        match status {
            OtaResultStatus::Ok => {
                self.state = OtaSenderState::Complete;
                Some(CanMessage::OtaReboot)
            }
            OtaResultStatus::CrcError | OtaResultStatus::SizeMismatch => {
                self.state = OtaSenderState::Error;
                None
            }
        }
    }

    /// Abort the current transfer
    pub fn abort(&mut self) -> CanMessage {
        self.state = OtaSenderState::Idle;
        CanMessage::OtaAbort
    }

    /// Get progress as (sent_chunks, total_chunks)
    pub fn progress(&self) -> (u16, u16) {
        match self.state {
            OtaSenderState::WaitingAck {
                chunk_idx,
                total_chunks,
            } => (chunk_idx, total_chunks),
            OtaSenderState::WaitingResult | OtaSenderState::Complete => {
                let total = self.total_chunks();
                (total, total)
            }
            _ => (0, self.total_chunks()),
        }
    }

    fn total_chunks(&self) -> u16 {
        self.firmware.len().div_ceil(OTA_CHUNK_SIZE) as u16
    }

    fn build_chunk(&self, idx: u16) -> CanMessage {
        let offset = idx as usize * OTA_CHUNK_SIZE;
        let remaining = self.firmware.len() - offset;
        let len = remaining.min(OTA_CHUNK_SIZE);

        let mut data = [0u8; OTA_CHUNK_SIZE];
        data[..len].copy_from_slice(&self.firmware[offset..offset + len]);

        CanMessage::OtaData {
            chunk_idx: idx,
            data,
            data_len: len as u8,
        }
    }
}

/// OTA receiver state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaReceiverState {
    /// Idle, ready to receive
    Idle,
    /// Receiving data
    Receiving {
        expected_size: u32,
        expected_crc: u32,
        received_chunks: u16,
        total_chunks: u16,
    },
    /// Transfer complete, waiting for reboot command
    Complete,
    /// Error occurred
    Error,
}

/// OTA receiver callback trait
///
/// Implement this trait to handle OTA data on the target device.
pub trait OtaStorage {
    /// Called when OTA transfer starts
    ///
    /// Should prepare storage (e.g., erase flash partition).
    /// Return OtaReadyStatus indicating if ready to receive.
    fn prepare(&mut self, fw_size: u32) -> OtaReadyStatus;

    /// Write a chunk of firmware data
    ///
    /// Return true on success, false on error.
    fn write_chunk(&mut self, offset: u32, data: &[u8]) -> bool;

    /// Finalize the transfer and verify CRC
    ///
    /// Should compute CRC of received data and compare with expected.
    /// Return (status, computed_crc).
    fn finalize(&mut self, expected_size: u32, expected_crc: u32) -> (OtaResultStatus, u32);

    /// Apply the update (mark partition as bootable)
    ///
    /// Called when OTA_REBOOT is received.
    fn apply(&mut self);

    /// Abort the transfer
    ///
    /// Should clean up any partial data.
    fn abort(&mut self);
}

/// OTA receiver for receiving firmware updates
pub struct OtaReceiver<S: OtaStorage> {
    storage: S,
    state: OtaReceiverState,
}

impl<S: OtaStorage> OtaReceiver<S> {
    /// Create a new OTA receiver with the given storage backend
    pub fn new(storage: S) -> Self {
        Self {
            storage,
            state: OtaReceiverState::Idle,
        }
    }

    /// Get current state
    pub fn state(&self) -> OtaReceiverState {
        self.state
    }

    /// Handle OTA_START message
    ///
    /// Returns OTA_READY response
    pub fn handle_start(&mut self, fw_size: u32, fw_crc32: u32) -> CanMessage {
        let status = self.storage.prepare(fw_size);

        if status == OtaReadyStatus::Ok {
            let total_chunks = ((fw_size as usize + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE) as u16;
            self.state = OtaReceiverState::Receiving {
                expected_size: fw_size,
                expected_crc: fw_crc32,
                received_chunks: 0,
                total_chunks,
            };
        } else {
            self.state = OtaReceiverState::Error;
        }

        CanMessage::OtaReady { status }
    }

    /// Handle OTA_DATA message
    ///
    /// Returns OTA_ACK response
    pub fn handle_data(&mut self, chunk_idx: u16, data: &[u8], data_len: u8) -> CanMessage {
        match self.state {
            OtaReceiverState::Receiving {
                received_chunks,
                total_chunks,
                expected_size,
                expected_crc,
            } => {
                // Reject chunks too far ahead
                if chunk_idx > received_chunks {
                    return CanMessage::OtaAck {
                        chunk_idx,
                        status: 1,
                    };
                }

                // Duplicate chunk, ACK without re-writing
                if chunk_idx < received_chunks {
                    return CanMessage::OtaAck {
                        chunk_idx,
                        status: 0,
                    };
                }

                let offset = chunk_idx as u32 * OTA_CHUNK_SIZE as u32;
                let success = self.storage.write_chunk(offset, &data[..data_len as usize]);

                if success {
                    self.state = OtaReceiverState::Receiving {
                        expected_size,
                        expected_crc,
                        received_chunks: chunk_idx + 1,
                        total_chunks,
                    };
                    CanMessage::OtaAck {
                        chunk_idx,
                        status: 0,
                    }
                } else {
                    self.state = OtaReceiverState::Error;
                    CanMessage::OtaAck {
                        chunk_idx,
                        status: 1,
                    }
                }
            }
            _ => CanMessage::OtaAck {
                chunk_idx,
                status: 1,
            },
        }
    }

    /// Handle OTA_FINISH message
    ///
    /// Returns OTA_RESULT response
    pub fn handle_finish(&mut self) -> CanMessage {
        match self.state {
            OtaReceiverState::Receiving {
                expected_size,
                expected_crc,
                ..
            } => {
                let (status, crc_computed) = self.storage.finalize(expected_size, expected_crc);

                if status == OtaResultStatus::Ok {
                    self.state = OtaReceiverState::Complete;
                } else {
                    self.state = OtaReceiverState::Error;
                }

                CanMessage::OtaResult {
                    status,
                    crc_computed,
                }
            }
            _ => {
                self.state = OtaReceiverState::Error;
                CanMessage::OtaResult {
                    status: OtaResultStatus::SizeMismatch,
                    crc_computed: 0,
                }
            }
        }
    }

    /// Handle OTA_REBOOT message
    ///
    /// Calls storage.apply() and returns true if reboot should proceed
    pub fn handle_reboot(&mut self) -> bool {
        if self.state == OtaReceiverState::Complete {
            self.storage.apply();
            true
        } else {
            false
        }
    }

    /// Handle OTA_ABORT message
    pub fn handle_abort(&mut self) {
        self.storage.abort();
        self.state = OtaReceiverState::Idle;
    }

    /// Get progress as (received_chunks, total_chunks)
    pub fn progress(&self) -> (u16, u16) {
        match self.state {
            OtaReceiverState::Receiving {
                received_chunks,
                total_chunks,
                ..
            } => (received_chunks, total_chunks),
            OtaReceiverState::Complete => {
                // Could track this better, but return 100%
                (1, 1)
            }
            _ => (0, 0),
        }
    }

    /// Get mutable reference to storage (for direct access if needed)
    pub fn storage_mut(&mut self) -> &mut S {
        &mut self.storage
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock storage for testing
    struct MockStorage {
        data: [u8; 1024],
        written: usize,
        should_fail_write: bool,
        should_fail_prepare: bool,
    }

    impl MockStorage {
        fn new() -> Self {
            Self {
                data: [0u8; 1024],
                written: 0,
                should_fail_write: false,
                should_fail_prepare: false,
            }
        }
    }

    impl OtaStorage for MockStorage {
        fn prepare(&mut self, _fw_size: u32) -> OtaReadyStatus {
            if self.should_fail_prepare {
                OtaReadyStatus::NoSpace
            } else {
                self.written = 0;
                OtaReadyStatus::Ok
            }
        }

        fn write_chunk(&mut self, offset: u32, data: &[u8]) -> bool {
            if self.should_fail_write {
                return false;
            }
            let start = offset as usize;
            let end = start + data.len();
            if end <= self.data.len() {
                self.data[start..end].copy_from_slice(data);
                self.written = self.written.max(end);
                true
            } else {
                false
            }
        }

        fn finalize(&mut self, expected_size: u32, _expected_crc: u32) -> (OtaResultStatus, u32) {
            if self.written == expected_size as usize {
                (OtaResultStatus::Ok, 0xDEADBEEF)
            } else {
                (OtaResultStatus::SizeMismatch, 0)
            }
        }

        fn apply(&mut self) {}
        fn abort(&mut self) {
            self.written = 0;
        }
    }

    // ==================== OtaSender ====================

    #[test]
    fn sender_happy_path() {
        let firmware = [0xAAu8; 12]; // 12 bytes = 2 chunks
        let mut sender = OtaSender::new(&firmware, 0x12345678);
        assert_eq!(sender.state(), OtaSenderState::Idle);

        // Start
        let start_msg = sender.start();
        assert_eq!(sender.state(), OtaSenderState::WaitingReady);
        assert!(matches!(start_msg, CanMessage::OtaStart { fw_size: 12, fw_crc32: 0x12345678 }));

        // Ready
        let chunk0 = sender.handle_ready(OtaReadyStatus::Ok).unwrap();
        assert!(matches!(sender.state(), OtaSenderState::WaitingAck { chunk_idx: 0, .. }));
        assert!(matches!(chunk0, CanMessage::OtaData { chunk_idx: 0, data_len: 6, .. }));

        // ACK chunk 0 → get chunk 1
        let chunk1 = sender.handle_ack(0, 0).unwrap();
        assert!(matches!(sender.state(), OtaSenderState::WaitingAck { chunk_idx: 1, .. }));
        assert!(matches!(chunk1, CanMessage::OtaData { chunk_idx: 1, data_len: 6, .. }));

        // ACK chunk 1 → get OtaFinish
        let finish = sender.handle_ack(1, 0).unwrap();
        assert_eq!(sender.state(), OtaSenderState::WaitingResult);
        assert!(matches!(finish, CanMessage::OtaFinish));

        // Result OK → get OtaReboot
        let reboot = sender.handle_result(OtaResultStatus::Ok).unwrap();
        assert_eq!(sender.state(), OtaSenderState::Complete);
        assert!(matches!(reboot, CanMessage::OtaReboot));

        assert_eq!(sender.progress(), (2, 2));
    }

    #[test]
    fn sender_ready_busy() {
        let firmware = [0u8; 6];
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        assert!(sender.handle_ready(OtaReadyStatus::Busy).is_none());
        assert_eq!(sender.state(), OtaSenderState::Error);
    }

    #[test]
    fn sender_ack_error() {
        let firmware = [0u8; 6];
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        sender.handle_ready(OtaReadyStatus::Ok);
        // ACK with error status
        assert!(sender.handle_ack(0, 1).is_none());
        assert_eq!(sender.state(), OtaSenderState::Error);
    }

    #[test]
    fn sender_out_of_order_ack() {
        let firmware = [0u8; 18]; // 3 chunks
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        sender.handle_ready(OtaReadyStatus::Ok);
        // Out-of-order ACK should be ignored
        assert!(sender.handle_ack(1, 0).is_none());
        assert!(matches!(sender.state(), OtaSenderState::WaitingAck { chunk_idx: 0, .. }));
    }

    #[test]
    fn sender_empty_firmware() {
        let firmware: [u8; 0] = [];
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        let msg = sender.handle_ready(OtaReadyStatus::Ok).unwrap();
        assert!(matches!(msg, CanMessage::OtaFinish));
        assert_eq!(sender.state(), OtaSenderState::WaitingResult);
    }

    #[test]
    fn sender_result_crc_error() {
        let firmware = [0u8; 6];
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        sender.handle_ready(OtaReadyStatus::Ok);
        sender.handle_ack(0, 0);
        assert!(sender.handle_result(OtaResultStatus::CrcError).is_none());
        assert_eq!(sender.state(), OtaSenderState::Error);
    }

    #[test]
    fn sender_abort() {
        let firmware = [0u8; 6];
        let mut sender = OtaSender::new(&firmware, 0);
        sender.start();
        let abort_msg = sender.abort();
        assert!(matches!(abort_msg, CanMessage::OtaAbort));
        assert_eq!(sender.state(), OtaSenderState::Idle);
    }

    #[test]
    fn sender_progress() {
        let firmware = [0u8; 18]; // 3 chunks
        let mut sender = OtaSender::new(&firmware, 0);
        assert_eq!(sender.progress(), (0, 3));
        sender.start();
        sender.handle_ready(OtaReadyStatus::Ok);
        assert_eq!(sender.progress(), (0, 3));
        sender.handle_ack(0, 0);
        assert_eq!(sender.progress(), (1, 3));
        sender.handle_ack(1, 0);
        assert_eq!(sender.progress(), (2, 3));
        sender.handle_ack(2, 0); // OtaFinish
        assert_eq!(sender.progress(), (3, 3));
    }

    // ==================== OtaReceiver ====================

    #[test]
    fn receiver_happy_path() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        assert_eq!(receiver.state(), OtaReceiverState::Idle);

        // Start
        let ready = receiver.handle_start(12, 0xDEADBEEF);
        assert!(matches!(ready, CanMessage::OtaReady { status: OtaReadyStatus::Ok }));
        assert!(matches!(receiver.state(), OtaReceiverState::Receiving { .. }));

        // Data chunks
        let ack0 = receiver.handle_data(0, &[1, 2, 3, 4, 5, 6], 6);
        assert!(matches!(ack0, CanMessage::OtaAck { chunk_idx: 0, status: 0 }));

        let ack1 = receiver.handle_data(1, &[7, 8, 9, 10, 11, 12], 6);
        assert!(matches!(ack1, CanMessage::OtaAck { chunk_idx: 1, status: 0 }));

        // Finish
        let result = receiver.handle_finish();
        assert!(matches!(result, CanMessage::OtaResult { status: OtaResultStatus::Ok, .. }));
        assert_eq!(receiver.state(), OtaReceiverState::Complete);

        // Reboot
        assert!(receiver.handle_reboot());
    }

    #[test]
    fn receiver_prepare_fails() {
        let mut storage = MockStorage::new();
        storage.should_fail_prepare = true;
        let mut receiver = OtaReceiver::new(storage);

        let ready = receiver.handle_start(12, 0);
        assert!(matches!(ready, CanMessage::OtaReady { status: OtaReadyStatus::NoSpace }));
        assert_eq!(receiver.state(), OtaReceiverState::Error);
    }

    #[test]
    fn receiver_write_fails() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        receiver.handle_start(12, 0);

        receiver.storage_mut().should_fail_write = true;
        let ack = receiver.handle_data(0, &[1, 2, 3, 4, 5, 6], 6);
        assert!(matches!(ack, CanMessage::OtaAck { status: 1, .. }));
        assert_eq!(receiver.state(), OtaReceiverState::Error);
    }

    #[test]
    fn receiver_duplicate_chunk() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        receiver.handle_start(12, 0);
        receiver.handle_data(0, &[1, 2, 3, 4, 5, 6], 6);

        // Duplicate chunk 0 - should be silently ACKed (but note: chunk 0 is
        // NOT deduplicated due to the `chunk_idx > 0` check in the code)
        let ack = receiver.handle_data(0, &[0xFF; 6], 6);
        // chunk 0 gets re-written (known bug)
        assert!(matches!(ack, CanMessage::OtaAck { chunk_idx: 0, status: 0 }));
    }

    #[test]
    fn receiver_chunk_too_far_ahead() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        receiver.handle_start(24, 0);
        receiver.handle_data(0, &[0; 6], 6);

        // Chunk 2 when we've only received 1 - too far ahead
        let ack = receiver.handle_data(2, &[0; 6], 6);
        assert!(matches!(ack, CanMessage::OtaAck { status: 1, .. }));
    }

    #[test]
    fn receiver_abort() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        receiver.handle_start(12, 0);
        receiver.handle_data(0, &[0; 6], 6);

        receiver.handle_abort();
        assert_eq!(receiver.state(), OtaReceiverState::Idle);
    }

    #[test]
    fn receiver_reboot_when_not_complete() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        assert!(!receiver.handle_reboot());

        receiver.handle_start(6, 0);
        assert!(!receiver.handle_reboot());
    }

    #[test]
    fn receiver_finish_when_not_receiving() {
        let mut receiver = OtaReceiver::new(MockStorage::new());
        let result = receiver.handle_finish();
        assert!(matches!(result, CanMessage::OtaResult { status: OtaResultStatus::SizeMismatch, .. }));
        assert_eq!(receiver.state(), OtaReceiverState::Error);
    }
}
