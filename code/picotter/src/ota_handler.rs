//! OTA (Over-The-Air) firmware update handler
//!
//! Implements the OtaStorage trait from cancaner.
//! TODO: Integrate with embassy-boot-stm32 for actual flash programming.

use cancaner::ota::{OtaReceiver, OtaReceiverState, OtaStorage};
use cancaner::{CanMessage, OtaReadyStatus, OtaResultStatus};

/// Placeholder OTA storage implementation
///
/// Accepts OTA transfers but doesn't write to flash.
/// Replace with embassy-boot integration when ready.
pub struct OtaFlashStorage {
    expected_size: u32,
    received_size: u32,
}

impl OtaFlashStorage {
    pub const fn new() -> Self {
        Self {
            expected_size: 0,
            received_size: 0,
        }
    }
}

impl OtaStorage for OtaFlashStorage {
    fn prepare(&mut self, fw_size: u32) -> OtaReadyStatus {
        if fw_size > 256 * 1024 {
            return OtaReadyStatus::NoSpace;
        }

        self.expected_size = fw_size;
        self.received_size = 0;
        OtaReadyStatus::Ok
    }

    fn write_chunk(&mut self, offset: u32, data: &[u8]) -> bool {
        if offset != self.received_size {
            return false;
        }
        self.received_size += data.len() as u32;
        true
    }

    fn finalize(&mut self, expected_size: u32, expected_crc: u32) -> (OtaResultStatus, u32) {
        if self.received_size != expected_size {
            return (OtaResultStatus::SizeMismatch, 0);
        }
        // TODO: Compute actual CRC32 of received data
        (OtaResultStatus::Ok, expected_crc)
    }

    fn apply(&mut self) {
        // TODO: Mark DFU partition as bootable via embassy-boot
    }

    fn abort(&mut self) {
        self.expected_size = 0;
        self.received_size = 0;
    }
}

/// OTA handler wrapper
pub struct OtaHandler {
    receiver: OtaReceiver<OtaFlashStorage>,
}

impl OtaHandler {
    pub fn new() -> Self {
        Self {
            receiver: OtaReceiver::new(OtaFlashStorage::new()),
        }
    }

    pub fn state(&self) -> OtaReceiverState {
        self.receiver.state()
    }

    /// Handle an OTA-related CAN message, returns optional response
    pub fn handle_message(&mut self, msg: &CanMessage) -> Option<CanMessage> {
        match msg {
            CanMessage::OtaStart { fw_size, fw_crc32 } => {
                Some(self.receiver.handle_start(*fw_size, *fw_crc32))
            }
            CanMessage::OtaData {
                chunk_idx,
                data,
                data_len,
            } => Some(self.receiver.handle_data(*chunk_idx, data, *data_len)),
            CanMessage::OtaFinish => Some(self.receiver.handle_finish()),
            CanMessage::OtaReboot => {
                self.receiver.handle_reboot();
                None
            }
            CanMessage::OtaAbort => {
                self.receiver.handle_abort();
                None
            }
            _ => None,
        }
    }
}
