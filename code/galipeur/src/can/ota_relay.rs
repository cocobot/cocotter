//! CAN OTA relay handler: receives firmware from BLE and relays to picotter via CAN

use std::time::Duration;

use ble::OtaHandler;
use board_sabotter::SabotterBoard;
use cancaner::{CanMessage, OtaReadyStatus, OtaResultStatus, RebootMode};

use super::GalipeurCan;

const CAN_OTA_CHUNK_SIZE: usize = 6;
/// Timeout waiting for CAN responses
const CAN_RESPONSE_TIMEOUT: Duration = Duration::from_secs(5);
/// Delay after sending reboot command before starting OTA
const REBOOT_DELAY: Duration = Duration::from_millis(1000);

/// OTA relay handler that forwards firmware to picotter over CAN.
///
/// Threading model:
/// - `data()` is called from the OTA handler thread (blocking is OK)
/// - CAN RX callback pushes responses to a flume channel
/// - Handler thread blocks on the channel waiting for ACKs
pub struct CanOtaRelayHandler<B : SabotterBoard + 'static> {
    can: GalipeurCan<B>,
    /// Receives OTA responses (OtaReady, OtaAck, OtaResult) from CAN RX callback
    response_rx: flume::Receiver<CanOtaResponse>,
    /// Firmware state
    fw_size: u32,
    /// Actual firmware byte offset (for BLE-level tracking)
    fw_offset: u32,
    next_chunk_idx: u16,
    total_chunks: u16,
    /// Buffer for partial CAN chunk spanning BLE data boundaries
    can_buf: [u8; CAN_OTA_CHUNK_SIZE],
    can_buf_len: usize,
}

enum CanOtaResponse {
    Ready(OtaReadyStatus),
    Ack { chunk_idx: u16, status: u8 },
    Result(OtaResultStatus),
}

impl<B: SabotterBoard + 'static> CanOtaRelayHandler<B> {
    pub fn new(can: GalipeurCan<B>) -> Self {
        let (response_tx, response_rx) = flume::bounded(8);

        // Register CAN callback for OTA responses from picotter
        can.add_callback(move |msg| {
            let resp = match msg {
                CanMessage::OtaReady { status } => CanOtaResponse::Ready(*status),
                CanMessage::OtaAck { chunk_idx, status } => {
                    CanOtaResponse::Ack { chunk_idx: *chunk_idx, status: *status }
                }
                CanMessage::OtaResult { status, .. } => CanOtaResponse::Result(*status),
                _ => return,
            };
            let _ = response_tx.try_send(resp);
        });

        Self {
            can: can.clone(),
            response_rx,
            fw_size: 0,
            fw_offset: 0,
            next_chunk_idx: 0,
            total_chunks: 0,
            can_buf: [0u8; CAN_OTA_CHUNK_SIZE],
            can_buf_len: 0,
        }
    }

    /// Drain any stale responses from previous (possibly aborted) OTA sessions
    fn drain_responses(&self) {
        while self.response_rx.try_recv().is_ok() {}
    }

    /// Wait for a specific response type with timeout
    fn wait_ready(&self) -> Option<OtaReadyStatus> {
        match self.response_rx.recv_timeout(CAN_RESPONSE_TIMEOUT) {
            Ok(CanOtaResponse::Ready(status)) => Some(status),
            Ok(_) => {
                log::warn!("CAN OTA relay: unexpected response while waiting for Ready");
                None
            }
            Err(_) => {
                log::error!("CAN OTA relay: timeout waiting for OtaReady");
                None
            }
        }
    }

    fn wait_ack(&self) -> Option<(u16, u8)> {
        match self.response_rx.recv_timeout(CAN_RESPONSE_TIMEOUT) {
            Ok(CanOtaResponse::Ack { chunk_idx, status }) => Some((chunk_idx, status)),
            Ok(_) => {
                log::warn!("CAN OTA relay: unexpected response while waiting for Ack");
                None
            }
            Err(_) => {
                log::error!("CAN OTA relay: timeout waiting for OtaAck");
                None
            }
        }
    }

    fn wait_result(&self) -> Option<OtaResultStatus> {
        match self.response_rx.recv_timeout(CAN_RESPONSE_TIMEOUT) {
            Ok(CanOtaResponse::Result(status)) => Some(status),
            Ok(_) => {
                log::warn!("CAN OTA relay: unexpected response while waiting for Result");
                None
            }
            Err(_) => {
                log::error!("CAN OTA relay: timeout waiting for OtaResult");
                None
            }
        }
    }

    /// Send one CAN OTA_DATA message and wait for ACK
    fn send_can_chunk(&mut self, data: &[u8; CAN_OTA_CHUNK_SIZE], len: usize) -> bool {
        self.can.send(&CanMessage::OtaData {
            chunk_idx: self.next_chunk_idx,
            data: *data,
            data_len: len as u8,
        });

        match self.wait_ack() {
            Some((idx, status)) => {
                if idx != self.next_chunk_idx {
                    log::error!(
                        "CAN OTA relay: ACK for wrong chunk: expected {}, got {idx}",
                        self.next_chunk_idx
                    );
                    return false;
                }
                if status != 0 {
                    log::error!("CAN OTA relay: ACK error status {status} for chunk {idx}");
                    return false;
                }
            }
            None => return false,
        }

        self.next_chunk_idx += 1;
        true
    }

    /// Flush the partial CAN chunk buffer (for the last chunk at OTA finish)
    fn flush_can_buf(&mut self) -> bool {
        if self.can_buf_len == 0 {
            return true;
        }
        let mut data = [0u8; CAN_OTA_CHUNK_SIZE];
        data[..self.can_buf_len].copy_from_slice(&self.can_buf[..self.can_buf_len]);
        let len = self.can_buf_len;
        self.can_buf_len = 0;
        self.send_can_chunk(&data, len)
    }

    /// Send a BLE data chunk as multiple CAN OTA_DATA messages with flow control.
    /// Buffers partial chunks across calls so CAN chunks are contiguously packed.
    fn relay_data_chunk(&mut self, data: &[u8]) -> bool {
        let mut pos = 0;

        // Fill existing buffer first
        if self.can_buf_len > 0 {
            let space = CAN_OTA_CHUNK_SIZE - self.can_buf_len;
            let to_copy = space.min(data.len());
            self.can_buf[self.can_buf_len..self.can_buf_len + to_copy]
                .copy_from_slice(&data[..to_copy]);
            self.can_buf_len += to_copy;
            pos = to_copy;

            if self.can_buf_len == CAN_OTA_CHUNK_SIZE {
                let buf = self.can_buf;
                self.can_buf_len = 0;
                if !self.send_can_chunk(&buf, CAN_OTA_CHUNK_SIZE) {
                    return false;
                }
            }
        }

        // Send full chunks directly
        while pos + CAN_OTA_CHUNK_SIZE <= data.len() {
            let mut can_data = [0u8; CAN_OTA_CHUNK_SIZE];
            can_data.copy_from_slice(&data[pos..pos + CAN_OTA_CHUNK_SIZE]);
            if !self.send_can_chunk(&can_data, CAN_OTA_CHUNK_SIZE) {
                return false;
            }
            pos += CAN_OTA_CHUNK_SIZE;
        }

        // Buffer remaining partial chunk
        let remaining = data.len() - pos;
        if remaining > 0 {
            self.can_buf[..remaining].copy_from_slice(&data[pos..]);
            self.can_buf_len = remaining;
        }

        true
    }
}

impl<B: SabotterBoard + 'static>  OtaHandler  for CanOtaRelayHandler<B> {
    fn start(&mut self, size: u32, crc: u32) -> u8 {
        self.drain_responses();
        self.fw_size = size;
        self.fw_offset = 0;
        self.next_chunk_idx = 0;
        self.total_chunks = ((size as usize + CAN_OTA_CHUNK_SIZE - 1) / CAN_OTA_CHUNK_SIZE) as u16;
        self.can_buf_len = 0;

        log::info!(
            "CAN OTA relay: starting, size={size}, crc=0x{crc:08x}, {} CAN chunks",
            self.total_chunks
        );

        // Reboot picotter into bootloader mode
        self.can.send(&CanMessage::Reboot { mode: RebootMode::Bootloader });
        std::thread::sleep(REBOOT_DELAY);

        // Send OTA START
        self.can.send(&CanMessage::OtaStart {
            fw_size: size,
            fw_crc32: crc,
        });

        // Wait for READY
        match self.wait_ready() {
            Some(OtaReadyStatus::Ok) => {
                log::info!("CAN OTA relay: picotter ready");
                0 // ok
            }
            Some(OtaReadyStatus::Busy) => {
                log::error!("CAN OTA relay: picotter busy");
                1 // busy
            }
            Some(OtaReadyStatus::NoSpace) => {
                log::error!("CAN OTA relay: picotter no space");
                2 // no_space
            }
            None => {
                log::error!("CAN OTA relay: no response from picotter");
                1 // busy (treat timeout as busy)
            }
        }
    }

    fn data(&mut self, offset: u32, data: &[u8]) -> u32 {
        if offset != self.fw_offset {
            log::warn!(
                "CAN OTA relay: offset mismatch: BLE sent {offset}, expected {}",
                self.fw_offset
            );
            return self.fw_offset;
        }

        if self.relay_data_chunk(data) {
            self.fw_offset += data.len() as u32;
            self.fw_offset
        } else {
            self.fw_offset
        }
    }

    fn finish(&mut self) -> u8 {
        // Flush any buffered partial CAN chunk (last chunk of firmware)
        if !self.flush_can_buf() {
            return 1;
        }

        log::info!("CAN OTA relay: sending FINISH");
        self.can.send(&CanMessage::OtaFinish);

        match self.wait_result() {
            Some(OtaResultStatus::Ok) => {
                log::info!("CAN OTA relay: picotter verified OK");
                0 // ok
            }
            Some(OtaResultStatus::CrcError) => {
                log::error!("CAN OTA relay: picotter CRC error");
                1 // crc_fail
            }
            Some(OtaResultStatus::SizeMismatch) => {
                log::error!("CAN OTA relay: picotter size mismatch");
                2 // size_mismatch
            }
            None => {
                log::error!("CAN OTA relay: no result from picotter");
                1
            }
        }
    }

    fn reboot(&mut self) {
        log::info!("CAN OTA relay: sending REBOOT to picotter");
        self.can.send(&CanMessage::OtaReboot);
    }

    fn abort(&mut self) {
        log::info!("CAN OTA relay: sending ABORT to picotter");
        self.can.send(&CanMessage::OtaAbort);
        self.fw_offset = 0;
        self.next_chunk_idx = 0;
        self.can_buf_len = 0;
    }
}
