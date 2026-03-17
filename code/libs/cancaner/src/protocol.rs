//! Protocol constants for CAN communication

/// CAN bus bitrate (500 kbps)
pub const CAN_BITRATE: u32 = 500_000;

/// Maximum CAN data length
pub const MAX_DATA_LEN: usize = 8;

/// Number of arms per module (used for flat encoding in CAN ID target field)
pub const ARMS_PER_MODULE: usize = 4;

/// OTA chunk data size (6 bytes per frame)
pub const OTA_CHUNK_SIZE: usize = 6;

/// Log message payload size (first frame, after seq + level)
pub const LOG_MSG_PAYLOAD_SIZE: usize = 6;

/// Log continuation payload size (after seq)
pub const LOG_CONT_PAYLOAD_SIZE: usize = 7;

/// Log end payload size (after seq + total_len)
pub const LOG_END_PAYLOAD_SIZE: usize = 6;
