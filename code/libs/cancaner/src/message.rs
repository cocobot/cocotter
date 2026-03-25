//! CAN message definitions and encoding/decoding
//!
//! New ID format: 0x[DOMAIN:3][CMD:4][TARGET:4]
//! - DOMAIN: 0-7 (System, Arm, Ground, Log, Ota, reserved)
//! - CMD: 0-F command within domain
//! - TARGET: 0-F target interpretation depends on domain

use embedded_can::{Frame, Id, StandardId};

use crate::protocol::{LOG_MSG_PAYLOAD_SIZE, LOG_CONT_PAYLOAD_SIZE, LOG_END_PAYLOAD_SIZE, MAX_DATA_LEN, OTA_CHUNK_SIZE};
use crate::types::*;


/// Encoded CAN message ready to be wrapped in a frame
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncodedMessage {
    pub id: StandardId,
    pub data: [u8; MAX_DATA_LEN],
    pub len: usize,
}

impl EncodedMessage {
    pub fn to_frame<F: Frame>(&self) -> F {
        // `self.len < 8` so `unwrap()` is safe
        F::new(self.id, &self.data[..self.len]).unwrap()
    }
}


/// Parsed CAN message organized by domain
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CanMessage {
    // ==================== SYSTEM Domain (0x0) ====================
    /// Ping request/response (ID: 0x000)
    Ping { value: u8 },

    /// Board info request (ID: 0x010)
    BoardInfoRequest,

    /// Board info response (ID: 0x011)
    BoardInfoResponse {
        fw_version: u32,
        uptime_ms: u32,
    },

    /// System error report (ID: 0x0E0)
    SystemError {
        error_type: ErrorType,
        error_code: u8,
    },

    /// Reboot command (ID: 0x0F0)
    Reboot { mode: RebootMode },

    /// Set servo ID command (ID: 0x020)
    /// Broadcasts on the specified bus to set servo ID
    SetServoId {
        bus: ServoBus,
        new_id: u8,
    },

    /// Set servo ID result (ID: 0x021)
    SetServoIdResult {
        bus: ServoBus,
        new_id: u8,
        success: bool,
    },

    // ==================== ARM Domain (0x1) ====================
    /// Set arm position, pump and valve (ID: 0x10T)
    SetArm {
        target: ArmTarget,
        position: u16,
        time_ms: u16,
        pump: bool,
        valve: bool,
    },

    /// Arm status report (ID: 0x11T)
    ArmStatus {
        target: ArmTarget,
        position: u16,
        color: Color,
        pump: bool,
        valve: bool,
        error: u8,
        flags: ArmFlags,
        pump_current: u8,
    },

    /// Request arm status (ID: 0x12T)
    RequestArmStatus { target: ArmTarget },

    /// Set torque enable (ID: 0x13T)
    SetTorque { target: ArmTarget, enable: bool },

    /// Set pump only (ID: 0x14T)
    SetPump { target: ArmTarget, enable: bool },

    /// Set valve only (ID: 0x15T)
    SetValve { target: ArmTarget, enable: bool },

    /// Set translation position (ID: 0x16M) - M is module index
    /// Controls the translation servo (shared bus for all 3 modules)
    SetTranslation {
        module: u8,
        position: u16,
        time_ms: u16,
    },

    /// Translation status report (ID: 0x17M)
    TranslationStatus {
        module: u8,
        position: u16,
        error: u8,
    },

    // ==================== GROUND Domain (0x2) ====================
    /// Ground sensors detection status (ID: 0x20S)
    GroundStatus {
        sensor: u8,
        detection_mask: u8,
    },

    /// Ground sensor value request/response (ID: 0x21S)
    GroundValue {
        sensor: u8,
        value: u16,
        threshold: u16,
    },

    /// Request ground sensor value (ID: 0x21S with no data)
    RequestGroundValue { sensor: u8 },

    /// Set ground sensor threshold (ID: 0x22S)
    SetGroundThreshold { sensor: u8, threshold: u16 },

    // ==================== LOG Domain (0x3) ====================
    /// Log level configuration (ID: 0x300)
    LogConfig { level: LogLevel },

    /// Log message start (ID: 0x310)
    LogMsg {
        seq: u8,
        level: LogLevel,
        payload: [u8; LOG_MSG_PAYLOAD_SIZE],
        payload_len: u8,
    },

    /// Log message continuation (ID: 0x311)
    LogCont {
        seq: u8,
        payload: [u8; LOG_CONT_PAYLOAD_SIZE],
        payload_len: u8,
    },

    /// Log message end (ID: 0x31F)
    LogEnd {
        seq: u8,
        total_len: u8,
        payload: [u8; LOG_END_PAYLOAD_SIZE],
        payload_len: u8,
    },

    // ==================== OTA Domain (0x4) ====================
    /// OTA start command (ID: 0x400)
    OtaStart { fw_size: u32, fw_crc32: u32 },

    /// OTA ready response (ID: 0x401)
    OtaReady { status: OtaReadyStatus },

    /// OTA data chunk (ID: 0x410)
    OtaData {
        chunk_idx: u16,
        data: [u8; OTA_CHUNK_SIZE],
        data_len: u8,
    },

    /// OTA acknowledge (ID: 0x411)
    OtaAck { chunk_idx: u16, status: u8 },

    /// OTA finish command (ID: 0x420)
    OtaFinish,

    /// OTA result response (ID: 0x421)
    OtaResult {
        status: OtaResultStatus,
        crc_computed: u32,
    },

    /// OTA abort command (ID: 0x4F0)
    OtaAbort,

    /// OTA reboot command (ID: 0x4F1)
    OtaReboot,
}

impl CanMessage {
    /// Build CAN ID from domain, command and target
    fn build_id(domain: Domain, cmd: u8, target: u8) -> StandardId {
        let id = ((domain as u16) << 8) | ((cmd as u16) << 4) | (target as u16);
        StandardId::new(id).expect("CAN ID exceeds 11-bit range")
    }

    /// Extract domain, command and target from raw ID
    fn parse_id(id: u16) -> (u8, u8, u8) {
        let domain = ((id >> 8) & 0x7) as u8;
        let cmd = ((id >> 4) & 0xF) as u8;
        let target = (id & 0xF) as u8;
        (domain, cmd, target)
    }

    /// Parse from raw CAN ID and data bytes
    pub fn parse(id: u16, data: &[u8]) -> Option<Self> {
        let (domain, cmd, target) = Self::parse_id(id);

        match Domain::from_u8(domain)? {
            Domain::System => Self::parse_system(cmd, data),
            Domain::Arm => Self::parse_arm(cmd, target, data),
            Domain::Ground => Self::parse_ground(cmd, target, data),
            Domain::Log => Self::parse_log(cmd, data),
            Domain::Ota => Self::parse_ota(cmd, data),
        }
    }

    /// Parse from an `embedded_can` frame
    pub fn from_frame<F: Frame>(frame: &F) -> Option<Self> {
        match frame.id() {
            Id::Standard(id) => Self::parse(id.as_raw(), frame.data()),
            _ => None
        }
    }

    /// Convert to an `embedded_can` frame
    pub fn to_frame<F: Frame>(&self) -> F {
        self.encode().to_frame()
    }

    fn parse_system(cmd: u8, data: &[u8]) -> Option<Self> {
        match SystemCmd::from_u8(cmd)? {
            SystemCmd::Ping => {
                if !data.is_empty() {
                    Some(CanMessage::Ping { value: data[0] })
                } else {
                    None
                }
            }
            SystemCmd::BoardInfo => {
                if data.is_empty() {
                    Some(CanMessage::BoardInfoRequest)
                } else if data.len() >= 8 {
                    Some(CanMessage::BoardInfoResponse {
                        fw_version: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
                        uptime_ms: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
                    })
                } else {
                    None
                }
            }
            SystemCmd::Error => {
                if data.len() >= 2 {
                    Some(CanMessage::SystemError {
                        error_type: ErrorType::from_u8(data[0])?,
                        error_code: data[1],
                    })
                } else {
                    None
                }
            }
            SystemCmd::SetServoId => {
                if data.len() >= 2 {
                    let bus = ServoBus::from_u8(data[0])?;
                    if data.len() >= 3 {
                        Some(CanMessage::SetServoIdResult {
                            bus,
                            new_id: data[1],
                            success: data[2] != 0,
                        })
                    } else {
                        Some(CanMessage::SetServoId {
                            bus,
                            new_id: data[1],
                        })
                    }
                } else {
                    None
                }
            }
            SystemCmd::Reboot => {
                if !data.is_empty() {
                    Some(CanMessage::Reboot {
                        mode: RebootMode::from_u8(data[0]),
                    })
                } else {
                    None
                }
            }
        }
    }

    fn parse_arm(cmd: u8, raw_target: u8, data: &[u8]) -> Option<Self> {
        let target = ArmTarget::from_u8(raw_target);
        match ArmCmd::from_u8(cmd)? {
            ArmCmd::SetArm => {
                if data.len() >= 6 {
                    Some(CanMessage::SetArm {
                        target,
                        position: u16::from_le_bytes([data[0], data[1]]),
                        time_ms: u16::from_le_bytes([data[2], data[3]]),
                        pump: data[4] != 0,
                        valve: data[5] != 0,
                    })
                } else {
                    None
                }
            }
            ArmCmd::Status => {
                if data.len() >= 7 {
                    Some(CanMessage::ArmStatus {
                        target,
                        position: u16::from_le_bytes([data[0], data[1]]),
                        color: Color::from_u8(data[2]),
                        pump: data[3] != 0,
                        valve: data[4] != 0,
                        error: data[5],
                        flags: ArmFlags::from_u8(data[6]),
                        pump_current: if data.len() >= 8 { data[7] } else { 0 },
                    })
                } else {
                    None
                }
            }
            ArmCmd::RequestStatus => {
                Some(CanMessage::RequestArmStatus { target })
            }
            ArmCmd::SetTorque => {
                if !data.is_empty() {
                    Some(CanMessage::SetTorque {
                        target,
                        enable: data[0] != 0,
                    })
                } else {
                    None
                }
            }
            ArmCmd::SetPump => {
                if !data.is_empty() {
                    Some(CanMessage::SetPump {
                        target,
                        enable: data[0] != 0,
                    })
                } else {
                    None
                }
            }
            ArmCmd::SetValve => {
                if !data.is_empty() {
                    Some(CanMessage::SetValve {
                        target,
                        enable: data[0] != 0,
                    })
                } else {
                    None
                }
            }
            ArmCmd::SetTranslation => {
                if data.len() >= 4 {
                    Some(CanMessage::SetTranslation {
                        module: raw_target,
                        position: u16::from_le_bytes([data[0], data[1]]),
                        time_ms: u16::from_le_bytes([data[2], data[3]]),
                    })
                } else {
                    None
                }
            }
            ArmCmd::TranslationStatus => {
                if data.len() >= 3 {
                    Some(CanMessage::TranslationStatus {
                        module: raw_target,
                        position: u16::from_le_bytes([data[0], data[1]]),
                        error: data[2],
                    })
                } else {
                    None
                }
            }
        }
    }

    fn parse_ground(cmd: u8, sensor: u8, data: &[u8]) -> Option<Self> {
        match GroundCmd::from_u8(cmd)? {
            GroundCmd::Status => {
                if !data.is_empty() {
                    Some(CanMessage::GroundStatus {
                        sensor,
                        detection_mask: data[0],
                    })
                } else {
                    None
                }
            }
            GroundCmd::Value => {
                if data.len() >= 4 {
                    Some(CanMessage::GroundValue {
                        sensor,
                        value: u16::from_le_bytes([data[0], data[1]]),
                        threshold: u16::from_le_bytes([data[2], data[3]]),
                    })
                } else {
                    Some(CanMessage::RequestGroundValue { sensor })
                }
            }
            GroundCmd::SetThreshold => {
                if data.len() >= 2 {
                    Some(CanMessage::SetGroundThreshold {
                        sensor,
                        threshold: u16::from_le_bytes([data[0], data[1]]),
                    })
                } else {
                    None
                }
            }
        }
    }

    fn parse_log(cmd: u8, data: &[u8]) -> Option<Self> {
        match LogCmd::from_u8(cmd)? {
            LogCmd::Config => {
                if !data.is_empty() {
                    Some(CanMessage::LogConfig {
                        level: LogLevel::from_u8(data[0]),
                    })
                } else {
                    None
                }
            }
            LogCmd::Msg => {
                if data.len() >= 2 {
                    let mut payload = [0u8; LOG_MSG_PAYLOAD_SIZE];
                    let payload_len = (data.len() - 2).min(payload.len()) as u8;
                    payload[..payload_len as usize].copy_from_slice(&data[2..2 + payload_len as usize]);
                    Some(CanMessage::LogMsg {
                        seq: data[0],
                        level: LogLevel::from_u8(data[1]),
                        payload,
                        payload_len,
                    })
                } else {
                    None
                }
            }
            LogCmd::Cont => {
                if !data.is_empty() {
                    let mut payload = [0u8; LOG_CONT_PAYLOAD_SIZE];
                    let payload_len = (data.len() - 1).min(payload.len()) as u8;
                    payload[..payload_len as usize].copy_from_slice(&data[1..1 + payload_len as usize]);
                    Some(CanMessage::LogCont {
                        seq: data[0],
                        payload,
                        payload_len,
                    })
                } else {
                    None
                }
            }
            LogCmd::End => {
                if data.len() >= 2 {
                    let mut payload = [0u8; LOG_END_PAYLOAD_SIZE];
                    let payload_len = (data.len() - 2).min(payload.len()) as u8;
                    payload[..payload_len as usize].copy_from_slice(&data[2..2 + payload_len as usize]);
                    Some(CanMessage::LogEnd {
                        seq: data[0],
                        total_len: data[1],
                        payload,
                        payload_len,
                    })
                } else {
                    None
                }
            }
        }
    }

    fn parse_ota(cmd: u8, data: &[u8]) -> Option<Self> {
        match OtaCmd::from_u8(cmd)? {
            OtaCmd::StartReady => {
                if data.len() >= 8 {
                    Some(CanMessage::OtaStart {
                        fw_size: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
                        fw_crc32: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
                    })
                } else if !data.is_empty() {
                    Some(CanMessage::OtaReady {
                        status: OtaReadyStatus::from_u8(data[0]),
                    })
                } else {
                    None
                }
            }
            OtaCmd::DataAck => {
                if data.len() >= 3 {
                    let chunk_idx = u16::from_le_bytes([data[0], data[1]]);
                    if data.len() > 3 {
                        let mut chunk_data = [0u8; OTA_CHUNK_SIZE];
                        let data_len = (data.len() - 2).min(chunk_data.len()) as u8;
                        chunk_data[..data_len as usize].copy_from_slice(&data[2..2 + data_len as usize]);
                        Some(CanMessage::OtaData {
                            chunk_idx,
                            data: chunk_data,
                            data_len,
                        })
                    } else {
                        Some(CanMessage::OtaAck {
                            chunk_idx,
                            status: data[2],
                        })
                    }
                } else {
                    None
                }
            }
            OtaCmd::FinishResult => {
                if data.len() >= 5 {
                    Some(CanMessage::OtaResult {
                        status: OtaResultStatus::from_u8(data[0]),
                        crc_computed: u32::from_le_bytes([data[1], data[2], data[3], data[4]]),
                    })
                } else {
                    Some(CanMessage::OtaFinish)
                }
            }
            OtaCmd::AbortReboot => {
                if data.is_empty() {
                    Some(CanMessage::OtaAbort)
                } else {
                    Some(CanMessage::OtaReboot)
                }
            }
        }
    }

    /// Encode message to raw ID and data
    pub fn encode(&self) -> EncodedMessage {
        match self {
            // ==================== SYSTEM ====================
            CanMessage::Ping { value } => {
                let id = Self::build_id(Domain::System, SystemCmd::Ping as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *value;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::BoardInfoRequest => {
                let id = Self::build_id(Domain::System, SystemCmd::BoardInfo as u8, 0);
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
            CanMessage::BoardInfoResponse {
                fw_version,
                uptime_ms,
            } => {
                let id = Self::build_id(Domain::System, SystemCmd::BoardInfo as u8, 1);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..4].copy_from_slice(&fw_version.to_le_bytes());
                data[4..8].copy_from_slice(&uptime_ms.to_le_bytes());
                EncodedMessage { id, data, len: 8 }
            }
            CanMessage::SystemError {
                error_type,
                error_code,
            } => {
                let id = Self::build_id(Domain::System, SystemCmd::Error as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *error_type as u8;
                data[1] = *error_code;
                EncodedMessage { id, data, len: 2 }
            }
            CanMessage::Reboot { mode } => {
                let id = Self::build_id(Domain::System, SystemCmd::Reboot as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *mode as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::SetServoId { bus, new_id } => {
                let id = Self::build_id(Domain::System, SystemCmd::SetServoId as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *bus as u8;
                data[1] = *new_id;
                EncodedMessage { id, data, len: 2 }
            }
            CanMessage::SetServoIdResult { bus, new_id, success } => {
                let id = Self::build_id(Domain::System, SystemCmd::SetServoId as u8, 1);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *bus as u8;
                data[1] = *new_id;
                data[2] = *success as u8;
                EncodedMessage { id, data, len: 3 }
            }

            // ==================== ARM ====================
            CanMessage::SetArm {
                target,
                position,
                time_ms,
                pump,
                valve,
            } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::SetArm as u8, target.to_u8());
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&position.to_le_bytes());
                data[2..4].copy_from_slice(&time_ms.to_le_bytes());
                data[4] = *pump as u8;
                data[5] = *valve as u8;
                EncodedMessage { id, data, len: 6 }
            }
            CanMessage::ArmStatus {
                target,
                position,
                color,
                pump,
                valve,
                error,
                flags,
                pump_current,
            } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::Status as u8, target.to_u8());
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&position.to_le_bytes());
                data[2] = *color as u8;
                data[3] = *pump as u8;
                data[4] = *valve as u8;
                data[5] = *error;
                data[6] = flags.to_u8();
                data[7] = *pump_current;
                EncodedMessage { id, data, len: 8 }
            }
            CanMessage::RequestArmStatus { target } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::RequestStatus as u8, target.to_u8());
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
            CanMessage::SetTorque { target, enable } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::SetTorque as u8, target.to_u8());
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *enable as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::SetPump { target, enable } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::SetPump as u8, target.to_u8());
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *enable as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::SetValve { target, enable } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::SetValve as u8, target.to_u8());
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *enable as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::SetTranslation { module, position, time_ms } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::SetTranslation as u8, *module);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&position.to_le_bytes());
                data[2..4].copy_from_slice(&time_ms.to_le_bytes());
                EncodedMessage { id, data, len: 4 }
            }
            CanMessage::TranslationStatus { module, position, error } => {
                let id = Self::build_id(Domain::Arm, ArmCmd::TranslationStatus as u8, *module);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&position.to_le_bytes());
                data[2] = *error;
                EncodedMessage { id, data, len: 3 }
            }

            // ==================== GROUND ====================
            CanMessage::GroundStatus {
                sensor,
                detection_mask,
            } => {
                let id = Self::build_id(Domain::Ground, GroundCmd::Status as u8, *sensor);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *detection_mask;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::GroundValue {
                sensor,
                value,
                threshold,
            } => {
                let id = Self::build_id(Domain::Ground, GroundCmd::Value as u8, *sensor);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&value.to_le_bytes());
                data[2..4].copy_from_slice(&threshold.to_le_bytes());
                EncodedMessage { id, data, len: 4 }
            }
            CanMessage::RequestGroundValue { sensor } => {
                let id = Self::build_id(Domain::Ground, GroundCmd::Value as u8, *sensor);
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
            CanMessage::SetGroundThreshold { sensor, threshold } => {
                let id = Self::build_id(Domain::Ground, GroundCmd::SetThreshold as u8, *sensor);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&threshold.to_le_bytes());
                EncodedMessage { id, data, len: 2 }
            }

            // ==================== LOG ====================
            CanMessage::LogConfig { level } => {
                let id = Self::build_id(Domain::Log, LogCmd::Config as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *level as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::LogMsg {
                seq,
                level,
                payload,
                payload_len,
            } => {
                let id = Self::build_id(Domain::Log, LogCmd::Msg as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *seq;
                data[1] = *level as u8;
                let len = (*payload_len as usize).min(6);
                data[2..2 + len].copy_from_slice(&payload[..len]);
                EncodedMessage {
                    id,
                    data,
                    len: 2 + len,
                }
            }
            CanMessage::LogCont {
                seq,
                payload,
                payload_len,
            } => {
                let id = Self::build_id(Domain::Log, LogCmd::Cont as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *seq;
                let len = (*payload_len as usize).min(7);
                data[1..1 + len].copy_from_slice(&payload[..len]);
                EncodedMessage {
                    id,
                    data,
                    len: 1 + len,
                }
            }
            CanMessage::LogEnd {
                seq,
                total_len,
                payload,
                payload_len,
            } => {
                let id = Self::build_id(Domain::Log, LogCmd::End as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *seq;
                data[1] = *total_len;
                let len = (*payload_len as usize).min(6);
                data[2..2 + len].copy_from_slice(&payload[..len]);
                EncodedMessage {
                    id,
                    data,
                    len: 2 + len,
                }
            }

            // ==================== OTA ====================
            CanMessage::OtaStart { fw_size, fw_crc32 } => {
                let id = Self::build_id(Domain::Ota, OtaCmd::StartReady as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..4].copy_from_slice(&fw_size.to_le_bytes());
                data[4..8].copy_from_slice(&fw_crc32.to_le_bytes());
                EncodedMessage { id, data, len: 8 }
            }
            CanMessage::OtaReady { status } => {
                let id = Self::build_id(Domain::Ota, OtaCmd::StartReady as u8, 1);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *status as u8;
                EncodedMessage { id, data, len: 1 }
            }
            CanMessage::OtaData {
                chunk_idx,
                data: chunk_data,
                data_len,
            } => {
                let id = Self::build_id(Domain::Ota, OtaCmd::DataAck as u8, 0);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&chunk_idx.to_le_bytes());
                let len = (*data_len as usize).min(6);
                data[2..2 + len].copy_from_slice(&chunk_data[..len]);
                EncodedMessage {
                    id,
                    data,
                    len: 2 + len,
                }
            }
            CanMessage::OtaAck { chunk_idx, status } => {
                let id = Self::build_id(Domain::Ota, OtaCmd::DataAck as u8, 1);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0..2].copy_from_slice(&chunk_idx.to_le_bytes());
                data[2] = *status;
                EncodedMessage { id, data, len: 3 }
            }
            CanMessage::OtaFinish => {
                let id = Self::build_id(Domain::Ota, OtaCmd::FinishResult as u8, 0);
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
            CanMessage::OtaResult {
                status,
                crc_computed,
            } => {
                let id = Self::build_id(Domain::Ota, OtaCmd::FinishResult as u8, 1);
                let mut data = [0u8; MAX_DATA_LEN];
                data[0] = *status as u8;
                data[1..5].copy_from_slice(&crc_computed.to_le_bytes());
                EncodedMessage { id, data, len: 5 }
            }
            CanMessage::OtaAbort => {
                let id = Self::build_id(Domain::Ota, OtaCmd::AbortReboot as u8, 0);
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
            CanMessage::OtaReboot => {
                let id = Self::build_id(Domain::Ota, OtaCmd::AbortReboot as u8, 1);
                EncodedMessage {
                    id,
                    data: [0u8; MAX_DATA_LEN],
                    len: 0,
                }
            }
        }
    }

    /// Get the domain of this message
    pub fn domain(&self) -> Domain {
        match self {
            CanMessage::Ping { .. }
            | CanMessage::BoardInfoRequest
            | CanMessage::BoardInfoResponse { .. }
            | CanMessage::SystemError { .. }
            | CanMessage::Reboot { .. }
            | CanMessage::SetServoId { .. }
            | CanMessage::SetServoIdResult { .. } => Domain::System,

            CanMessage::SetArm { .. }
            | CanMessage::ArmStatus { .. }
            | CanMessage::RequestArmStatus { .. }
            | CanMessage::SetTorque { .. }
            | CanMessage::SetPump { .. }
            | CanMessage::SetValve { .. }
            | CanMessage::SetTranslation { .. }
            | CanMessage::TranslationStatus { .. } => Domain::Arm,

            CanMessage::GroundStatus { .. }
            | CanMessage::GroundValue { .. }
            | CanMessage::RequestGroundValue { .. }
            | CanMessage::SetGroundThreshold { .. } => Domain::Ground,

            CanMessage::LogConfig { .. }
            | CanMessage::LogMsg { .. }
            | CanMessage::LogCont { .. }
            | CanMessage::LogEnd { .. } => Domain::Log,

            CanMessage::OtaStart { .. }
            | CanMessage::OtaReady { .. }
            | CanMessage::OtaData { .. }
            | CanMessage::OtaAck { .. }
            | CanMessage::OtaFinish
            | CanMessage::OtaResult { .. }
            | CanMessage::OtaAbort
            | CanMessage::OtaReboot => Domain::Ota,
        }
    }

    /// Get arm target if applicable
    pub fn arm_target(&self) -> Option<ArmTarget> {
        match self {
            CanMessage::SetArm { target, .. }
            | CanMessage::ArmStatus { target, .. }
            | CanMessage::RequestArmStatus { target }
            | CanMessage::SetTorque { target, .. }
            | CanMessage::SetPump { target, .. }
            | CanMessage::SetValve { target, .. } => Some(*target),
            _ => None,
        }
    }
}


/// Build a ping response message
pub fn ping_response(value: u8) -> CanMessage {
    CanMessage::Ping {
        value: value.wrapping_add(1),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::Id;

    /// Helper: encode then parse, assert roundtrip equality
    fn roundtrip(msg: &CanMessage) {
        let encoded = msg.encode();
        let parsed = CanMessage::parse(encoded.id.as_raw(), &encoded.data[..encoded.len]);
        assert_eq!(parsed.as_ref(), Some(msg), "roundtrip failed for {:?}", msg);
    }

    // ==================== SYSTEM ====================

    #[test]
    fn roundtrip_ping() {
        roundtrip(&CanMessage::Ping { value: 0 });
        roundtrip(&CanMessage::Ping { value: 42 });
        roundtrip(&CanMessage::Ping { value: 255 });
    }

    #[test]
    fn roundtrip_board_info_response() {
        roundtrip(&CanMessage::BoardInfoResponse {
            fw_version: 0x01020304,
            uptime_ms: 123456,
        });
    }

    #[test]
    fn roundtrip_system_error() {
        roundtrip(&CanMessage::SystemError {
            error_type: ErrorType::ServoCommunication,
            error_code: 0x42,
        });
        roundtrip(&CanMessage::SystemError {
            error_type: ErrorType::OtaError,
            error_code: 0,
        });
    }

    #[test]
    fn roundtrip_reboot() {
        roundtrip(&CanMessage::Reboot { mode: RebootMode::Normal });
        roundtrip(&CanMessage::Reboot { mode: RebootMode::Bootloader });
    }

    #[test]
    fn roundtrip_set_servo_id() {
        roundtrip(&CanMessage::SetServoId {
            bus: ServoBus::Module0,
            new_id: 5,
        });
        roundtrip(&CanMessage::SetServoIdResult {
            bus: ServoBus::Translation,
            new_id: 3,
            success: true,
        });
    }

    // ==================== ARM ====================

    #[test]
    fn roundtrip_set_arm_all_modules() {
        // All 3 modules × 4 arms roundtrip correctly with flat encoding
        for module in 0..3u8 {
            for arm in 0..4u8 {
                roundtrip(&CanMessage::SetArm {
                    target: ArmTarget::new(module, arm),
                    position: 512,
                    time_ms: 1000,
                    pump: true,
                    valve: false,
                });
            }
        }
    }

    #[test]
    fn roundtrip_arm_status() {
        roundtrip(&CanMessage::ArmStatus {
            target: ArmTarget::new(2, 3),
            position: 1024,
            color: Color::Yellow,
            pump: false,
            valve: true,
            error: 0,
            flags: ArmFlags {
                torque_enabled: true,
                moving: false,
                position_reached: true,
            },
            pump_current: 128,
        });
    }

    #[test]
    fn roundtrip_arm_commands() {
        roundtrip(&CanMessage::SetTorque {
            target: ArmTarget::new(1, 3),
            enable: true,
        });
        roundtrip(&CanMessage::SetPump {
            target: ArmTarget::new(2, 1),
            enable: true,
        });
        roundtrip(&CanMessage::SetValve {
            target: ArmTarget::new(0, 0),
            enable: false,
        });
        roundtrip(&CanMessage::RequestArmStatus {
            target: ArmTarget::new(1, 2),
        });
    }

    #[test]
    fn roundtrip_arm_broadcast() {
        roundtrip(&CanMessage::SetTorque {
            target: ArmTarget::BROADCAST_ALL,
            enable: true,
        });
    }

    #[test]
    fn arm_target_flat_encoding() {
        // Verify flat encoding: target = module * 4 + arm
        assert_eq!(ArmTarget::new(0, 0).to_u8(), 0);
        assert_eq!(ArmTarget::new(0, 3).to_u8(), 3);
        assert_eq!(ArmTarget::new(1, 0).to_u8(), 4);
        assert_eq!(ArmTarget::new(1, 2).to_u8(), 6);
        assert_eq!(ArmTarget::new(2, 3).to_u8(), 11);
        assert_eq!(ArmTarget::BROADCAST_ALL.to_u8(), 0xF);

        // Verify roundtrip
        for module in 0..3u8 {
            for arm in 0..4u8 {
                let t = ArmTarget::new(module, arm);
                let decoded = ArmTarget::from_u8(t.to_u8());
                assert_eq!(decoded, t);
            }
        }
    }

    #[test]
    fn roundtrip_translation_all_modules() {
        for module in 0..3u8 {
            roundtrip(&CanMessage::SetTranslation {
                module,
                position: 2048,
                time_ms: 500,
            });
            roundtrip(&CanMessage::TranslationStatus {
                module,
                position: 4000,
                error: 0,
            });
        }
    }

    // ==================== GROUND ====================

    #[test]
    fn roundtrip_ground_status() {
        roundtrip(&CanMessage::GroundStatus {
            sensor: 1,
            detection_mask: 0b101,
        });
    }

    #[test]
    fn roundtrip_ground_value() {
        roundtrip(&CanMessage::GroundValue {
            sensor: 2,
            value: 1234,
            threshold: 500,
        });
    }

    #[test]
    fn roundtrip_set_ground_threshold() {
        roundtrip(&CanMessage::SetGroundThreshold {
            sensor: 0,
            threshold: 800,
        });
    }

    // ==================== LOG ====================

    #[test]
    fn roundtrip_log_config() {
        roundtrip(&CanMessage::LogConfig { level: LogLevel::Debug });
    }

    #[test]
    fn roundtrip_log_msg() {
        roundtrip(&CanMessage::LogMsg {
            seq: 5,
            level: LogLevel::Warn,
            payload: [b'H', b'e', b'l', b'l', b'o', 0],
            payload_len: 5,
        });
    }

    #[test]
    fn roundtrip_log_cont() {
        roundtrip(&CanMessage::LogCont {
            seq: 5,
            payload: [b'W', b'o', b'r', b'l', b'd', b'!', 0],
            payload_len: 6,
        });
    }

    #[test]
    fn roundtrip_log_end() {
        roundtrip(&CanMessage::LogEnd {
            seq: 5,
            total_len: 11,
            payload: [b'!', 0, 0, 0, 0, 0],
            payload_len: 1,
        });
    }

    // ==================== OTA ====================

    #[test]
    fn roundtrip_ota_start() {
        roundtrip(&CanMessage::OtaStart {
            fw_size: 65536,
            fw_crc32: 0xDEADBEEF,
        });
    }

    #[test]
    fn roundtrip_ota_data() {
        roundtrip(&CanMessage::OtaData {
            chunk_idx: 42,
            data: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            data_len: 6,
        });
    }

    #[test]
    fn roundtrip_ota_ack() {
        roundtrip(&CanMessage::OtaAck {
            chunk_idx: 42,
            status: 0,
        });
    }

    #[test]
    fn roundtrip_ota_result() {
        roundtrip(&CanMessage::OtaResult {
            status: OtaResultStatus::Ok,
            crc_computed: 0xDEADBEEF,
        });
        roundtrip(&CanMessage::OtaResult {
            status: OtaResultStatus::CrcError,
            crc_computed: 0x12345678,
        });
    }

    #[test]
    fn roundtrip_ota_control() {
        roundtrip(&CanMessage::OtaFinish);
    }

    // ==================== Domain ====================

    #[test]
    fn domain_classification() {
        assert_eq!(CanMessage::Ping { value: 0 }.domain(), Domain::System);
        assert_eq!(CanMessage::SetArm {
            target: ArmTarget::new(0, 0),
            position: 0, time_ms: 0, pump: false, valve: false,
        }.domain(), Domain::Arm);
        assert_eq!(CanMessage::GroundStatus { sensor: 0, detection_mask: 0 }.domain(), Domain::Ground);
        assert_eq!(CanMessage::LogConfig { level: LogLevel::Off }.domain(), Domain::Log);
        assert_eq!(CanMessage::OtaFinish.domain(), Domain::Ota);
    }

    #[test]
    fn ping_response_wraps() {
        let resp = ping_response(254);
        assert_eq!(resp, CanMessage::Ping { value: 255 });
        let resp = ping_response(255);
        assert_eq!(resp, CanMessage::Ping { value: 0 });
    }

    #[test]
    fn parse_invalid_domain_returns_none() {
        assert!(CanMessage::parse(0x700, &[0]).is_none()); // domain 7
    }

    #[test]
    fn parse_invalid_cmd_returns_none() {
        // System domain, cmd=0xA (not defined)
        assert!(CanMessage::parse(0x0A0, &[0]).is_none());
    }
}
