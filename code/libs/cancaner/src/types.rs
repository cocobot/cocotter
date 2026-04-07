//! Core types for CAN protocol

/// Protocol domain (3 bits in ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Domain {
    System = 0x0,
    Arm = 0x1,
    Ground = 0x2,
    Log = 0x3,
    Ota = 0x4,
}

impl Domain {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(Domain::System),
            0x1 => Some(Domain::Arm),
            0x2 => Some(Domain::Ground),
            0x3 => Some(Domain::Log),
            0x4 => Some(Domain::Ota),
            _ => None,
        }
    }
}

/// Error types for system ERROR message
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorType {
    ServoCommunication = 0x01,
    I2cError = 0x02,
    InvalidCommand = 0x03,
    CanOverflow = 0x04,
    OtaError = 0x10,
}

impl ErrorType {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x01 => Some(ErrorType::ServoCommunication),
            0x02 => Some(ErrorType::I2cError),
            0x03 => Some(ErrorType::InvalidCommand),
            0x04 => Some(ErrorType::CanOverflow),
            0x10 => Some(ErrorType::OtaError),
            _ => None,
        }
    }
}

/// Arm status flags
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ArmFlags {
    pub torque_enabled: bool,
    pub moving: bool,
    pub position_reached: bool,
}

impl ArmFlags {
    pub fn to_u8(self) -> u8 {
        let mut flags = 0u8;
        if self.torque_enabled {
            flags |= 0x01;
        }
        if self.moving {
            flags |= 0x02;
        }
        if self.position_reached {
            flags |= 0x04;
        }
        flags
    }

    pub fn from_u8(val: u8) -> Self {
        Self {
            torque_enabled: (val & 0x01) != 0,
            moving: (val & 0x02) != 0,
            position_reached: (val & 0x04) != 0,
        }
    }
}

/// Target specification for ARM domain (module/arm with broadcast support)
///
/// Encoded as a flat index in the 4-bit CAN ID target field:
/// - target = module * ARMS_PER_MODULE + arm (0-11 for 3 modules × 4 arms)
/// - target = 0xF for broadcast to all
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmTarget {
    /// Module index: 0-2 or 0xF for broadcast
    pub module: u8,
    /// Arm index: 0-3 or 0xF for broadcast
    pub arm: u8,
}

impl ArmTarget {
    const ARMS_PER_MODULE: u8 = crate::protocol::ARMS_PER_MODULE as u8;

    /// Broadcast to all modules and arms
    pub const BROADCAST_ALL: Self = Self {
        module: 0xF,
        arm: 0xF,
    };

    pub fn new(module: u8, arm: u8) -> Self {
        Self { module, arm }
    }

    /// Encode to 4-bit CAN ID target field (flat index)
    pub fn to_u8(self) -> u8 {
        if self.module == 0xF || self.arm == 0xF {
            0xF
        } else {
            self.module * Self::ARMS_PER_MODULE + self.arm
        }
    }

    /// Decode from 4-bit CAN ID target field (flat index)
    pub fn from_u8(val: u8) -> Self {
        if val == 0xF {
            Self::BROADCAST_ALL
        } else {
            Self {
                module: val / Self::ARMS_PER_MODULE,
                arm: val % Self::ARMS_PER_MODULE,
            }
        }
    }

    /// Check if this target matches a specific module/arm
    pub fn matches(&self, module: u8, arm: u8) -> bool {
        let module_match = self.module == 0xF || self.module == module;
        let arm_match = self.arm == 0xF || self.arm == arm;
        module_match && arm_match
    }

     pub fn match_module(&self, module: u8) -> bool {
        self.module == 0xF || self.module == module
    }

    /// Check if broadcast to all arms
    pub fn is_arm_broadcast(&self) -> bool {
        self.arm == 0xF
    }

    /// Check if broadcast to all modules
    pub fn is_module_broadcast(&self) -> bool {
        self.module == 0xF
    }
}

/// Target specification for stage2 servos (module/servo with broadcast support)
///
/// Encoded as a flat index in the 4-bit CAN ID target field:
/// - target = module * STAGE2_SERVOS_PER_MODULE + servo (0-5 for 3 modules × 2 servos)
/// - target = 0xF for broadcast to all
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stage2Target {
    /// Module index: 0-2 or 0xF for broadcast
    pub module: u8,
    /// Servo index: 0-1 or 0xF for broadcast
    pub servo: u8,
}

impl Stage2Target {
    const SERVOS_PER_MODULE: u8 = crate::protocol::STAGE2_SERVOS_PER_MODULE as u8;

    /// Broadcast to all modules and servos
    pub const BROADCAST_ALL: Self = Self {
        module: 0xF,
        servo: 0xF,
    };

    pub fn new(module: u8, servo: u8) -> Self {
        Self { module, servo }
    }

    /// Encode to 4-bit CAN ID target field (flat index)
    pub fn to_u8(self) -> u8 {
        if self.module == 0xF || self.servo == 0xF {
            0xF
        } else {
            self.module * Self::SERVOS_PER_MODULE + self.servo
        }
    }

    /// Decode from 4-bit CAN ID target field (flat index)
    pub fn from_u8(val: u8) -> Self {
        if val == 0xF {
            Self::BROADCAST_ALL
        } else {
            Self {
                module: val / Self::SERVOS_PER_MODULE,
                servo: val % Self::SERVOS_PER_MODULE,
            }
        }
    }

    /// Check if this target matches a specific module/servo
    pub fn matches(&self, module: u8, servo: u8) -> bool {
        let module_match = self.module == 0xF || self.module == module;
        let servo_match = self.servo == 0xF || self.servo == servo;
        module_match && servo_match
    }

    pub fn match_module(&self, module: u8) -> bool {
        self.module == 0xF || self.module == module
    }

    /// Check if broadcast to all servos
    pub fn is_servo_broadcast(&self) -> bool {
        self.servo == 0xF
    }

    /// Check if broadcast to all modules
    pub fn is_module_broadcast(&self) -> bool {
        self.module == 0xF
    }
}

/// Log level (matches Rust log crate levels)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum LogLevel {
    Off = 0,
    Error = 1,
    Warn = 2,
    Info = 3,
    Debug = 4,
    Trace = 5,
}

impl LogLevel {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => LogLevel::Off,
            1 => LogLevel::Error,
            2 => LogLevel::Warn,
            3 => LogLevel::Info,
            4 => LogLevel::Debug,
            5 => LogLevel::Trace,
            _ => LogLevel::Trace,
        }
    }
}

/// OTA ready status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OtaReadyStatus {
    Ok = 0,
    Busy = 1,
    NoSpace = 2,
}

impl OtaReadyStatus {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => OtaReadyStatus::Ok,
            1 => OtaReadyStatus::Busy,
            2 => OtaReadyStatus::NoSpace,
            _ => OtaReadyStatus::Busy,
        }
    }
}

/// OTA result status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OtaResultStatus {
    Ok = 0,
    CrcError = 1,
    SizeMismatch = 2,
}

impl OtaResultStatus {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => OtaResultStatus::Ok,
            1 => OtaResultStatus::CrcError,
            2 => OtaResultStatus::SizeMismatch,
            _ => OtaResultStatus::CrcError,
        }
    }
}

/// Reboot mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RebootMode {
    Normal = 0,
    Bootloader = 1,
}

impl RebootMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            0 => RebootMode::Normal,
            1 => RebootMode::Bootloader,
            _ => RebootMode::Normal,
        }
    }
}

/// Servo bus identifier for SetServoId command
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ServoBus {
    /// Module 0 servo bus (4 arm servos)
    Module0 = 0,
    /// Module 1 servo bus (4 arm servos)
    Module1 = 1,
    /// Module 2 servo bus (4 arm servos)
    Module2 = 2,
    /// Translation bus (3 servos, one per module)
    Translation = 3,
}

impl ServoBus {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(ServoBus::Module0),
            1 => Some(ServoBus::Module1),
            2 => Some(ServoBus::Module2),
            3 => Some(ServoBus::Translation),
            _ => None,
        }
    }
}

// ==================== Command enums per domain ====================

/// System domain commands (4 bits in CAN ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SystemCmd {
    Ping = 0x0,
    BoardInfo = 0x1,
    SetServoId = 0x2,
    ScanBus = 0x3,
    Error = 0xE,
    Reboot = 0xF,
}

impl SystemCmd {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(SystemCmd::Ping),
            0x1 => Some(SystemCmd::BoardInfo),
            0x2 => Some(SystemCmd::SetServoId),
            0x3 => Some(SystemCmd::ScanBus),
            0xE => Some(SystemCmd::Error),
            0xF => Some(SystemCmd::Reboot),
            _ => None,
        }
    }
}

/// Arm domain commands (4 bits in CAN ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ArmCmd {
    SetArm = 0x0,
    Status = 0x1,
    RequestStatus = 0x2,
    SetTorque = 0x3,
    SetPump = 0x4,
    SetValve = 0x5,
    SetTranslation = 0x6,
    TranslationStatus = 0x7,
    SetColorConfig = 0x8,
    SetColorSensorConfig = 0x9,
    ColorSensorRaw = 0xA,
    SetColorLedPwm = 0xB,
    RequestTranslationStatus = 0xC,
    SetStage2 = 0xD,
    SetStage2Torque = 0xE,
    Stage2Status = 0xF,
}

impl ArmCmd {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(ArmCmd::SetArm),
            0x1 => Some(ArmCmd::Status),
            0x2 => Some(ArmCmd::RequestStatus),
            0x3 => Some(ArmCmd::SetTorque),
            0x4 => Some(ArmCmd::SetPump),
            0x5 => Some(ArmCmd::SetValve),
            0x6 => Some(ArmCmd::SetTranslation),
            0x7 => Some(ArmCmd::TranslationStatus),
            0x8 => Some(ArmCmd::SetColorConfig),
            0x9 => Some(ArmCmd::SetColorSensorConfig),
            0xA => Some(ArmCmd::ColorSensorRaw),
            0xB => Some(ArmCmd::SetColorLedPwm),
            0xC => Some(ArmCmd::RequestTranslationStatus),
            0xD => Some(ArmCmd::SetStage2),
            0xE => Some(ArmCmd::SetStage2Torque),
            0xF => Some(ArmCmd::Stage2Status),
            _ => None,
        }
    }
}

/// Ground domain commands (4 bits in CAN ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GroundCmd {
    Status = 0x0,
    Value = 0x1,
    SetThreshold = 0x2,
}

impl GroundCmd {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(GroundCmd::Status),
            0x1 => Some(GroundCmd::Value),
            0x2 => Some(GroundCmd::SetThreshold),
            _ => None,
        }
    }
}

/// Log domain commands (4 bits in CAN ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LogCmd {
    Config = 0x0,
    Msg = 0x1,
    Cont = 0x2,
    End = 0xF,
}

impl LogCmd {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(LogCmd::Config),
            0x1 => Some(LogCmd::Msg),
            0x2 => Some(LogCmd::Cont),
            0xF => Some(LogCmd::End),
            _ => None,
        }
    }
}

/// OTA domain commands (4 bits in CAN ID)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OtaCmd {
    StartReady = 0x0,
    DataAck = 0x1,
    FinishResult = 0x2,
    AbortReboot = 0xF,
}

impl OtaCmd {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x0 => Some(OtaCmd::StartReady),
            0x1 => Some(OtaCmd::DataAck),
            0x2 => Some(OtaCmd::FinishResult),
            0xF => Some(OtaCmd::AbortReboot),
            _ => None,
        }
    }
}
