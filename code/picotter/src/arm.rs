//! Single arm management
//!
//! An arm consists of:
//! - 1 servo (SCS0009)
//! - 1 pump (via IO expander)
//! - 1 valve (via IO expander)
//! - 1 color sensor

use crate::can_protocol::{ArmFlags, ArmTarget, CanMessage};

/// Arm state
#[derive(Debug, Clone, Default)]
pub struct ArmState {
    /// Current servo position (0-1023)
    pub position: u16,
    /// Target servo position
    pub target_position: u16,
    /// Color: bit 7 = detected (dC > threshold), bits 0-6 = hue (0-127, always computed)
    pub color: u8,
    /// Pump state
    pub pump: bool,
    /// Valve state
    pub valve: bool,
    /// Servo error code (0 = OK)
    pub servo_error: u8,
    /// Torque enabled
    pub torque_enabled: bool,
    /// Servo is moving
    pub moving: bool,
    /// Position reached
    pub position_reached: bool,
    /// Pump current (ADC raw value, 12-bit)
    pub pump_current: u16,
}

impl ArmState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Get flags for CAN status message
    pub fn flags(&self) -> ArmFlags {
        ArmFlags {
            torque_enabled: self.torque_enabled,
            moving: self.moving,
            position_reached: self.position_reached,
        }
    }

    /// Build status CAN message
    pub fn to_status_message(&self, module: u8, arm: u8) -> CanMessage {
        CanMessage::ArmStatus {
            target: ArmTarget::new(module, arm),
            position: self.position,
            color: self.color,
            pump: self.pump,
            valve: self.valve,
            error: self.servo_error,
            flags: self.flags(),
            // Scale 12-bit ADC to 8-bit
            pump_current: (self.pump_current >> 4) as u8,
        }
    }

    /// Check if position is reached (within tolerance)
    pub fn check_position_reached(&mut self, tolerance: u16) {
        let diff = if self.position > self.target_position {
            self.position - self.target_position
        } else {
            self.target_position - self.position
        };
        self.position_reached = diff <= tolerance;
        self.moving = !self.position_reached;
    }
}

/// Arm command to be executed
#[derive(Debug, Clone)]
pub enum ArmCommand {
    /// Set position, pump and valve
    SetAll {
        position: u16,
        time_ms: u16,
        pump: bool,
        valve: bool,
    },
    /// Set torque enable
    SetTorque(bool),
    /// Set pump only
    SetPump(bool),
    /// Set valve only
    SetValve(bool),
}

/// Arm errors
#[derive(Debug, Clone, Copy)]
pub enum ArmError {
    /// Servo communication timeout
    ServoTimeout,
    /// Servo reported error
    ServoError(u8),
    /// I2C communication error
    I2cError,
    /// Color sensor error
    ColorSensorError,
}

impl ArmError {
    /// Convert to CAN error type code
    pub fn to_error_type(&self) -> u8 {
        match self {
            ArmError::ServoTimeout | ArmError::ServoError(_) => 0x01,
            ArmError::I2cError | ArmError::ColorSensorError => 0x02,
        }
    }

    /// Convert to CAN error detail code
    pub fn to_error_code(&self) -> u8 {
        match self {
            ArmError::ServoTimeout => 0x01,
            ArmError::ServoError(e) => *e,
            ArmError::I2cError => 0x01,
            ArmError::ColorSensorError => 0x02,
        }
    }
}
