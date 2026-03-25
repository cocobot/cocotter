//! Module management (4 arms per module)
//!
//! A module consists of:
//! - 4 arms (servo + pump + valve + color sensor each)
//! - 1 shared UART bus for all 4 servos
//! - 1 shared I2C bus for IO expander (pumps/valves) and ground sensor
//!
//! Pin mapping on PCA9535:
//! - Bank 0, pins 0-3: Valves 0-3
//! - Bank 0, pin 7: LED
//! - Bank 1, pins 0-3: Pumps 0-3

use crate::arm::{ArmCommand, ArmError, ArmState};
use crate::i2c_devices::{GPIOBank, I2cDevices};
use crate::scs0009::{Scs0009, ScsError};
use embedded_hal_async::i2c::I2c;
use embedded_io_async::{Read, Write};
use rtt_target::rprintln;
use cancaner::{ArmTarget, CanMessage, Color};

/// Number of arms per module
pub const ARMS_PER_MODULE: usize = 4;

/// Pin assignments for valves (Bank 0, pins 0-3)
const VALVE_BANK: GPIOBank = GPIOBank::Bank0;
const VALVE_PINS: [u8; 4] = [0, 1, 2, 3];

/// Pin assignment for LED (Bank 0, pin 7)
const LED_BANK: GPIOBank = GPIOBank::Bank0;
const LED_PIN: u8 = 7;

/// Pin assignments for pumps (Bank 1, pins 0-3)
const PUMP_BANK: GPIOBank = GPIOBank::Bank1;
const PUMP_PINS: [u8; 4] = [0, 1, 2, 3];

/// Module state containing all 4 arms
pub struct ModuleState {
    pub arms: [ArmState; ARMS_PER_MODULE],
    pub module_id: u8,
}

impl ModuleState {
    pub fn new(module_id: u8) -> Self {
        Self {
            arms: [
                ArmState::new(),
                ArmState::new(),
                ArmState::new(),
                ArmState::new(),
            ],
            module_id,
        }
    }

    /// Build status messages for specified target
    pub fn status_messages(&self, target: ArmTarget) -> impl Iterator<Item = CanMessage> + '_ {
        (0..ARMS_PER_MODULE as u8)
            .filter(move |&arm| target.matches(self.module_id, arm))
            .map(move |arm| self.arms[arm as usize].to_status_message(self.module_id, arm))
    }
}

/// Color sensor interface
pub trait ColorSensor {
    /// Read color for specified arm
    async fn read_color(&mut self, arm: u8) -> Result<Color, ArmError>;
}

/// Module controller
/// Manages 4 arms with shared UART (servos) and I2C (IO expander, ground sensor)
pub struct Module<TX, RX, I2C, CS>
where
    TX: Write,
    RX: Read,
    I2C: I2c,
    CS: ColorSensor,
{
    /// Module ID (0-2)
    pub id: u8,
    /// Servo controller (shared for all 4 servos)
    servo: Scs0009<TX, RX>,
    /// I2C devices (PCA9535 + VCNL4040 + TLA2528)
    i2c_devices: I2cDevices<I2C>,
    /// Color sensor
    color_sensor: CS,
    /// State of all 4 arms
    state: ModuleState,
    /// Servo IDs for each arm (configurable)
    servo_ids: [u8; ARMS_PER_MODULE],
}

impl<TX, RX, I2C, CS> Module<TX, RX, I2C, CS>
where
    TX: Write,
    RX: Read,
    I2C: I2c,
    CS: ColorSensor,
{
    /// Create new module
    pub fn new(
        id: u8,
        servo: Scs0009<TX, RX>,
        i2c_devices: I2cDevices<I2C>,
        color_sensor: CS,
        servo_ids: [u8; ARMS_PER_MODULE],
    ) -> Self {
        Self {
            id,
            servo,
            i2c_devices,
            color_sensor,
            state: ModuleState::new(id),
            servo_ids,
        }
    }

    /// Scan for servos on the bus and log found IDs
    pub async fn scan_servos(&mut self) {
        rprintln!("Module {}: Scanning for servos...", self.id);
        let (found, count) = self.servo.scan::<8>(1, 32).await;
        if count == 0 {
            rprintln!("Module {}: No servos found", self.id);
        } else {
            rprintln!(
                "Module {}: Found {} servo(s): {:?}",
                self.id,
                count,
                &found[..count]
            );
        }
    }

    /// Initialize IO expander pins and ground sensor
    pub async fn init(&mut self) -> Result<(), ArmError> {
        log::debug!("Module {}: Scanning servos", self.id);
        self.scan_servos().await;

        log::debug!("Module {}: Initializing I2C devices", self.id);
        self.i2c_devices
            .init()
            .await
            .map_err(|_| ArmError::I2cError)?;

        // Configure valve pins as outputs (Bank 0, pins 0-3)
        for &pin in &VALVE_PINS {
            self.i2c_devices
                .pca_pin_into_output(VALVE_BANK, pin)
                .await
                .map_err(|_| ArmError::I2cError)?;
            self.i2c_devices
                .pca_pin_set_low(VALVE_BANK, pin)
                .await
                .map_err(|_| ArmError::I2cError)?;
        }

        // Configure LED pin as output (Bank 0, pin 7)
        self.i2c_devices
            .pca_pin_into_output(LED_BANK, LED_PIN)
            .await
            .map_err(|_| ArmError::I2cError)?;
        self.i2c_devices
            .pca_pin_set_low(LED_BANK, LED_PIN)
            .await
            .map_err(|_| ArmError::I2cError)?;

        // Configure pump pins as outputs (Bank 1, pins 0-3)
        for &pin in &PUMP_PINS {
            self.i2c_devices
                .pca_pin_into_output(PUMP_BANK, pin)
                .await
                .map_err(|_| ArmError::I2cError)?;
            self.i2c_devices
                .pca_pin_set_low(PUMP_BANK, pin)
                .await
                .map_err(|_| ArmError::I2cError)?;
        }

        Ok(())
    }

    /// Set pump state for arm (0-3)
    pub async fn set_pump(&mut self, arm: u8, enable: bool) -> Result<(), ArmError> {
        if arm as usize >= ARMS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }
        let pin = PUMP_PINS[arm as usize];
        self.i2c_devices
            .pca_pin_set(PUMP_BANK, pin, enable)
            .await
            .map_err(|_| ArmError::I2cError)
    }

    /// Set valve state for arm (0-3)
    pub async fn set_valve(&mut self, arm: u8, enable: bool) -> Result<(), ArmError> {
        if arm as usize >= ARMS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }
        let pin = VALVE_PINS[arm as usize];
        self.i2c_devices
            .pca_pin_set(VALVE_BANK, pin, enable)
            .await
            .map_err(|_| ArmError::I2cError)
    }

    /// Toggle LED
    pub async fn toggle_led(&mut self) -> Result<(), ArmError> {
        self.i2c_devices
            .pca_pin_toggle(LED_BANK, LED_PIN)
            .await
            .map_err(|_| ArmError::I2cError)
    }

    /// Get mutable access to the servo bus (for SetServoId command)
    pub fn servo_bus_mut(&mut self) -> &mut Scs0009<TX, RX> {
        &mut self.servo
    }

    /// Get current pump states as bitmask
    pub fn get_pumps(&self) -> u8 {
        self.i2c_devices.pca_get_output(PUMP_BANK) & 0x0F
    }

    /// Get current valve states as bitmask
    pub fn get_valves(&self) -> u8 {
        self.i2c_devices.pca_get_output(VALVE_BANK) & 0x0F
    }

    // ==================== Ground sensor methods ====================

    /// Update ground sensor reading
    pub async fn update_ground_sensor(&mut self) -> Result<(), ArmError> {
        self.i2c_devices
            .ground_update()
            .await
            .map_err(|_| ArmError::I2cError)
    }

    pub fn ground_value(&self) -> u16 {
        self.i2c_devices.ground_value()
    }

    pub fn ground_threshold(&self) -> u16 {
        self.i2c_devices.ground_threshold()
    }

    pub fn set_ground_threshold(&mut self, threshold: u16) {
        self.i2c_devices.ground_set_threshold(threshold);
    }

    pub fn ground_detected(&self) -> bool {
        self.i2c_devices.ground_detected()
    }

    // ==================== Arm control methods ====================

    /// Execute command on specified arm
    pub async fn execute_arm(&mut self, arm: u8, cmd: ArmCommand) -> Result<(), ArmError> {
        if arm as usize >= ARMS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }

        let servo_id = self.servo_ids[arm as usize];
        let arm_idx = arm as usize;

        match cmd {
            ArmCommand::SetAll {
                position,
                time_ms,
                pump,
                valve,
            } => {
                match self.servo.set_position(servo_id, position, time_ms, 0).await {
                    Ok(()) => {
                        self.state.arms[arm_idx].target_position = position;
                        self.state.arms[arm_idx].servo_error = 0;
                        self.state.arms[arm_idx].moving = true;
                        self.state.arms[arm_idx].position_reached = false;
                    }
                    Err(e) => {
                        self.state.arms[arm_idx].servo_error = scs_error_to_code(&e);
                        self.servo.set_torque(servo_id, true).await.ok();
                        return Err(scs_error_to_arm_error(e));
                    }
                }

                self.set_pump(arm, pump).await?;
                self.state.arms[arm_idx].pump = pump;

                self.set_valve(arm, valve).await?;
                self.state.arms[arm_idx].valve = valve;

                Ok(())
            }
            ArmCommand::SetTorque(enable) => {
                match self.servo.set_torque(servo_id, enable).await {
                    Ok(()) => {
                        self.state.arms[arm_idx].torque_enabled = enable;
                        self.state.arms[arm_idx].servo_error = 0;
                        Ok(())
                    }
                    Err(e) => {
                        self.state.arms[arm_idx].servo_error = scs_error_to_code(&e);
                        Err(scs_error_to_arm_error(e))
                    }
                }
            }
            ArmCommand::SetPump(enable) => {
                self.set_pump(arm, enable).await?;
                self.state.arms[arm_idx].pump = enable;
                Ok(())
            }
            ArmCommand::SetValve(enable) => {
                self.set_valve(arm, enable).await?;
                self.state.arms[arm_idx].valve = enable;
                Ok(())
            }
        }
    }

    /// Execute command on all arms matching target
    pub async fn execute_target(
        &mut self,
        target: ArmTarget,
        cmd: ArmCommand,
    ) -> Result<(), ArmError> {
        let mut last_error = None;

        for arm in 0..ARMS_PER_MODULE as u8 {
            if target.matches(self.id, arm) {
                if let Err(e) = self.execute_arm(arm, cmd.clone()).await {
                    last_error = Some(e);
                }
            }
        }

        match last_error {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Update state from hardware for specified arm
    pub async fn update_arm_state(&mut self, arm: u8) -> Result<(), ArmError> {
        if arm as usize >= ARMS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }

        let servo_id = self.servo_ids[arm as usize];
        let arm_state = &mut self.state.arms[arm as usize];

        // Read servo position
        match self.servo.read_position(servo_id).await {
            Ok((pos, servo_error)) => {
                arm_state.position = pos;
                arm_state.servo_error = servo_error;
                arm_state.check_position_reached(10);
            }
            Err(e) => {
                arm_state.servo_error = scs_error_to_code(&e);
            }
        }

        // Read color
        match self.color_sensor.read_color(arm).await {
            Ok(color) => arm_state.color = color,
            Err(_) => arm_state.color = Color::Unknown,
        }

        Ok(())
    }

    pub fn get_arm_state(&self, arm: u8) -> ArmState {
        self.state.arms[arm as usize].clone()
    }

    /// Update state for all arms
    pub async fn update_all_states(&mut self) -> Result<(), ArmError> {
        // Update pump currents from ADC
        if self.i2c_devices.pump_currents_update().await.is_ok() {
            for arm in 0..ARMS_PER_MODULE {
                self.state.arms[arm].pump_current = self.i2c_devices.pump_current(arm);
            }
        }

        for arm in 0..ARMS_PER_MODULE as u8 {
            let _ = self.update_arm_state(arm).await;
        }
        Ok(())
    }

    /// Handle CAN message if it targets this module
    pub async fn handle_message(&mut self, msg: &CanMessage) -> Option<Result<(), ArmError>> {
        let target = msg.arm_target()?;

        // Check if message targets this module
        if target.module != self.id && !target.is_module_broadcast() {
            return None;
        }

        rprintln!("Module {}: Handling message for target {:?}: {:?}", self.id, target, msg);

        let result = match msg {
            CanMessage::SetArm {
                position,
                time_ms,
                pump,
                valve,
                ..
            } => {
                let cmd = ArmCommand::SetAll {
                    position: *position,
                    time_ms: *time_ms,
                    pump: *pump,
                    valve: *valve,
                };
                if target.is_arm_broadcast() {
                    self.execute_target(target, cmd).await
                } else {
                    self.execute_arm(target.arm, cmd).await
                }
            }
            CanMessage::SetTorque { enable, .. } => {
                if target.is_arm_broadcast() {
                    self.execute_target(target, ArmCommand::SetTorque(*enable))
                        .await
                } else {
                    self.execute_arm(target.arm, ArmCommand::SetTorque(*enable))
                        .await
                }
            }
            CanMessage::SetPump { enable, .. } => {
                if target.is_arm_broadcast() {
                    self.execute_target(target, ArmCommand::SetPump(*enable))
                        .await
                } else {
                    self.execute_arm(target.arm, ArmCommand::SetPump(*enable))
                        .await
                }
            }
            CanMessage::SetValve { enable, .. } => {
                if target.is_arm_broadcast() {
                    self.execute_target(target, ArmCommand::SetValve(*enable))
                        .await
                } else {
                    self.execute_arm(target.arm, ArmCommand::SetValve(*enable))
                        .await
                }
            }
            CanMessage::RequestArmStatus { .. } => {
                if target.is_arm_broadcast() {
                    self.update_all_states().await
                } else {
                    self.update_arm_state(target.arm).await
                }
            }
            _ => return None,
        };

        Some(result)
    }

    /// Get status messages for target
    pub fn get_status_messages(&self, target: ArmTarget) -> impl Iterator<Item = CanMessage> + '_ {
        self.state.status_messages(target)
    }
}

fn scs_error_to_code(e: &ScsError) -> u8 {
    match e {
        ScsError::Timeout => 0x01,
        ScsError::InvalidResponse => 0x02,
        ScsError::ServoError(code) => *code,
        ScsError::TxError => 0x03,
        ScsError::RxError => 0x04,
    }
}

fn scs_error_to_arm_error(e: ScsError) -> ArmError {
    match e {
        ScsError::Timeout => ArmError::ServoTimeout,
        ScsError::InvalidResponse => ArmError::ServoError(0x02),
        ScsError::ServoError(code) => ArmError::ServoError(code),
        ScsError::TxError => ArmError::ServoError(0x03),
        ScsError::RxError => ArmError::ServoError(0x04),
    }
}
