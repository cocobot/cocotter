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
use crate::can_protocol::{ArmFlags, ArmTarget, CanMessage, Stage2Target};
use crate::i2c_devices::{GPIOBank, I2cDevices};
use crate::scs0009::{Scs0009, ScsError};
use embedded_hal_async::i2c::I2c;
use embedded_io_async::{Read, Write};
use log::debug;
use rtt_target::rprintln;

/// Number of arms per module
pub const ARMS_PER_MODULE: usize = 4;

/// Number of stage2 servos per module
pub const STAGE2_SERVOS_PER_MODULE: usize = 2;

/// Pin assignments for valves (Bank 0, pins 0-3)
const VALVE_BANK: GPIOBank = GPIOBank::Bank0;
const VALVE_PINS: [u8; 4] = [0, 1, 2, 3];

/// Pin assignment for LED (Bank 0, pin 7)
const LED_BANK: GPIOBank = GPIOBank::Bank0;
const LED_PIN: u8 = 7;

/// Pin assignments for pumps (Bank 1, pins 0-3)
const PUMP_BANK: GPIOBank = GPIOBank::Bank1;
const PUMP_PINS: [u8; 4] = [0, 1, 2, 3];

/// Pin assignment for battery measurement enable (Bank 0, pin 6)
const BATT_ENABLE_BANK: GPIOBank = GPIOBank::Bank0;
const BATT_ENABLE_PIN: u8 = 6;

/// Stage2 servo state
#[derive(Debug, Clone, Default)]
pub struct Stage2ServoState {
    pub position: u16,
    pub target_position: u16,
    pub servo_error: u8,
    pub torque_enabled: bool,
    pub moving: bool,
    pub position_reached: bool,
}

impl Stage2ServoState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn flags(&self) -> ArmFlags {
        ArmFlags {
            torque_enabled: self.torque_enabled,
            moving: self.moving,
            position_reached: self.position_reached,
        }
    }

    pub fn to_status_message(&self, module: u8, servo: u8) -> CanMessage {
        CanMessage::Stage2Status {
            target: Stage2Target::new(module, servo),
            position: self.position,
            error: self.servo_error,
            flags: self.flags(),
        }
    }

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

/// Module state containing all 4 arms
pub struct ModuleState {
    pub arms: [ArmState; ARMS_PER_MODULE],
    pub stage2: [Stage2ServoState; STAGE2_SERVOS_PER_MODULE],
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
            stage2: [Stage2ServoState::new(), Stage2ServoState::new()],
            module_id,
        }
    }

    /// Build status messages for specified target
    pub fn status_messages(&self, target: ArmTarget) -> impl Iterator<Item = CanMessage> + '_ {
        (0..ARMS_PER_MODULE as u8)
            .filter(move |&arm| target.matches(self.module_id, arm))
            .map(move |arm| self.arms[arm as usize].to_status_message(self.module_id, arm))
    }

    /// Build stage2 status messages for specified target
    pub fn stage2_status_messages(
        &self,
        target: Stage2Target,
    ) -> impl Iterator<Item = CanMessage> + '_ {
        (0..STAGE2_SERVOS_PER_MODULE as u8)
            .filter(move |&servo| target.matches(self.module_id, servo))
            .map(move |servo| {
                self.stage2[servo as usize].to_status_message(self.module_id, servo)
            })
    }
}

/// Module controller
/// Manages 4 arms with shared UART (servos) and I2C (IO expander, ground sensor, color sensors)
pub struct Module<TX, RX, I2C>
where
    TX: Write,
    RX: Read,
    I2C: I2c,
{
    /// Module ID (0-2)
    pub id: u8,
    /// Servo controller (shared for all 4 servos)
    servo: Scs0009<TX, RX>,
    /// I2C devices (PCA9535 + VCNL4040 + TLA2528 + TCA9548A/TCS3472)
    i2c_devices: I2cDevices<I2C>,
    /// State of all 4 arms
    state: ModuleState,
    /// Servo IDs for each arm (configurable)
    servo_ids: [u8; ARMS_PER_MODULE],
    /// Servo IDs for stage2 servos (configurable)
    stage2_servo_ids: [u8; STAGE2_SERVOS_PER_MODULE],
    /// Per-arm color delta (last measured on-off difference) [C, R, G, B]
    color_deltas: [[u16; 4]; ARMS_PER_MODULE],
    /// Per-arm color baseline (LED off reading) [C, R, G, B]
    color_baseline: [[u16; 4]; ARMS_PER_MODULE],
    /// Previous moving states for change detection
    prev_arm_moving: [bool; ARMS_PER_MODULE],
    prev_stage2_moving: [bool; STAGE2_SERVOS_PER_MODULE],
}

impl<TX, RX, I2C> Module<TX, RX, I2C>
where
    TX: Write,
    RX: Read,
    I2C: I2c,
{
    /// Create new module
    pub fn new(
        id: u8,
        servo: Scs0009<TX, RX>,
        i2c_devices: I2cDevices<I2C>,
        servo_ids: [u8; ARMS_PER_MODULE],
        stage2_servo_ids: [u8; STAGE2_SERVOS_PER_MODULE],
    ) -> Self {
        Self {
            id,
            servo,
            i2c_devices,
            state: ModuleState::new(id),
            servo_ids,
            stage2_servo_ids,
            color_deltas: [[0u16; 4]; ARMS_PER_MODULE],
            color_baseline: [[0u16; 4]; ARMS_PER_MODULE],
            prev_arm_moving: [false; ARMS_PER_MODULE],
            prev_stage2_moving: [false; STAGE2_SERVOS_PER_MODULE],
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
        //debug!("Module {}: Scanning servos", self.id);
        //self.scan_servos().await;

        //let from = 1;
        //let target = 21;
        // if self.servo.unlock_eeprom(from).await.is_err() {
        //    }
        //    Timer::after_millis(50).await;
//
        //    if self.servo.modify_id(from, target).await.is_err() {
        //    }
        //    Timer::after_millis(50).await;
//
        //    if self.servo.lock_eeprom(target).await.is_err() {
        //    }
        //    Timer::after_millis(50).await;
//
        //    match self.servo.ping(target).await {
        //        Ok(true) => {
        //            rprintln!("Servo ID set to {} successfully", target);
        //            
        //        }
        //        _ => {
        //            rprintln!("Failed to verify servo ID {}", target);
        //            
        //        }
        //    };

        debug!("Module {}: Initializing I2C devices", self.id);
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

        // Enable battery voltage measurement (Bank 0, pin 6 = HIGH)
        self.i2c_devices
            .pca_pin_into_output(BATT_ENABLE_BANK, BATT_ENABLE_PIN)
            .await
            .map_err(|_| ArmError::I2cError)?;
        self.i2c_devices
            .pca_pin_set_high(BATT_ENABLE_BANK, BATT_ENABLE_PIN)
            .await
            .map_err(|_| ArmError::I2cError)?;

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
    pub async fn update_ground_sensor(&mut self) -> Result<bool, ArmError> {
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

        Ok(())
    }

    pub fn get_arm_state(&self, arm: u8) -> ArmState {
        self.state.arms[arm as usize].clone()
    }

    pub fn get_stage2_state(&self, servo: u8) -> Stage2ServoState {
        self.state.stage2[servo as usize].clone()
    }

    /// Get battery voltage raw ADC value (None if I2C failed)
    pub fn battery_voltage_raw(&self) -> Option<u16> {
        self.i2c_devices.battery_voltage_raw()
    }

    /// Update state for all arms
    pub async fn update_all_states(&mut self) -> Result<(), ArmError> {
        // Update pump currents from ADC
        if self.i2c_devices.pump_currents_update().await.is_ok() {
            for arm in 0..ARMS_PER_MODULE {
                self.state.arms[arm].pump_current = self.i2c_devices.pump_current(arm);
            }
        }

        // Update battery voltage from ADC channel 6
        self.i2c_devices.battery_voltage_raw_update().await;

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
            CanMessage::SetColorSensorConfig {
                integration_time,
                gain,
                ..
            } => {
                if target.is_arm_broadcast() {
                    let mut last_err = Ok(());
                    for arm in 0..ARMS_PER_MODULE as u8 {
                        if let Err(e) = self.i2c_devices
                            .tcs_set_config(arm, *integration_time as u16, *gain)
                            .await
                        {
                            let _ = e;
                            last_err = Err(ArmError::ColorSensorError);
                        }
                    }
                    last_err
                } else {
                    self.i2c_devices
                        .tcs_set_config(target.arm, *integration_time as u16, *gain)
                        .await
                        .map_err(|_| ArmError::ColorSensorError)
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

    /// Get stage2 status messages for target
    pub fn get_stage2_status_messages(
        &self,
        target: Stage2Target,
    ) -> impl Iterator<Item = CanMessage> + '_ {
        self.state.stage2_status_messages(target)
    }

    /// Check if any arm or stage2 moving state has changed since last call
    pub fn has_moving_changed(&mut self) -> bool {
        let mut changed = false;
        for i in 0..ARMS_PER_MODULE {
            let moving = self.state.arms[i].moving;
            if moving != self.prev_arm_moving[i] {
                self.prev_arm_moving[i] = moving;
                changed = true;
            }
        }
        for i in 0..STAGE2_SERVOS_PER_MODULE {
            let moving = self.state.stage2[i].moving;
            if moving != self.prev_stage2_moving[i] {
                self.prev_stage2_moving[i] = moving;
                changed = true;
            }
        }
        changed
    }

    // ==================== Stage2 servo methods ====================

    /// Set stage2 servo position
    pub async fn set_stage2_position(
        &mut self,
        servo: u8,
        position: u16,
        time_ms: u16,
    ) -> Result<(), ArmError> {
        if servo as usize >= STAGE2_SERVOS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }
        let servo_id = self.stage2_servo_ids[servo as usize];
        let state = &mut self.state.stage2[servo as usize];
        match self.servo.set_position(servo_id, position, time_ms, 0).await {
            Ok(()) => {
                state.target_position = position;
                state.servo_error = 0;
                state.moving = true;
                state.position_reached = false;
                Ok(())
            }
            Err(e) => {
                state.servo_error = scs_error_to_code(&e);
                self.servo.set_torque(servo_id, true).await.ok();
                Err(scs_error_to_arm_error(e))
            }
        }
    }

    /// Set stage2 servo torque
    pub async fn set_stage2_torque(
        &mut self,
        servo: u8,
        enable: bool,
    ) -> Result<(), ArmError> {
        if servo as usize >= STAGE2_SERVOS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }
        let servo_id = self.stage2_servo_ids[servo as usize];
        let state = &mut self.state.stage2[servo as usize];
        match self.servo.set_torque(servo_id, enable).await {
            Ok(()) => {
                state.torque_enabled = enable;
                state.servo_error = 0;
                Ok(())
            }
            Err(e) => {
                state.servo_error = scs_error_to_code(&e);
                Err(scs_error_to_arm_error(e))
            }
        }
    }

    /// Update stage2 servo state from hardware
    pub async fn update_stage2_state(&mut self, servo: u8) -> Result<(), ArmError> {
        if servo as usize >= STAGE2_SERVOS_PER_MODULE {
            return Err(ArmError::ServoError(0xFF));
        }
        let servo_id = self.stage2_servo_ids[servo as usize];
        let state = &mut self.state.stage2[servo as usize];
        match self.servo.read_position(servo_id).await {
            Ok((pos, servo_error)) => {
                state.position = pos;
                state.servo_error = servo_error;
                state.check_position_reached(10);
            }
            Err(e) => {
                state.servo_error = scs_error_to_code(&e);
            }
        }
        Ok(())
    }

    /// Update all stage2 servo states
    pub async fn update_all_stage2_states(&mut self) -> Result<(), ArmError> {
        for servo in 0..STAGE2_SERVOS_PER_MODULE as u8 {
            let _ = self.update_stage2_state(servo).await;
        }
        Ok(())
    }

    /// Handle stage2 CAN message if it targets this module
    pub async fn handle_stage2_message(&mut self, msg: &CanMessage) -> Option<Result<(), ArmError>> {
        let target = msg.stage2_target()?;

        if target.module != self.id && !target.is_module_broadcast() {
            return None;
        }

        let result = match msg {
            CanMessage::SetStage2 {
                position, time_ms, ..
            } => {
                let mut last_error = None;
                for servo in 0..STAGE2_SERVOS_PER_MODULE as u8 {
                    if target.matches(self.id, servo) {
                        if let Err(e) = self.set_stage2_position(servo, *position, *time_ms).await {
                            last_error = Some(e);
                        }
                    }
                }
                last_error.map_or(Ok(()), Err)
            }
            CanMessage::SetStage2Torque { enable, .. } => {
                let mut last_error = None;
                for servo in 0..STAGE2_SERVOS_PER_MODULE as u8 {
                    if target.matches(self.id, servo) {
                        if let Err(e) = self.set_stage2_torque(servo, *enable).await {
                            last_error = Some(e);
                        }
                    }
                }
                last_error.map_or(Ok(()), Err)
            }
            CanMessage::RequestStage2Status { .. } => {
                for servo in 0..STAGE2_SERVOS_PER_MODULE as u8 {
                    if target.matches(self.id, servo) {
                        let _ = self.update_stage2_state(servo).await;
                    }
                }
                Ok(())
            }
            _ => return None,
        };

        Some(result)
    }

    // ==================== Color sensor methods ====================

    /// Set color config for a specific arm
    /// Store color result for an arm (called from main after delta computation)
    pub fn set_color_result(&mut self, arm: usize, hue_byte: u8, delta: [u16; 4]) {
        if arm < ARMS_PER_MODULE {
            self.state.arms[arm].color = hue_byte;
            self.color_deltas[arm] = delta;
        }
    }

    /// Get last measured color delta for a specific arm
    pub fn get_color_delta(&self, arm: u8) -> Option<CanMessage> {
        if (arm as usize) >= ARMS_PER_MODULE {
            return None;
        }
        let d = &self.color_deltas[arm as usize];
        Some(CanMessage::ColorSensorRaw {
            target: ArmTarget::new(self.id, arm),
            clear: d[0],
            red: d[1],
            green: d[2],
            blue: d[3],
        })
    }

    /// Read colors and update baseline (led_on=false) or compute delta/hue (led_on=true)
    pub async fn update_color(&mut self, led_on: bool, threshold: u16) {
        let colors = self.read_all_color_raw().await;
        if led_on {
            for arm in 0..ARMS_PER_MODULE {
                let dc = colors[arm][0].saturating_sub(self.color_baseline[arm][0]);
                let dr = colors[arm][1].saturating_sub(self.color_baseline[arm][1]) as i32;
                let dg = colors[arm][2].saturating_sub(self.color_baseline[arm][2]) as i32;
                let db = colors[arm][3].saturating_sub(self.color_baseline[arm][3]) as i32;

                let delta = [dc, dr as u16, dg as u16, db as u16];

                let max_rgb = dr.max(dg).max(db);
                let min_rgb = dr.min(dg).min(db);
                let range = max_rgb - min_rgb;

                let hue = if range == 0 {
                    0
                } else if max_rgb == dr {
                    (60 * (dg - db) / range + 360) % 360
                } else if max_rgb == dg {
                    60 * (db - dr) / range + 120
                } else {
                    60 * (dr - dg) / range + 240
                };

                let hue7 = (hue * 128 / 360) as u8;
                let detected = dc >= threshold;
                let hue_byte = if detected { 0x80 | hue7 } else { hue7 };

                self.set_color_result(arm, hue_byte, delta);
            }
        } else {
            self.color_baseline = colors;
        }
    }

    /// Read raw CRGB values for all arms (I2C read)
    pub async fn read_all_color_raw(&mut self) -> [[u16; 4]; ARMS_PER_MODULE] {
        let mut result = [[0u16; 4]; ARMS_PER_MODULE];
        for arm in 0..ARMS_PER_MODULE as u8 {
            if let Ok(crgb) = self.i2c_devices.tcs_read_all(arm).await {
                result[arm as usize] = crgb;
            }
        }
        result
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
