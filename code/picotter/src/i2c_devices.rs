//! I2C devices wrapper
//!
//! Groups PCA9535 (IO expander), VCNL4040 (ground sensor) and TLA2528 (ADC)
//! on the same I2C bus. Access is sequential (no concurrent I2C operations).

use embedded_hal_async::i2c::I2c;

use crate::ground_sensors::GroundSensorState;

// ==================== GPIO Bank (from PCA9535) ====================

/// GPIO bank selection for PCA9535 IO expander
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GPIOBank {
    Bank0,
    Bank1,
}

// ==================== Device addresses ====================

const PCA9535_ADDR: u8 = 0x20;
const VCNL4040_ADDR: u8 = 0x60;
const TLA2528_ADDR: u8 = 0x13;

// VCNL4040 registers
const REG_PS_CONF1_2: u8 = 0x03;
const REG_PS_DATA: u8 = 0x08;

// TLA2528 opcodes and registers
const TLA_OPCODE_WRITE: u8 = 0x08;
const TLA_REG_OPMODE_CFG: u8 = 0x04;
const TLA_REG_DATA_CFG: u8 = 0x02;
const TLA_REG_PIN_CFG: u8 = 0x05;
const TLA_REG_CHANNEL_SEL: u8 = 0x11;

/// I2C devices on a single bus
pub struct I2cDevices<I2C> {
    i2c: I2C,
    /// PCA9535 cached state
    pca_output_0: u8,
    pca_output_1: u8,
    pca_config_0: u8,
    pca_config_1: u8,
    /// Ground sensor state
    ground_state: GroundSensorState,
    /// Pump current readings (channels 0-3)
    pump_currents: [u16; 4],
}

impl<I2C: I2c> I2cDevices<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            pca_output_0: 0xFF,
            pca_output_1: 0xFF,
            pca_config_0: 0xFF,
            pca_config_1: 0xFF,
            ground_state: GroundSensorState::new(),
            pump_currents: [0; 4],
        }
    }

    /// Initialize all devices
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        // Init VCNL4040: PS_DUTY = 1/40, PS_IT = 1T, PS_SD = 0 (power on)
        self.vcnl_write_register(REG_PS_CONF1_2, 0x0800).await?;

        // Init TLA2528 ADC
        self.tla_init().await?;

        Ok(())
    }

    // ==================== PCA9535 methods ====================

    /// Configure pin as output
    pub async fn pca_pin_into_output(
        &mut self,
        bank: GPIOBank,
        pin: u8,
    ) -> Result<(), I2C::Error> {
        if pin > 7 {
            return Ok(());
        }
        let mask = !(1u8 << pin);
        match bank {
            GPIOBank::Bank0 => {
                self.pca_config_0 &= mask;
                self.pca_write_reg(0x06, self.pca_config_0).await?;
            }
            GPIOBank::Bank1 => {
                self.pca_config_1 &= mask;
                self.pca_write_reg(0x07, self.pca_config_1).await?;
            }
        }
        Ok(())
    }

    /// Set pin high
    pub async fn pca_pin_set_high(
        &mut self,
        bank: GPIOBank,
        pin: u8,
    ) -> Result<(), I2C::Error> {
        if pin > 7 {
            return Ok(());
        }
        let mask = 1u8 << pin;
        match bank {
            GPIOBank::Bank0 => {
                self.pca_output_0 |= mask;
                self.pca_write_reg(0x02, self.pca_output_0).await?;
            }
            GPIOBank::Bank1 => {
                self.pca_output_1 |= mask;
                self.pca_write_reg(0x03, self.pca_output_1).await?;
            }
        }
        Ok(())
    }

    /// Set pin low
    pub async fn pca_pin_set_low(
        &mut self,
        bank: GPIOBank,
        pin: u8,
    ) -> Result<(), I2C::Error> {
        if pin > 7 {
            return Ok(());
        }
        let mask = !(1u8 << pin);
        match bank {
            GPIOBank::Bank0 => {
                self.pca_output_0 &= mask;
                self.pca_write_reg(0x02, self.pca_output_0).await?;
            }
            GPIOBank::Bank1 => {
                self.pca_output_1 &= mask;
                self.pca_write_reg(0x03, self.pca_output_1).await?;
            }
        }
        Ok(())
    }

    /// Set pin state
    pub async fn pca_pin_set(
        &mut self,
        bank: GPIOBank,
        pin: u8,
        high: bool,
    ) -> Result<(), I2C::Error> {
        if high {
            self.pca_pin_set_high(bank, pin).await
        } else {
            self.pca_pin_set_low(bank, pin).await
        }
    }

    /// Toggle pin
    pub async fn pca_pin_toggle(
        &mut self,
        bank: GPIOBank,
        pin: u8,
    ) -> Result<(), I2C::Error> {
        if pin > 7 {
            return Ok(());
        }
        let mask = 1u8 << pin;
        match bank {
            GPIOBank::Bank0 => {
                self.pca_output_0 ^= mask;
                self.pca_write_reg(0x02, self.pca_output_0).await?;
            }
            GPIOBank::Bank1 => {
                self.pca_output_1 ^= mask;
                self.pca_write_reg(0x03, self.pca_output_1).await?;
            }
        }
        Ok(())
    }

    /// Get cached output state
    pub fn pca_get_output(&self, bank: GPIOBank) -> u8 {
        match bank {
            GPIOBank::Bank0 => self.pca_output_0,
            GPIOBank::Bank1 => self.pca_output_1,
        }
    }

    async fn pca_write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(PCA9535_ADDR, &[reg, value]).await
    }

    // ==================== VCNL4040 methods ====================

    /// Read ground sensor and update state
    pub async fn ground_update(&mut self) -> Result<(), I2C::Error> {
        let value = self.vcnl_read_register(REG_PS_DATA).await?;
        self.ground_state.update(value);
        Ok(())
    }

    /// Get ground sensor state
    pub fn ground_state(&self) -> &GroundSensorState {
        &self.ground_state
    }

    /// Set ground sensor threshold
    pub fn ground_set_threshold(&mut self, threshold: u16) {
        self.ground_state.set_threshold(threshold);
    }

    /// Get ground sensor value
    pub fn ground_value(&self) -> u16 {
        self.ground_state.value
    }

    /// Get ground sensor threshold
    pub fn ground_threshold(&self) -> u16 {
        self.ground_state.threshold
    }

    /// Is ground detected
    pub fn ground_detected(&self) -> bool {
        self.ground_state.detected
    }

    async fn vcnl_read_register(&mut self, reg: u8) -> Result<u16, I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(VCNL4040_ADDR, &[reg], &mut buf)
            .await?;
        Ok(u16::from_le_bytes(buf))
    }

    async fn vcnl_write_register(&mut self, reg: u8, value: u16) -> Result<(), I2C::Error> {
        let bytes = value.to_le_bytes();
        self.i2c
            .write(VCNL4040_ADDR, &[reg, bytes[0], bytes[1]])
            .await
    }

    // ==================== TLA2528 ADC methods ====================

    /// Initialize TLA2528 ADC
    async fn tla_init(&mut self) -> Result<(), I2C::Error> {
        // Manual mode
        self.tla_write_reg(TLA_REG_OPMODE_CFG, 0x00).await?;
        // Append channel ID to data
        self.tla_write_reg(TLA_REG_DATA_CFG, 0x10).await?;
        // All pins as analog inputs
        self.tla_write_reg(TLA_REG_PIN_CFG, 0x00).await?;
        Ok(())
    }

    /// Read a single ADC channel (0-7)
    pub async fn tla_read_channel(&mut self, channel: u8) -> Result<u16, I2C::Error> {
        if channel > 7 {
            return Ok(0);
        }

        // Select channel
        self.tla_write_reg(TLA_REG_CHANNEL_SEL, channel).await?;

        // Read conversion result (2 bytes)
        let mut buf = [0u8; 2];
        self.i2c.read(TLA2528_ADDR, &mut buf).await?;

        // 12-bit result, left-aligned in 16 bits -> right-align
        let raw = ((buf[0] as u16) << 8) | (buf[1] as u16);
        Ok(raw >> 4)
    }

    /// Update all pump current readings (channels 0-3)
    pub async fn pump_currents_update(&mut self) -> Result<(), I2C::Error> {
        for i in 0..4 {
            self.pump_currents[i] = self.tla_read_channel(i as u8).await?;
        }
        Ok(())
    }

    /// Get pump current for a specific pump (0-3)
    pub fn pump_current(&self, pump: usize) -> u16 {
        if pump < 4 {
            self.pump_currents[pump]
        } else {
            0
        }
    }

    async fn tla_write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c
            .write(TLA2528_ADDR, &[TLA_OPCODE_WRITE | reg, value])
            .await
    }
}
