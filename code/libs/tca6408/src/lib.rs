#![no_std]
use embedded_hal::i2c::I2c;

/// TCA6408 I2C GPIO expander driver
/// 
/// The TCA6408 is an 8-bit I2C GPIO expander with interrupt capability.
/// It provides 8 GPIO pins that can be configured as inputs or outputs.
pub struct TCA6408<I2C> {
    i2c: I2C,
    address: u8,
}

/// TCA6408 register addresses
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
enum Register {
    /// Input register - read the state of GPIO pins
    Input = 0x00,
    /// Output register - set the state of GPIO pins configured as outputs
    Output = 0x01,
    /// Polarity inversion register - invert the polarity of input pins
    PolarityInversion = 0x02,
    /// Configuration register - configure pins as inputs (1) or outputs (0)
    Configuration = 0x03,
}

#[derive(Debug)]
pub enum TCA6408Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid pin number (must be 0-7)
    InvalidPin,
}

impl<I2C, E> TCA6408<I2C> 
where 
    I2C: I2c<Error = E>,
{
    /// Create a new TCA6408 instance
    /// 
    /// # Arguments
    /// * `i2c` - I2C peripheral
    /// * `address` - I2C address of the TCA6408 (typically 0x20-0x27)
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Initialize the TCA6408 with default configuration
    /// All pins are configured as inputs with normal polarity
    pub fn init(&mut self) -> Result<(), TCA6408Error<E>> {
        // Configure all pins as inputs (default)
        self.set_configuration(0xFF)?;
        // Set normal polarity (default)
        self.set_polarity_inversion(false)?;
        // Set all outputs to low (default)
        self.write_outputs(0x00)?;
        Ok(())
    }

    /// Read the input register
    pub fn read_inputs(&mut self) -> Result<u8, TCA6408Error<E>> {
        self.read_register(Register::Input)
    }

    /// Write to the output register
    pub fn write_outputs(&mut self, outputs: u8) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::Output, outputs)
    }

    /// Set the configuration register (1 = input, 0 = output)
    pub fn set_configuration(&mut self, input_pins: u8) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::Configuration, input_pins)
    }

    /// Get the configuration register
    pub fn get_configuration(&mut self) -> Result<u8, TCA6408Error<E>> {
        self.read_register(Register::Configuration)
    }

    /// Set the polarity inversion register (1 = inverted, 0 = normal)
    pub fn set_polarity_inversion(&mut self, inverted: bool) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::PolarityInversion, inverted as u8)
    }

    /// Get the polarity inversion register
    pub fn get_polarity_inversion(&mut self) -> Result<bool, TCA6408Error<E>> {
        let value = self.read_register(Register::PolarityInversion)?;
        Ok(value != 0)
    }

    /// Configure a specific pin as input or output
    pub fn set_pin_mode(&mut self, pin: u8, is_input: bool) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }

        let mut config = self.get_configuration()?;
        if is_input {
            config |= 1 << pin;
        } else {
            config &= !(1 << pin);
        }
        self.set_configuration(config)
    }

    /// Set a specific output pin high or low
    pub fn set_output_pin(&mut self, pin: u8, value: bool) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }

        let mut outputs = self.read_register(Register::Output)?;
        if value {
            outputs |= 1 << pin;
        } else {
            outputs &= !(1 << pin);
        }
        self.write_outputs(outputs)
    }

    /// Read a specific input pin
    pub fn read_input_pin(&mut self, pin: u8) -> Result<bool, TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }

        let inputs = self.read_inputs()?;
        Ok(inputs & (1 << pin) != 0)
    }

    /// Toggle a specific output pin
    pub fn toggle_output_pin(&mut self, pin: u8) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }

        let mut outputs = self.read_register(Register::Output)?;
        outputs ^= 1 << pin;
        self.write_outputs(outputs)
    }

    /// Read a register from the TCA6408
    fn read_register(&mut self, register: Register) -> Result<u8, TCA6408Error<E>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register as u8], &mut buffer)
            .map_err(TCA6408Error::I2c)?;
        Ok(buffer[0])
    }

    /// Write a register to the TCA6408
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), TCA6408Error<E>> {
        self.i2c
            .write(self.address, &[register as u8, value])
            .map_err(TCA6408Error::I2c)
    }
}
