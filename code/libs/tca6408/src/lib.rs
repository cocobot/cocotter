#![no_std]

use bitfield::bitfield;
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
pub enum Register {
    /// Input register - read the state of GPIO pins
    Input = 0x00,
    /// Output register - set the state of GPIO pins configured as outputs
    Output = 0x01,
    /// Polarity inversion register - invert the polarity of input pins
    PolarityInversion = 0x02,
    /// Configuration register - configure pins as inputs (1) or outputs (0)
    Configuration = 0x03,
}

bitfield! {
    /// GPIO pin state bitfield
    #[derive(Clone, Copy, PartialEq)]
    pub struct GpioPins(u8);
    impl Debug;
    
    pub p0, set_p0: 0;
    pub p1, set_p1: 1;
    pub p2, set_p2: 2;
    pub p3, set_p3: 3;
    pub p4, set_p4: 4;
    pub p5, set_p5: 5;
    pub p6, set_p6: 6;
    pub p7, set_p7: 7;
}

impl GpioPins {
    /// Create a new GpioPins with all pins set to 0
    pub fn new() -> Self {
        GpioPins(0)
    }
    
    /// Create a new GpioPins from a u8 value
    pub fn from_u8(value: u8) -> Self {
        GpioPins(value)
    }
    
    /// Get the raw u8 value
    pub fn as_u8(&self) -> u8 {
        self.0
    }
    
    /// Set a specific pin
    pub fn set_pin(&mut self, pin: u8, value: bool) {
        if pin < 8 {
            if value {
                self.0 |= 1 << pin;
            } else {
                self.0 &= !(1 << pin);
            }
        }
    }
    
    /// Get a specific pin
    pub fn get_pin(&self, pin: u8) -> bool {
        if pin < 8 {
            (self.0 & (1 << pin)) != 0
        } else {
            false
        }
    }
}

impl Default for GpioPins {
    fn default() -> Self {
        Self::new()
    }
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
        self.write_register(Register::Configuration, 0xFF)?;
        // Set normal polarity (default)
        self.write_register(Register::PolarityInversion, 0x00)?;
        // Set all outputs to low (default)
        self.write_register(Register::Output, 0x00)?;
        Ok(())
    }
    
    /// Read the input register
    pub fn read_inputs(&mut self) -> Result<GpioPins, TCA6408Error<E>> {
        let value = self.read_register(Register::Input)?;
        Ok(GpioPins::from_u8(value))
    }
    
    /// Write to the output register
    pub fn write_outputs(&mut self, outputs: GpioPins) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::Output, outputs.as_u8())
    }
    
    /// Set the configuration register (1 = input, 0 = output)
    pub fn set_configuration(&mut self, config: GpioPins) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::Configuration, config.as_u8())
    }
    
    /// Get the configuration register
    pub fn get_configuration(&mut self) -> Result<GpioPins, TCA6408Error<E>> {
        let value = self.read_register(Register::Configuration)?;
        Ok(GpioPins::from_u8(value))
    }
    
    /// Set the polarity inversion register (1 = inverted, 0 = normal)
    pub fn set_polarity_inversion(&mut self, inversion: GpioPins) -> Result<(), TCA6408Error<E>> {
        self.write_register(Register::PolarityInversion, inversion.as_u8())
    }
    
    /// Get the polarity inversion register
    pub fn get_polarity_inversion(&mut self) -> Result<GpioPins, TCA6408Error<E>> {
        let value = self.read_register(Register::PolarityInversion)?;
        Ok(GpioPins::from_u8(value))
    }
    
    /// Configure a specific pin as input or output
    pub fn set_pin_mode(&mut self, pin: u8, is_input: bool) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }
        
        let mut config = self.get_configuration()?;
        config.set_pin(pin, is_input);
        self.set_configuration(config)
    }
    
    /// Set a specific output pin high or low
    pub fn set_output_pin(&mut self, pin: u8, value: bool) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }
        
        let current_value = self.read_register(Register::Output)?;
        let mut outputs = GpioPins::from_u8(current_value);
        outputs.set_pin(pin, value);
        self.write_outputs(outputs)
    }
    
    /// Read a specific input pin
    pub fn read_input_pin(&mut self, pin: u8) -> Result<bool, TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }
        
        let inputs = self.read_inputs()?;
        Ok(inputs.get_pin(pin))
    }
    
    /// Toggle a specific output pin
    pub fn toggle_output_pin(&mut self, pin: u8) -> Result<(), TCA6408Error<E>> {
        if pin >= 8 {
            return Err(TCA6408Error::InvalidPin);
        }
        
        let current_value = self.read_register(Register::Output)?;
        let mut outputs = GpioPins::from_u8(current_value);
        let current_state = outputs.get_pin(pin);
        outputs.set_pin(pin, !current_state);
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

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_gpio_pins_bitfield() {
        let mut pins = GpioPins::new();
        
        // Test setting individual pins
        pins.set_p0(true);
        pins.set_p7(true);
        
        assert!(pins.p0());
        assert!(pins.p7());
        assert!(!pins.p1());
        assert_eq!(pins.as_u8(), 0b10000001);
        
        // Test setting via pin number
        pins.set_pin(3, true);
        assert!(pins.get_pin(3));
        assert_eq!(pins.as_u8(), 0b10001001);
    }
    
    #[test]
    fn test_gpio_pins_from_u8() {
        let pins = GpioPins::from_u8(0b10101010);
        
        assert!(!pins.p0());
        assert!(pins.p1());
        assert!(!pins.p2());
        assert!(pins.p3());
        assert!(!pins.p4());
        assert!(pins.p5());
        assert!(!pins.p6());
        assert!(pins.p7());
        
        assert_eq!(pins.as_u8(), 0b10101010);
    }
    
    #[test]
    fn test_gpio_pins_default() {
        let pins = GpioPins::default();
        assert_eq!(pins.as_u8(), 0);
        
        for i in 0..8 {
            assert!(!pins.get_pin(i));
        }
    }
    
    #[test]
    fn test_gpio_pins_set_get() {
        let mut pins = GpioPins::new();
        
        // Test all pins
        for i in 0..8 {
            pins.set_pin(i, true);
            assert!(pins.get_pin(i));
        }
        
        assert_eq!(pins.as_u8(), 0xFF);
        
        // Test clearing pins
        for i in 0..8 {
            pins.set_pin(i, false);
            assert!(!pins.get_pin(i));
        }
        
        assert_eq!(pins.as_u8(), 0x00);
    }
    
    #[test]
    fn test_gpio_pins_invalid_pin() {
        let pins = GpioPins::new();
        
        // Test invalid pin numbers
        assert!(!pins.get_pin(8));
        assert!(!pins.get_pin(255));
        
        let mut pins = GpioPins::new();
        pins.set_pin(8, true);  // Should be ignored
        pins.set_pin(255, true); // Should be ignored
        assert_eq!(pins.as_u8(), 0);
    }
}