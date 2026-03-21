use std::sync::{Arc, Mutex};
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use board_sabotter::{GpioExpander, GpioExpanderError};
use board_sabotter::pca9535::{GPIOBank, StandardExpanderInterface};


type GpioExpanderHandle = Arc<Mutex<GpioExpander>>;

pub struct SharedGpioPin {
    handle: GpioExpanderHandle,
    pin_number: u8,

    last_written_value: bool,
    is_output: bool,
}

impl SharedGpioPin {
    fn get_bank(&self) -> GPIOBank {
        match self.pin_number / 8 {
            0 => GPIOBank::Bank0,
            1 => GPIOBank::Bank1,
            
            _ => GPIOBank::Bank0, //should not happen
        }
    }

    fn get_pin_index_in_bank(&self) -> u8 {
        self.pin_number % 8
    }

    fn set_output(&mut self) -> Result<(), GpioExpanderError> {
        let mut device = self.handle.lock().unwrap();
        device.pin_into_output(self.get_bank(), self.get_pin_index_in_bank())?;
        self.is_output = true;
        Ok(())
    }
}


impl ErrorType for SharedGpioPin {
    type Error = GpioExpanderError;
}


impl InputPin for SharedGpioPin {
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_low(self.get_bank(), self.get_pin_index_in_bank())
    }

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_high(self.get_bank(), self.get_pin_index_in_bank())
    }
}

impl OutputPin for SharedGpioPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        if !self.is_output {
            self.set_output()?;
        }
        let mut device = self.handle.lock().unwrap();
        self.last_written_value = true;
        device.pin_set_high(self.get_bank(), self.get_pin_index_in_bank())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        if !self.is_output {
            self.set_output()?;
        }
        let mut device = self.handle.lock().unwrap();
        self.last_written_value = false;
        device.pin_set_low(self.get_bank(), self.get_pin_index_in_bank())
    }
}

impl StatefulOutputPin for SharedGpioPin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.last_written_value)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.last_written_value)
    }
}

pub struct SharedGpio {
    device : GpioExpanderHandle,
}

impl SharedGpio {
    pub fn new(device: GpioExpander) -> SharedGpio {
        SharedGpio {
            device: Arc::new(Mutex::new(device)),
        }
    }

    pub fn get_pin(&self, pin_number: u8) -> SharedGpioPin {
        SharedGpioPin {
            handle: self.device.clone(),
            pin_number,
            last_written_value: false,
            is_output: false,
        }
    }
}
