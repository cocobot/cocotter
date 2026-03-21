use std::sync::{Arc, Mutex};
use crate::{GpioExpander, StandardExpanderInterface, GPIOBank};

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

    fn set_output(&mut self) -> Result<(), ()> {
        let mut device = self.handle.lock().unwrap();
        device.pin_into_output(self.get_bank(), self.get_pin_index_in_bank()).or(Err(()))?;
        self.is_output = true;
        Ok(())
    }

    pub fn pin_set_high(&mut self) -> Result<(), ()> {
        if !self.is_output {
            self.set_output()?;
        }

        let mut device = self.handle.lock().unwrap();
        self.last_written_value = true;
        device.pin_set_high(self.get_bank(), self.get_pin_index_in_bank()).or(Err(()))
    }

    pub fn pin_set_low(&mut self) -> Result<(), ()> {
        if !self.is_output {
            self.set_output()?;
        }

        let mut device = self.handle.lock().unwrap();
        self.last_written_value = false;
        device.pin_set_low(self.get_bank(), self.get_pin_index_in_bank()).or(Err(()))
    }

    pub fn toggle(&mut self) -> Result<(), ()> {
        match self.last_written_value {
            true => self.pin_set_low(),
            false => self.pin_set_high(),
        }
    }

    pub fn pin_is_low(&self) -> Result<bool, ()> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_low(self.get_bank(), self.get_pin_index_in_bank()).or(Err(()))
    }

    pub fn pin_is_high(&self) -> Result<bool, ()> {
        let mut device = self.handle.lock().unwrap();
        device.pin_is_high(self.get_bank(), self.get_pin_index_in_bank()).or(Err(()))
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
