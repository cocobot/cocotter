use std::sync::Mutex;
use embedded_hal::i2c::{I2c, SevenBitAddress};


/// VLX I2C operations
///
/// Methods return booleans to avoid dependency on the Error type
pub trait VlxI2c: Send + Sync {
    fn vlx_i2c_read(&mut self, address: SevenBitAddress, register: u16, data: &mut [u8]) -> bool;
    fn vlx_i2c_write(&mut self, address: SevenBitAddress, register: u16, data: &[u8]) -> bool;
}

impl<T: I2c + Send + Sync> VlxI2c for T {
    fn vlx_i2c_read(&mut self, address: SevenBitAddress, register: u16, data: &mut [u8]) -> bool {
        let reg_bytes = register.to_be_bytes();
        self.write_read(address, &reg_bytes, data).is_ok()
    }

    fn vlx_i2c_write(&mut self, address: SevenBitAddress, register: u16, data: &[u8]) -> bool {
        let reg_bytes = register.to_be_bytes();
        let mut buffer = Vec::with_capacity(reg_bytes.len() + data.len());
        buffer.extend_from_slice(&reg_bytes);
        buffer.extend_from_slice(data);
        self.write(address, &buffer).is_ok()
    }
}


// Upstream C implementations don't support multiple I2C buses.
// Use a single, global I2C bus for all VLX devices.

pub struct VlxI2cDriver;

impl VlxI2cDriver {
    /// Register the unique I2C used for VLX devices
    ///
    /// This method must be called only once.
    pub fn register<I2C: I2c<SevenBitAddress> + Send + Sync + 'static>(i2c: I2C) -> Self {
        let mut singleton = VLX_I2C_SINGLETON.lock().unwrap();
        if singleton.is_some() {
            panic!("Attempt to register multiple VLX I2C devices");
        }
        *singleton = Some(Box::new(i2c));
        Self {}
    }

    pub(crate) fn vlx_i2c_write(address: u8, register: u16, data: &[u8]) -> bool {
        let mut singleton = match VLX_I2C_SINGLETON.lock() {
            Ok(guard) => guard,
            Err(_) => return false,
        };

        if let Some(ref mut i2c) = *singleton {
            i2c.vlx_i2c_write(address, register, data)
        } else {
            false  // No singleton, should never happen
        }
    }

    pub(crate) fn vlx_i2c_read(address: u8, register: u16, data: &mut [u8]) -> bool {
        let mut singleton = match VLX_I2C_SINGLETON.lock() {
            Ok(guard) => guard,
            Err(_) => return false,
        };

        if let Some(ref mut i2c) = *singleton {
            i2c.vlx_i2c_read(address, register, data)
        } else {
            false  // No singleton, should never happen
        }
    }
}

static VLX_I2C_SINGLETON: Mutex<Option<Box<dyn VlxI2c>>> = Mutex::new(None);

