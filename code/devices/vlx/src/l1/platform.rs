#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use std::{thread, time::Duration};
use std::collections::HashMap;
use std::sync::{Mutex, Arc};

// Global registry for I2C drivers by device address
type I2cDriverBox = Box<dyn crate::I2c<Error = Box<dyn std::error::Error + Send + Sync>> + Send + Sync>;
static I2C_DRIVERS: Mutex<Option<HashMap<u16, Arc<Mutex<I2cDriverBox>>>>> = Mutex::new(None);

// Initialize the global I2C registry
pub fn init_i2c_registry() {
    let mut drivers = I2C_DRIVERS.lock().unwrap();
    if drivers.is_none() {
        *drivers = Some(HashMap::new());
    }
}

// Register an I2C driver for a specific device address
pub fn register_i2c_driver(address: u16, driver: I2cDriverBox) {
    let mut drivers = I2C_DRIVERS.lock().unwrap();
    if let Some(ref mut map) = *drivers {
        map.insert(address, Arc::new(Mutex::new(driver)));
    }
}

// Get I2C driver for a device address
fn get_i2c_driver(address: u16) -> Option<Arc<Mutex<I2cDriverBox>>> {
    let drivers = I2C_DRIVERS.lock().unwrap();
    if let Some(ref map) = *drivers {
        map.get(&address).cloned()
    } else {
        None
    }
}

// Define the device struct exactly as in the C header so that the size/layout match.
#[repr(C)]
pub struct VL53L1_Dev_t {
    pub dummy: u32,
}

// Type alias to keep the same semantics as the C header.
pub type VL53L1_DEV = *mut VL53L1_Dev_t;

// Below are stub implementations of the platform-dependent functions expected by the
// ST VL53L1X driver. They currently return 0 (success) and do not perform any I/O.
// Replace the bodies with real I²C/SPI transactions and delays as needed.

#[no_mangle]
pub extern "C" fn VL53L1_WriteMulti(
    dev: u16,
    index: u16,
    pdata: *const u8,
    count: u32,
) -> i8 {
    if let Some(driver) = get_i2c_driver(dev) {
        let data = unsafe { core::slice::from_raw_parts(pdata, count as usize) };
        let mut i2c = driver.lock().unwrap();
        match i2c.write(dev as u8, index, data) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    } else {
        -1
    }
}

#[no_mangle]
pub extern "C" fn VL53L1_ReadMulti(
    dev: u16,
    index: u16,
    pdata: *mut u8,
    count: u32,
) -> i8 {
    if let Some(driver) = get_i2c_driver(dev) {
        let data = unsafe { core::slice::from_raw_parts_mut(pdata, count as usize) };
        let mut i2c = driver.lock().unwrap();
        match i2c.read(dev as u8, index, data) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    } else {
        -1
    }
}

#[no_mangle]
pub extern "C" fn VL53L1_WrByte(dev: u16, index: u16, data: u8) -> i8 {
    VL53L1_WriteMulti(dev, index, &data as *const u8, 1)
}

#[no_mangle]
pub extern "C" fn VL53L1_WrWord(dev: u16, index: u16, data: u16) -> i8 {
    let bytes = data.to_be_bytes();
    VL53L1_WriteMulti(dev, index, bytes.as_ptr(), 2)
}

#[no_mangle]
pub extern "C" fn VL53L1_WrDWord(dev: u16, index: u16, data: u32) -> i8 {
    let bytes = data.to_be_bytes();
    VL53L1_WriteMulti(dev, index, bytes.as_ptr(), 4)
}

#[no_mangle]
pub extern "C" fn VL53L1_RdByte(dev: u16, index: u16, pdata: *mut u8) -> i8 {
    let mut data = 0;
    let res =VL53L1_ReadMulti(dev, index, &mut data as *mut u8, 1);
    unsafe {
        *pdata = data;
    }
    res
}

#[no_mangle]
pub extern "C" fn VL53L1_RdWord(dev: u16, index: u16, pdata: *mut u16) -> i8 {
    let mut buf = [0u8; 2];
    let res = VL53L1_ReadMulti(dev, index, buf.as_mut_ptr(), 2);
    if res == 0 && !pdata.is_null() {
        unsafe { *pdata = u16::from_be_bytes(buf) };
    }
    res
}

#[no_mangle]
pub extern "C" fn VL53L1_RdDWord(dev: u16, index: u16, pdata: *mut u32) -> i8 {
    let mut buf = [0u8; 4];
    let res = VL53L1_ReadMulti(dev, index, buf.as_mut_ptr(), 4);
    if res == 0 && !pdata.is_null() {
        unsafe { *pdata = u32::from_be_bytes(buf) };
    }
    res
}

#[no_mangle]
pub extern "C" fn VL53L1_WaitMs(dev: u16, wait_ms: i32) -> i8 {
    thread::sleep(Duration::from_millis(wait_ms as u64));
    0
}

