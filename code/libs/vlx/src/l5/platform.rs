#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::ffi::c_void;
use std::{thread, time::Duration};

pub const VL53L5CX_NB_TARGET_PER_ZONE: u8 = 1;

#[repr(C)]
pub struct VL53L5CX_Platform {
    pub address: u16,
    pub self_ptr: *mut c_void,
    pub write_i2c: fn (self_ptr: *mut c_void, reg_addr: u16, data: *const u8, size: usize) -> bool,
    pub read_i2c: fn (self_ptr: *mut c_void, reg_addr: u16, data: *mut u8, size: usize) -> bool,
}


#[no_mangle]
pub extern "C" fn VL53L5CX_WrByte(platform: &mut VL53L5CX_Platform, register_addr: u16, value: u8) -> u8 {
    VL53L5CX_WrMulti(platform, register_addr, &value, 1)
}

#[no_mangle]
pub extern "C" fn VL53L5CX_WrMulti(platform: &mut VL53L5CX_Platform, register_addr: u16, p_values: *const u8, size: u32) -> u8 {
    let buffer = unsafe {
        core::slice::from_raw_parts(p_values, size as usize)
    };
    match crate::call_i2c_write(platform.address as u8, register_addr, buffer) {
        true => 0,
        false => 0xff,
    }
}

#[no_mangle]
pub extern "C" fn VL53L5CX_RdByte(platform: &mut VL53L5CX_Platform, register_addr: u16, p_value: *mut u8) -> u8 {
    VL53L5CX_RdMulti(platform, register_addr, p_value, 1)
}

#[no_mangle]
pub extern "C" fn VL53L5CX_RdMulti(platform: &mut VL53L5CX_Platform, register_addr: u16, p_values: *mut u8, size: u32) -> u8 {
    let buffer = unsafe {
        core::slice::from_raw_parts_mut(p_values, size as usize)
    };
    match crate::call_i2c_read(platform.address as u8, register_addr, buffer) {
        true => 0,
        false => 0xff,
    }
}

#[no_mangle]
pub extern "C" fn VL53L5CX_WaitMs(_platform: &mut VL53L5CX_Platform, time_ms: u32) -> u8 {
    thread::sleep(Duration::from_millis(time_ms as u64));

    0
}

#[no_mangle]
pub extern "C" fn VL53L5CX_SwapBuffer(buffer: *mut u8, size: u16) {
    // Convert raw pointer to slice safely
    let buffer_slice = unsafe {
        core::slice::from_raw_parts_mut(buffer, size as usize)
    };

    for chunk in buffer_slice.chunks_exact_mut(4) {
        let tmp = u32::from_be_bytes([
            chunk[0],
            chunk[1], 
            chunk[2],
            chunk[3]
        ]);
        chunk.copy_from_slice(&tmp.to_ne_bytes());
    }
}
