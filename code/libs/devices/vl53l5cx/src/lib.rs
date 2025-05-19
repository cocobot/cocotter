use core::ffi::c_void;
use std::pin::Pin;

use bindings::{vl53l5cx_check_data_ready, vl53l5cx_get_ranging_data, vl53l5cx_init, vl53l5cx_is_alive, vl53l5cx_set_ranging_frequency_hz, vl53l5cx_set_resolution, vl53l5cx_start_ranging, VL53L5CX_Configuration, VL53L5CX_Platform, VL53L5CX_ResultsData, VL53L5CX_ResultsData__bindgen_ty_1};
use embedded_hal::i2c::{I2c, Operation};

mod bindings;
pub mod platform;

#[derive(Debug)]
pub enum VlErrors {
    NoDevice,
    InitFailed,
    InternalError,
    NoMeasure,
}

pub struct Vl53l5cx<I2C> {
    i2c: I2C,
    i2c_addr: u8,

    binding_configuration: Pin<Box<VL53L5CX_Configuration>>,
}

impl<I2C, E> Vl53l5cx<I2C>
where
    I2C: I2c<Error = E>,
{
    const DEFAULT_I2C_ADDR: u8 = 0x29;

    pub fn new(i2c: I2C, i2c_addr: Option<u8>) -> Self {
        let mut instance = Vl53l5cx {
            i2c,
            i2c_addr: Self::DEFAULT_I2C_ADDR,

            
            binding_configuration: Box::into_pin(Box::new(VL53L5CX_Configuration {
                platform: VL53L5CX_Platform {
                    address: (i2c_addr.unwrap_or(Self::DEFAULT_I2C_ADDR)) as u16,
                    self_ptr: core::ptr::null_mut(),
                    write_i2c: Some(Self::write_i2c),
                    read_i2c: Some(Self::read_i2c),
                },
                streamcount: 0,
                data_read_size: 0,
                default_configuration: core::ptr::null_mut(),
                default_xtalk: core::ptr::null_mut(),
                offset_data: [0; 488],
                xtalk_data: [0; 776],
                temp_buffer: [0; 1452],
                is_auto_stop_enabled: 0,
            }))
        };

        let unsafe_self_ptr = &mut instance as *mut Vl53l5cx<I2C> as *mut c_void;
        instance.binding_configuration.platform.self_ptr = unsafe_self_ptr;

        instance
    }

    extern "C" fn write_i2c(self_ptr: *mut c_void, reg_addr: u16, data: *const u8, size: usize) -> bool {
        let self_instance = unsafe { &mut *(self_ptr as *mut Vl53l5cx<I2C>) };
        let data = unsafe { core::slice::from_raw_parts(data, size) };

        self_instance.i2c.transaction(self_instance.i2c_addr,  &mut [
            Operation::Write(&reg_addr.to_be_bytes()),
            Operation::Write(data),
        ]).is_ok()
    }

    extern "C" fn read_i2c(self_ptr: *mut c_void, reg_addr: u16, data: *mut u8, size: usize) -> bool {
        let self_instance = unsafe { &mut *(self_ptr as *mut Vl53l5cx<I2C>) };
        let data = unsafe { core::slice::from_raw_parts_mut(data, size) };

        self_instance.i2c.transaction(self_instance.i2c_addr, &mut [
            Operation::Write(&reg_addr.to_be_bytes()),
            Operation::Read(data),
        ]).is_ok()
    }

    pub fn init(&mut self) -> Result<(), VlErrors> {
        let mut is_alive = 0;

        let unsafe_self_ptr = &mut (*self) as *mut Vl53l5cx<I2C> as *mut c_void;
        self.binding_configuration.platform.self_ptr = unsafe_self_ptr;

        //check if device is connected
        let status = unsafe {
            vl53l5cx_is_alive(&mut *self.binding_configuration, &mut is_alive)
        };
        if (is_alive == 0) || (status != 0) {
            return Err(VlErrors::NoDevice)
        }

        //try to init device
        let status = unsafe {
            vl53l5cx_init(&mut *self.binding_configuration)
        };
        if status != 0 {
            return Err(VlErrors::InitFailed)
        }

        //set 8x8 mode
        let status = unsafe {
            vl53l5cx_set_resolution(&mut *self.binding_configuration, 64)
        };
        if status != 0 {
            return Err(VlErrors::InternalError)
        };

        //set 60Hz mode
        let status = unsafe {
            vl53l5cx_set_ranging_frequency_hz(&mut *self.binding_configuration, 60)
        };
        if status != 0 {
            return Err(VlErrors::InternalError)
        };

        //start ranging
        let status = unsafe {
            vl53l5cx_start_ranging(&mut *self.binding_configuration)
        };
        if status != 0 {
            return Err(VlErrors::InternalError)
        }

        Ok(())
    }

    pub fn get_distance(&mut self) -> Result<[[i16; 8]; 8], VlErrors> {
        let unsafe_self_ptr = &mut (*self) as *mut Vl53l5cx<I2C> as *mut c_void;
        self.binding_configuration.platform.self_ptr = unsafe_self_ptr;
        
        let mut is_ready = 0;
        let status = unsafe {
            vl53l5cx_check_data_ready(&mut *self.binding_configuration, &mut is_ready)
        };
        if (is_ready == 0) || (status != 0) {
            return Err(VlErrors::NoMeasure)
        }

        let mut results= VL53L5CX_ResultsData {
            silicon_temp_degc: 0,
            ambient_per_spad: [0; 64],
            nb_target_detected: [0; 64],
            nb_spads_enabled: [0; 64],
            signal_per_spad: [0; 64],
            range_sigma_mm: [0; 64],
            distance_mm: [0; 64],
            reflectance: [0; 64],
            target_status: [0; 64],
            motion_indicator: VL53L5CX_ResultsData__bindgen_ty_1 {
                global_indicator_1: 0,
                global_indicator_2: 0,
                status: 0,
                nb_of_detected_aggregates: 0,
                nb_of_aggregates: 0,
                spare: 0,
                motion: [0; 32],
            },
        };
        let status = unsafe {
            vl53l5cx_get_ranging_data(&mut *self.binding_configuration, &mut results)
        };
        if status != 0 {
            return Err(VlErrors::InternalError)
        }

        let mut result_2d = [[0; 8]; 8];

        for x in 0..8 {
            for y in 0..8 {
                result_2d[x][y] = results.distance_mm[x + y * 8];
            }
        }

        Ok(result_2d)
    }
}

unsafe impl<I2C> Send for Vl53l5cx<I2C> {}
unsafe impl<I2C> Sync for Vl53l5cx<I2C> {}