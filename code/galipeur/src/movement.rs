use std::{sync::{Arc, Mutex}, thread, time::Duration};
use embedded_hal::spi::SpiDevice;
use asserv::{conf::{AsservHardware}, Asserv};
use sch16t::Sch16t;
use super::ImuSpi;

pub struct Movement {
    asserv: Arc<Mutex<Asserv<MovementLowLevelHardware>>>,
}

impl Movement {
    pub fn new(asserv_low_level: MovementLowLevelHardware) -> Self {
        //init asserv module
        let asserv = Arc::new(Mutex::new(Asserv::new(
            asserv_low_level
        )));

        //start asserv thread
        let asserv_thread = asserv.clone();
        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(10));

                let mut asserv = asserv_thread.lock().unwrap();
                asserv.update();
            }
        });

        Movement {
            asserv,
        }
    }

    pub fn get_asserv(&self) -> Arc<Mutex<Asserv<MovementLowLevelHardware>>> {
        self.asserv.clone()
    }
}


pub struct MovementLowLevelHardware {
    gyro: Sch16t<ImuSpi>,
    gyro_last_angle: Option<f32>,
    gyro_is_in_error: bool,
}

impl MovementLowLevelHardware {
    pub fn new(gyro: Sch16t<ImuSpi>) -> Self {
        Self {
            gyro,
            gyro_last_angle: None,
            gyro_is_in_error: false,
        }
    }   
}

impl AsservHardware for MovementLowLevelHardware {
    fn set_motors_break(&mut self, enable: bool) {
        log::info!("Set motors break: {}", enable);
    }

    fn set_motor_consigns(&mut self, values: [f32; 3]) {
      //  log::info!("Set motor consigns: {:?}", values);
    }

    fn get_motor_offsets(&mut self) -> [f32; 3] {
     //   log::info!("Get motor offsets");
        [0.0, 0.0, 0.0]
    }

    fn get_gyro_offset(&mut self) -> f32 {
        if let Err(e) = self.gyro.update_angle() {
            if !self.gyro_is_in_error {
                log::error!("Error updating gyro angle: {:?}", e);
                self.gyro_is_in_error = true;
            }

            return 0.0;
        }

        let new_angle = self.gyro.read_angle()[2];
        self.gyro_is_in_error = false;

        match self.gyro_last_angle {
            Some(last_angle) => {
                let offset = new_angle - last_angle;
                self.gyro_last_angle = Some(new_angle);

                offset
            },
            None => {
                self.gyro_last_angle = Some(new_angle);

                0.0
            }
        }        
    }
}