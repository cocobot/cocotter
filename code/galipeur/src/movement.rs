use std::{sync::{Arc, Mutex}, thread, time::Duration};
use asserv::holonomic::conf::*;
use asserv::holonomic::Asserv;
use sch16t::Sch16t;
use crate::shared_gpio::SharedGpioPin;

use super::{ImuSpi, Motor};

pub struct Movement {
    asserv: Arc<Mutex<Asserv<MovementLowLevelHardware>>>,
}

impl Movement {
    pub fn new(asserv_low_level: MovementLowLevelHardware) -> Self {
        //init asserv module
        let mut asserv = Asserv::new(asserv_low_level);
        asserv.set_conf(AsservConf {
            pid_x: PidConf {
                gain_p: 50,
                gain_i: 1,
                gain_d: 0,
                max_in: 0,
                max_i: 1000,
                max_out: 0,
                out_shift: 0,
            },
            pid_y: PidConf {
                gain_p: 50,
                gain_i: 1,
                gain_d: 0,
                max_in: 0,
                max_i: 1000,
                max_out: 0,
                out_shift: 0,
            },
            pid_a: PidConf {
                gain_p: 50,
                gain_i: 1,
                gain_d: 0,
                max_in: 0,
                max_i: 1000,
                max_out: 150000,
                out_shift: 0,
            },
            trajectory: TrajectoryConf {
                a_speed: 3.14 * 200.0,
                a_acc: 3.14 * 10.0,
                xy_cruise_speed: 10.0,
                xy_cruise_acc: 0.2,
                xy_steering_speed: 4.0,
                xy_steering_acc: 0.2,
                xy_stop_speed: 3.0,
                xy_stop_acc: 0.1,
                xy_steering_window: 50.0,
                xy_stop_window: 10.0,
                a_stop_window: 0.1,
                autoset_speed: 0.0,
                autoset_wait: 0,
                autoset_duration: 0,
            },
            motors: MotorsConf {
                matrix: [
                    0.137193775559,     -0.227742535811,    32.7587578324,
                    -0.267514745628,    0.000225842067981,  32.2910980339,
                    0.138273262887,     0.235015679279,     32.2670974911,
                ],
                inv_matrix:[
                    -1.24627114282,     2.4735001584,       -1.21007913871,
                    2.15287736186,      -0.0169008404017,   -2.16876778164,
                    -0.0103397573436,   -0.010476522571,    -0.0100097003094,
                ],
            }
        });
        
        let asserv = Arc::new(Mutex::new(asserv));

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

    motor_reset: SharedGpioPin,
    motors: [(Motor, SharedGpioPin); 3],
    last_encoder_value: Option<[i32; 3]>,
}

impl MovementLowLevelHardware {
    pub fn new(gyro: Sch16t<ImuSpi>, motor_reset: SharedGpioPin, motors: [(Motor, SharedGpioPin); 3]) -> Self {
        Self {
            gyro,
            gyro_last_angle: None,
            gyro_is_in_error: false,

            motor_reset,
            motors,
            last_encoder_value: None,
        }
    }   
}

impl AsservHardware for MovementLowLevelHardware {
    fn set_motors_break(&mut self, enable: bool) {
        log::info!("Set motors break: {}", enable);
    }

    fn set_motor_consigns(&mut self, values: [f32; 3]) {

        for i in 0..3 {
            if values[i] >= 0.0 {
                self.motors[i].0.dir.set_duty(4095).ok();
            }    
            else {
                self.motors[i].0.dir.set_duty(0).ok();
            }

            self.motors[i].0.pwm.set_duty(values[i].abs().clamp(0.0, 4095.0) as u32).ok();
        }
    }

    fn get_motor_offsets(&mut self) -> [f32; 3] {
        let new_encoder_offsets = [
            self.motors[0].0.encoder.get_value(),
            self.motors[1].0.encoder.get_value(),
            self.motors[2].0.encoder.get_value(),
        ];

        let new_encoder_offsets = {
            if let (Ok(v0), Ok(v1), Ok(v2)) = (new_encoder_offsets[0], new_encoder_offsets[1], new_encoder_offsets[2]) {
                [v0, -v1, -v2]
            } else {
                log::error!("Error reading encoder values");
                return [0.0, 0.0, 0.0];
            }
        };

        let delta_offsets = if let Some(last_values) = &self.last_encoder_value {
            [
                (new_encoder_offsets[0] - last_values[0]) as f32,
                (new_encoder_offsets[1] - last_values[1]) as f32,
                (new_encoder_offsets[2] - last_values[2]) as f32,
            ]
        }
        else {
            //first read
            [0.0, 0.0, 0.0]
        };
        self.last_encoder_value = Some(new_encoder_offsets);

        delta_offsets
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

                offset.to_radians()
            },
            None => {
                self.gyro_last_angle = Some(new_angle);

                0.0
            }
        }        
    }
}
