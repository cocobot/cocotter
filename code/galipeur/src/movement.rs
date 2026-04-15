use std::time::Duration;
use asserv::holonomic::conf::*;
use board_sabotter::{Encoder, SabotterBoard, SabotterMotor};
use embedded_hal::pwm::SetDutyCycle;
use sch16t::Sch16t;

pub const ASSERV_PERIOD: Duration = Duration::from_millis(15);


pub struct MovementLowLevelHardware<B: SabotterBoard> {
    gyro: Sch16t<B::Spi>,
    gyro_last_angle: Option<f32>,
    gyro_is_in_error: bool,

    motors: [SabotterMotor<B::MotorEncoder, B::MotorPwm>; 3],
    last_encoder_values: Option<[i32; 3]>,
}

impl<B: SabotterBoard> MovementLowLevelHardware<B> {
    pub fn new(gyro: Sch16t<B::Spi>, motors: [SabotterMotor<B::MotorEncoder, B::MotorPwm>; 3]) -> Self {
        Self {
            gyro,
            gyro_last_angle: None,
            gyro_is_in_error: false,
            motors,
            last_encoder_values: None,
        }
    }

    fn get_offsets_option(&self) -> Option<[i32; 3]> {
        Some([
            self.motors[0].encoder.get_value().ok()?,
            self.motors[1].encoder.get_value().ok()?,
            self.motors[2].encoder.get_value().ok()?,
        ])
    }
}

impl<B: SabotterBoard> AsservHardware for MovementLowLevelHardware<B> {
    fn set_motors_break(&mut self, enable: bool) {
        log::info!("Set motors break: {}", enable);
    }

    fn set_motor_consigns(&mut self, values: [f32; 3]) {
        for i in 0..3 {
            if values[i] >= 0.0 {
                self.motors[i].dir.set_duty_cycle(4095).ok();
            } else {
                self.motors[i].dir.set_duty_cycle(0).ok();
            }
            self.motors[i].pwm.set_duty_cycle(values[i].abs().clamp(0.0, 4095.0) as u16).ok();
        }
    }

    fn get_motor_offsets(&mut self) -> [f32; 3] {
        let new_offsets = {
            if let Some([v0, v1, v2]) = self.get_offsets_option() {
                [v0, -v1, -v2]
            } else {
                log::error!("Error reading encoder values");
                return [0.0, 0.0, 0.0];
            }
        };

        let delta_offsets = if let Some(last_values) = &self.last_encoder_values {
            [
                (new_offsets[0] - last_values[0]) as f32,
                (new_offsets[1] - last_values[1]) as f32,
                (new_offsets[2] - last_values[2]) as f32,
            ]
        } else {
            //first read
            [0.0, 0.0, 0.0]
        };
        self.last_encoder_values = Some(new_offsets);

        delta_offsets
    }

    fn get_gyro_offset(&mut self) -> f32 {
        if let Err(e) = self.gyro.update_angle_z() {
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
