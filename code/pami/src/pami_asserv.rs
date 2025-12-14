use std::time::Duration;
use asserv::differential::conf::AsservHardware;
use board_pami::{Encoder, PamiBoard, PamiMotor, PamiMotors};
use embedded_hal::{
    digital::InputPin,
    pwm::SetDutyCycle,
};

pub const ASSERV_PERIOD: Duration = Duration::from_millis(15);


type PamiMotorImpl<B> = PamiMotor<<B as PamiBoard>::MotorEncoder, <B as PamiBoard>::MotorPwm>;
type PamiMotorsImpl<B> = PamiMotors<<B as PamiBoard>::MotorEncoder, <B as PamiBoard>::MotorPwm>;

pub struct PamiAsservHardware<B: PamiBoard> {
    emergency_stop: B::EmergencyStop,
    motors: PamiMotorsImpl<B>,
    last_encoder_reads: [i32; 2],
}

impl<B: PamiBoard> PamiAsservHardware<B> {
    pub fn new(board: &mut B) -> Self {
        Self {
            emergency_stop: board.emergency_stop().unwrap(),
            motors: board.motors().unwrap(),
            last_encoder_reads: [0; 2],
        }
    }

    fn set_duty_cycle(motor: &mut PamiMotorImpl<B>, value: f32) {
        const SPEED_LIMIT: f32 = 1000.0;
        const PWM_MAX: u16 = 1023;
        assert!(PWM_MAX < i16::MAX as u16);
        assert!(SPEED_LIMIT < PWM_MAX as f32);

        let value = value.clamp(-SPEED_LIMIT, SPEED_LIMIT) as i16;
        let pwm_value = value.unsigned_abs();
        assert!(pwm_value < PWM_MAX);
        if value >= 0 {
            let _ = motor.pwm_forward.set_duty_cycle(PWM_MAX);
            let _ = motor.pwm_backward.set_duty_cycle(PWM_MAX - pwm_value);
        } else {
            let _ = motor.pwm_backward.set_duty_cycle(PWM_MAX);
            let _ = motor.pwm_forward.set_duty_cycle(PWM_MAX - pwm_value);
        }
    }

    fn get_encoder_delta(motor: &mut PamiMotorImpl<B>, last_read: &mut i32) -> f32 {
        let new_read: i32 = motor.encoder.get_value().unwrap();  //TODO Avoid unwrap()?
        // Use wrapping to handle encoder overflow
        let delta = -new_read.wrapping_sub(*last_read);
        *last_read = new_read;
        delta as f32
    }
}

impl<B: PamiBoard> AsservHardware for PamiAsservHardware<B> {
    fn emergency_stop_active(&mut self) -> bool {
        //TODO Fallback to true? or false?
        InputPin::is_low(&mut self.emergency_stop).unwrap_or(true)
    }

    fn set_motor_consigns(&mut self, values: [f32; 2]) {
        Self::set_duty_cycle(&mut self.motors.left, values[0]);
        Self::set_duty_cycle(&mut self.motors.right, values[1]);
    }

    fn get_motor_offsets(&mut self) -> [f32; 2] {
        [
            Self::get_encoder_delta(&mut self.motors.left, &mut self.last_encoder_reads[0]),
            Self::get_encoder_delta(&mut self.motors.right, &mut self.last_encoder_reads[1]),
        ]
    }
}

