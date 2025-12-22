use core::time::Duration;
use crate::maths::XYA;
use super::motor_filter::MotorFilter;
use super::AsservHardware;


pub struct ControlSystem<H: AsservHardware> {
    pub(crate) hardware: H,
    pub(crate) motor_filter: MotorFilter,

    position: XYA,
    dist: f32,

    target_dist: f32,
    target_angle: f32,

    // Current speed, only for debug, not used internally
    speed_dist: f32,
    speed_angle: f32,

    tick_to_mm: f32,
    tick_to_rad: f32,
}

impl<H: AsservHardware> ControlSystem<H> {
    pub(crate) fn new(hardware: H) -> Self {
        Self {
            hardware,
            motor_filter: MotorFilter::new(),
            position: XYA::default(),
            dist: 0.0,
            target_dist: 0.0,
            target_angle: 0.0,
            speed_dist: 0.0,
            speed_angle: 0.0,
            // Note: those values will never be correct
            tick_to_mm: 1.0,
            tick_to_rad: 1.0,
        }
    }

    pub(crate) fn update(&mut self, elapsed: &Duration) {
        self.update_position(elapsed);
        self.update_motors();
    }

    fn update_position(&mut self, elapsed: &Duration) {
        assert!(*elapsed > Duration::ZERO);

        // Update encoder values
        let encoder_offsets = self.hardware.get_motor_offsets();
        let d_dist = (encoder_offsets[0] + encoder_offsets[1]) / 2.0 * self.tick_to_mm;
        let d_angle = (encoder_offsets[1] - encoder_offsets[0]) / 2.0 * self.tick_to_rad;

        let elapsed_secs = elapsed.as_secs_f32();
        self.speed_dist = d_dist / elapsed_secs;
        self.speed_angle = d_angle / elapsed_secs;

        self.dist += d_dist;
        self.position.a += d_angle;
        self.position.x += self.position.a.cos() * d_dist;
        self.position.y += self.position.a.sin() * d_dist;
    }

    fn update_motors(&mut self) {
        let speeds = if self.hardware.emergency_stop_active() {
            self.reset_targets();
            [0.0, 0.0]
        } else {
            let (dist_speed, angle_speed) = self.motor_filter.filter(self.dist, self.position.a, self.target_dist, self.target_angle);

            // Assign the control loop output to the motors
            let left_speed = dist_speed - angle_speed;
            let right_speed = dist_speed + angle_speed;
            // Note: speed is clamped in `set_motor_consigns()`, if needed
            [left_speed, right_speed]
        };

        self.hardware.set_motor_consigns(speeds);
    }

    /// Return current position
    pub fn position(&self) -> &XYA {
        &self.position
    }

    /// Return current distance
    pub fn dist(&self) -> f32 {
        self.dist
    }

    /// Return current linear and angular speeds (in this order)
    pub fn speeds(&self) -> (f32, f32) {
        (self.speed_dist, self.speed_angle)
    }

    /// Reset robot position, and target (but not consigns)
    ///
    /// This method should only be called when robot is not moving.
    pub fn reset_position(&mut self, xya: XYA) {
        self.position = xya;
        self.dist = 0.0;
        self.target_dist = self.dist;
        self.target_angle = self.position.a;
        self.speed_dist = 0.0;
        self.speed_angle = 0.0;
        self.motor_filter.reset();
    }

    /// Reset current targets to current position
    pub fn reset_targets(&mut self) {
        self.target_dist = self.dist;
        self.target_angle = self.position.a;
    }

    /// Set target distance (consign)
    pub fn set_target_dist(&mut self, dist: f32) {
        self.target_dist = dist;
    }

    /// Set target angle (consign)
    pub fn set_target_a(&mut self, a: f32) {
        self.target_angle = a;
    }

    /// Set linear speed and acceleration
    pub fn set_xy_speed(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.motor_filter.set_dist_ramp_conf(speed, acc, time_step);
    }

    /// Set angular speed and acceleration
    pub fn set_a_speed(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.motor_filter.set_angle_ramp_conf(speed, acc, time_step);
    }

    /// Set encoder tick ratios
    pub fn set_encoder_conversion(&mut self, tick_to_mm: f32, tick_to_rad: f32) {
        self.tick_to_mm = tick_to_mm;
        self.tick_to_rad = tick_to_rad;
    }
}

