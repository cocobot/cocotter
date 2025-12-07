use crate::maths::{mult_matrix33_vec, Matrix33, MATRIX33_IDENTITY, XY, XYA};
use super::conf::AsservHardware;
use super::motor_filter::MotorFilter;


pub struct ControlSystem<H: AsservHardware> {
    pub(crate) hardware: H,
    pub(crate) motor_filter: MotorFilter,
    position: XYA,
    target: XYA,
    motor_control: bool,
    motors_reactivated: bool,
    motors_matrix: Matrix33,
    motors_inv_matrix: Matrix33,
}


impl<H: AsservHardware> ControlSystem<H> {
    pub(crate) fn new(hardware: H) -> Self {
        Self {
            hardware,
            motor_filter: MotorFilter::new(),
            position: XYA::default(),
            target: XYA::default(),
            motor_control: true,
            motors_reactivated: false,
            motors_matrix: MATRIX33_IDENTITY,
            motors_inv_matrix: MATRIX33_IDENTITY,
        }
    }

    pub(crate) fn update(&mut self) {
        self.update_position();
        self.update_motors();
    }

    fn update_position(&mut self) {
        let motor_offsets = self.hardware.get_motor_offsets();
        let gyro_offset = self.hardware.get_gyro_offset();

        // Convert speed from encoders coordinates to robot coordinates
        let dp = mult_matrix33_vec(&self.motors_inv_matrix, &motor_offsets);
        let dp = XY::new(dp[0], dp[1]);

        //TODO Why this? Shouldn't be included in matrix?!
        // Scale units
        let dp = dp / 1000.0;

        // Integrate speed in robot coordinates to position
        let cos_a = self.position.a.cos();
        let sin_a = self.position.a.sin();
        self.position.x += dp.x * cos_a - dp.y * sin_a;
        self.position.y += dp.x * sin_a + dp.y * cos_a;
        self.position.a += gyro_offset;
    }

    fn update_motors(&mut self) {
        if !self.motor_control {
            return;
        }

        // If CS was previously inactive, we need a little hack for quadramps
        if self.motors_reactivated {
            self.motor_filter.reset();
            self.motors_reactivated = false;
        }

        // Compute control system first level (x,y,a)
        let velocity = self.motor_filter.filter(&self.position, &self.target);

        // Transform output velocity vector from table coords to robot coords
        let alpha = -self.position.a;
        let cos_a = alpha.cos();
        let sin_a = alpha.sin();
        let vx_r = velocity.x * cos_a - velocity.y * sin_a;
        let vy_r = velocity.x * sin_a + velocity.y * cos_a;

        // Set second level consigns
        self.set_motors_from_velocities(vx_r, vy_r, velocity.a);
    }

    /// Return current position
    pub fn position(&self) -> &XYA {
        &self.position
    }

    /// Reset robot position, and target (but not consigns)
    ///
    /// This method should only be called when robot is not moving.
    pub fn reset_position(&mut self, xya: XYA) {
        self.position = xya;
        self.target = xya;
        self.motor_filter.reset();
    }

    /// Set target position (consign)
    pub fn set_target_position(&mut self, target: XYA) {
        self.target = target;
    }

    /// Set target linear position (consign)
    pub fn set_target_xy(&mut self, x: f32, y: f32) {
        self.target.x = x;
        self.target.y = y;
    }

    /// Set target angle (consign)
    pub fn set_target_a(&mut self, a: f32) {
        self.target.a = a;
    }

    /// Enable or disable break on motors
    pub fn set_motors_break(&mut self, enabled: bool) {
        self.hardware.set_motors_break(enabled);
    }

    /// Set motors duty cycles from linear and angular velocities
    pub fn set_motors_from_velocities(&mut self, vx: f32, vy: f32, va: f32) {
        let values = mult_matrix33_vec(&self.motors_matrix, &[vx, vy, va]);
        self.hardware.set_motor_consigns(values);
    }

    /// Enable motor control
    pub fn enable_motor_control(&mut self) {
        self.motors_reactivated = true;
        self.motor_control = true;
    }

    /// Disable motor control
    pub fn disable_motor_control(&mut self) {
        // Clear previous motors consign
        self.stop_motors();
        self.motor_control = false;
    }

    /// Stop motors, set them to null velocities but don't disable them
    pub fn stop_motors(&mut self) {
        self.set_motors_from_velocities(0.0, 0.0, 0.0);
    }

    /// Set angular speed and acceleration
    pub fn set_a_speed(&mut self, speed: f32, acc: f32) {
        self.motor_filter.set_qramp_a_vars(speed as u32, acc as u32);
    }

    /// Set matrix that converts velocities to motor consigns
    pub fn set_motors_matrix(&mut self, matrix: Matrix33) {
        self.motors_matrix = matrix;
    }

    /// Set matrix that converts motor offsets to position offsets
    pub fn set_motors_inv_matrix(&mut self, matrix: Matrix33) {
        self.motors_inv_matrix = matrix;
    }
}

