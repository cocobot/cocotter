
#[derive(Default)]
pub struct RampFilter {
    max_speed: f32,
    acceleration: f32,
    position: f32,
    speed: f32,
}

impl RampFilter {
    /// Configure using raw values (unit-less)
    pub fn configure_raw(&mut self, max_speed: f32, acceleration: f32) {
        self.max_speed = max_speed;
        self.acceleration = acceleration;
    }

    /// Configure using in s⁻̂¹, s⁻², and step duration values
    pub fn configure_scaled(&mut self, max_speed: f32, acceleration: f32, step_secs: f32) {
        self.configure_raw(max_speed * step_secs, acceleration * step_secs * step_secs);
    }

    /// Return filtered target position
    pub fn filter(&mut self, target: f32) -> f32 {
        // Compute how much distance the robot will do if we start decreasing the speed now
        let delta_position = self.speed * self.speed / (2.0 * self.acceleration);

        // Check if we want to move forward or backward
        let forward = (target - self.position) >= 0.0;
        let abs_diff: f32 = (target - self.position).abs();

        let delta_position = if forward { delta_position } else { -delta_position };
        // Check if we need to speed up or down
        if self.position + delta_position < target {
            // We can increase the speed
            self.speed += self.acceleration;
        } else {
            // We should decrease the speed
            self.speed -= self.acceleration;
        }

        // Set speed limit
        self.speed = self.speed.clamp(-self.max_speed, self.max_speed);

        // Compute next output
        let mut output = if forward {
            (self.position + self.speed).min(target)
        }
        else {
            (self.position + self.speed).max(target)
        };

        // Prevent position overshoot because of discrete speed
        let overshoot = if forward {
            self.speed > 0.0 && output > target
        } else {
            self.speed < 0.0 && output < target
        };
        if abs_diff < 0.001 || overshoot {
            output = target;
            self.speed = 0.0;
        }

        // Assign output
        self.position = output;
        output
    }

    #[allow(dead_code)]
    pub fn reset_finished_to(&mut self, consign: f32) {
        self.position = consign;
        self.speed = 0.0;
    }
}

