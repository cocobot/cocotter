use core::{f32::EPSILON, fmt};

#[derive(Debug, Clone, Copy)]
pub struct RampConfiguration {
    pub max_speed: f32,
    pub acceleration: f32,
}

impl RampConfiguration {
    pub fn with_timestep_ms(self, timestamp_ms: u64) -> RampConfiguration {
        let max_speed = self.max_speed * 1e-3 * (timestamp_ms as f32);
        let acceleration = self.acceleration * 1e-6 * ((timestamp_ms * timestamp_ms) as f32);

        RampConfiguration {
            max_speed,
            acceleration,
        }
    }
}

pub struct Ramp {
    configuration: RampConfiguration,

    speed_factor: f32,
    acceleration_factor: f32,

    position: f32,
    target: f32,
    speed: f32,
}

impl Ramp {
    pub fn new(configuration: RampConfiguration) -> Ramp {
        Ramp {
            configuration,

            speed_factor: 1.0,
            acceleration_factor: 1.0,

            position: 0.0,
            target: 0.0,
            speed: 0.0,
        }
    }

    pub fn get_conf_as_mut(&mut self) -> &mut RampConfiguration {
        &mut self.configuration
    }

    pub fn set_max_speed(&mut self, max_speed: f32) {
        self.configuration.max_speed = max_speed;
    }

    pub fn set_acceleration(&mut self, acceleration: f32) {
        self.configuration.acceleration = acceleration;
    }

    pub fn set_target(&mut self, target: f32, emergency: bool) {
        self.target = target;
        if emergency {
            self.position = target;
            self.speed = 0.0;
        }
    }

    pub fn get_target(&self) -> f32 {
        self.target
    }

    pub fn get_output(&self) -> f32 {
        self.position
    }

    pub fn set_max_speed_factor(&mut self, factor: f32) {
        self.speed_factor = factor;
    }

    pub fn set_acceleration_factor(&mut self, factor: f32) {
        self.acceleration_factor = factor;
    }

    pub fn compute(&mut self) -> f32 {
        let acceleration = self.configuration.acceleration * self.acceleration_factor;
        let max_speed = self.configuration.max_speed * self.speed_factor;

        //compute how much distance the robot will do if we start decreasing the speed now
        let delta_position = self.speed * self.speed / (2.0 * acceleration);

        //check if we want to move forward or backward
        let forward = (self.target - self.position) >= 0.0;
        if forward {
            //check if we need to speed up or down
            if delta_position + self.position < self.target {
                //we can increase the speed
                self.speed += acceleration;
            }
            else {
                //we should decrease the speed
                self.speed -= acceleration;
            }
        }
        else {
            //check if we need to speed up or down
            if -delta_position + self.position < self.target {
                //we should decrease the (negative) speed
                self.speed += acceleration;
            }
            else {
                //we can increase the (negative) speed
                self.speed -= acceleration;
            }
        }


        //set speed limit (because of cops !)
        self.speed = self.speed.clamp(-max_speed, max_speed);

        //compute next output
        let mut output = if forward {
            (self.position + self.speed).min(self.target)
        }
        else {
            (self.position + self.speed).max(self.target)
        };

        // log::info!("output: {}", output);
        let mut abs_diff : f32 = self.target - self.position;

        if !forward{
            abs_diff = - abs_diff;
        }

        //prevent position overshoot because of discrete speed
        if abs_diff < 0.001 {
            //we are close enough to the target
            output = self.target;
            self.speed = 0.0;
        }
        if forward && (self.speed > 0.0) {
            if output > self.target {
                output = self.target;
                self.speed = 0.0;
            }
        }
        else if !forward && (self.speed < 0.0) {
            if output < self.target {
                output = self.target;
                self.speed = 0.0;
            }
        }
        
        //assign output
        self.position = output;
        output
    }

    pub fn is_done(&self) -> bool {
        let diff: f32 = self.position - self.target;
        diff.abs() <= EPSILON
    }

}

impl fmt::Debug for Ramp {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Ramp (position: {}, target {}, speed {})", self.position, self.target, self.speed)
    }
}