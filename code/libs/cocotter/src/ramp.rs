use core::fmt;

pub struct RampConfiguration {
    pub max_speed: f32,
    pub acceleration: f32,
}

impl RampConfiguration {
    pub fn with_timestep_ms(self, timestamp_ms: u64) -> RampConfiguration {
        let max_speed = self.max_speed / (timestamp_ms as f32 / 1000.0);
        let acceleration = self.acceleration / ((timestamp_ms as f32 / 1000.0) * (timestamp_ms as f32 / 1000.0));

        RampConfiguration {
            max_speed,
            acceleration,
        }
    }
}

pub struct Ramp {
    configuration: RampConfiguration,

    position: f32,
    target: f32,
    speed: f32,
}

impl Ramp {
    pub fn new(configuration: RampConfiguration) -> Ramp {
        Ramp {
            configuration,

            position: 0.0,
            target: 0.0,
            speed: 0.0,
        }
    }

    pub fn set_max_speed(&mut self, max_speed: f32) {
        self.configuration.max_speed = max_speed;
    }

    pub fn set_acceleration(&mut self, acceleration: f32) {
        self.configuration.acceleration = acceleration;
    }

    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    pub fn get_output(&self) -> f32 {
        self.position
    }

    pub fn compute(&mut self) -> f32 {
        //compute how much distance the robot will do if we start decreasing the speed now
        let delta_position = self.speed * self.speed / (2.0 * self.configuration.acceleration);

        //check if we want to move forward or backward
        let forward = (self.target - self.position) >= 0.0;
        if forward {
            //check if we need to speed up or down
            if delta_position + self.position < self.target {
                //we can increase the speed
                self.speed += self.configuration.acceleration;
            }
            else {
                //we should decrease the speed
                self.speed -= self.configuration.acceleration;
            }
        }
        else {
            //check if we need to speed up or down
            if -delta_position + self.position < self.target {
                //we should decrease the (negative) speed
                self.speed += self.configuration.acceleration;
            }
            else {
                //we can increase the (negative) speed
                self.speed -= self.configuration.acceleration;
            }
        }

        //set speed limit (because of cops !)
        self.speed = self.speed.clamp(-self.configuration.max_speed, self.configuration.max_speed);

        //compute next output
        let mut output = self.position + self.speed;

        //prevent position overshoot because of discrete speed
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
}

impl fmt::Debug for Ramp {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Ramp (position: {}, target {}, speed {})", self.position, self.target, self.speed)
    }
}