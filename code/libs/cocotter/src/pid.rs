
#[derive(Debug, Clone, Copy)]
pub struct PIDConfiguration {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub max_integral: f32,
    pub max_err_for_integral: f32,
}

pub struct PID {
    configuration: PIDConfiguration,

    integral: f32,
    last_error: f32,

    p_contrib: f32,
    i_contrib: f32,
    d_contrib: f32,

    output: f32,
}

impl PID {
    pub fn new(configuration: PIDConfiguration) -> PID {
        PID {
            configuration,

            integral: 0.0,
            last_error: 0.0,

            p_contrib: 0.0,
            i_contrib: 0.0,
            d_contrib: 0.0,

            output: 0.0,
        }
    }

    pub fn set_configuration(&mut self, configuration: PIDConfiguration) {
        self.configuration = configuration;
    }

    pub fn compute(&mut self, error: f32) -> f32 {
        self.integral += error;

        if self.integral > self.configuration.max_integral {
            self.integral = self.configuration.max_integral;
        }
        else if self.integral < -self.configuration.max_integral {
            self.integral = -self.configuration.max_integral;
        }

        if (error > self.configuration.max_err_for_integral) || (error < -self.configuration.max_err_for_integral) {
            //reset the integral part to prevent windup
            self.integral = 0.0;
        }

        self.p_contrib = self.configuration.kp * error;
        self.i_contrib = self.configuration.ki * self.integral;
        self.d_contrib = self.configuration.kd * (error - self.last_error);

        self.last_error = error;

        self.output = self.p_contrib + self.i_contrib + self.d_contrib;

        self.output
    }
}