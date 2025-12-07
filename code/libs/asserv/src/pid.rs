use crate::conf::PidConf;


#[derive(Default)]
pub struct PidFilter {
    pub conf: PidConf,
    /// Previous sample value
    prev_sample: i32,
    /// Previous integral parameter
    integral: i32,
}

impl PidFilter {
    pub fn reset(&mut self) {
        self.prev_sample = 0;
        self.integral = 0;
    }

    pub fn filter(&mut self, mut value_in: i32) -> i32 {
        // Integral value: the integral becomes bigger with time (think to area of graph, we add
        // one area to the previous) so, integral = previous integral + current value
        //
        // derivate value (h = 1 with current implementation)
        //             f(t+h) - f(t)        with f(t+h) = current value
        //  derivate = -------------             f(t)   = previous value
        //                    h
        // so derivate = current error - previous error
        //
        // We can apply a filter to reduce noise on the derivate term, by using a bigger period.

        // Saturate input... it influences integral and derivate
        if self.conf.max_in != 0 {
            value_in = value_in.clamp(-self.conf.max_in, self.conf.max_in);
        }

        let derivate = value_in - self.prev_sample;
        self.integral += value_in;
        if self.conf.max_i != 0 {
            self.integral = self.integral.clamp(-self.conf.max_i, self.conf.max_i);
        }

        // So, command = P.coef_P + I.coef_I + D.coef_D
        let mut command = value_in * self.conf.gain_p as i32 +
            self.integral * self.conf.gain_i as i32 +
            derivate * self.conf.gain_d as i32;
        if command < 0 {
            command = -( command.saturating_neg() >> self.conf.out_shift );
        } else {
            command = command >> self.conf.out_shift;
        }

        if self.conf.max_out != 0 {
            command = command.clamp(-self.conf.max_out, self.conf.max_out);
        }

        // Backup of current error value (for the next calcul of derivate value)
        self.prev_sample = value_in;

        command
    }
}

