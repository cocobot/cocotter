
#[derive(Default)]
pub struct QuadrampFilter {
    order2_pos: u32,
    order2_neg: u32,
    order1_pos: u32,
    order1_neg: u32,
    previous_var: i32,
    previous_out: i32,
    previous_in: i32,
}

impl QuadrampFilter {
    //XXX Currently 'pos' and 'neg' are always equal. Are both really needed?

    pub fn set_order2_vars(&mut self, pos: u32, neg: u32) {
        self.order2_pos = pos;
        self.order2_neg = neg;
    }

    pub fn set_order1_vars(&mut self, pos: u32, neg: u32) {
        self.order1_pos = pos;
        self.order1_neg = neg;
    }

    #[allow(dead_code)]
    pub fn finished(&self) -> bool {
        self.previous_out == self.previous_in && self.previous_var == 0
    }

    #[allow(dead_code)]
    pub fn reset_finished(&mut self) {
        self.previous_var = 0;
        self.previous_out = self.previous_in;
    }

    #[allow(dead_code)]
    pub fn reset_finished_to(&mut self, consign: i32) {
        self.previous_var = 0;
        self.previous_out = consign;
        self.previous_in = consign;
    }

    pub fn filter(&mut self, new_value: i32) -> i32 {
        //TODO Store i32 to avoid overflows?
        let mut order1_pos: i32 = self.order1_pos as i32;
        let mut order1_neg: i32 = -(self.order1_neg as i32);
        let order2_pos: i32 = self.order2_pos as i32;
        let order2_neg: i32 = -(self.order2_neg as i32);
        let d = new_value - self.previous_out;

        // Deceleration ramp
        if d > 0 && order2_neg != 0 {
            // order2_neg < 0
            let ramp_pos = (order2_neg * order2_neg - 2 * d * order2_neg).isqrt() + order2_neg / 2;
            if ramp_pos < order1_pos {
                order1_pos = ramp_pos;
            }
        }
        else if d < 0 && order2_pos != 0 {
            // order2_pos > 0
            let ramp_neg = -(order2_pos * order2_pos - 2 * d * order2_pos).isqrt() - order2_pos / 2;
            if ramp_neg > order1_neg {
                order1_neg = ramp_neg;
            }
        }

        // Try to set the speed: can we reach the speed with our acceleration?
        if self.previous_var < order1_pos {
            // We are going slower than Vmax
            // Acceleration would be to high, we reduce the speed
            if order2_pos != 0 && order1_pos - self.previous_var > order2_pos {
                order1_pos = self.previous_var + order2_pos;
            }
        }
        else if self.previous_var > order1_pos {
            // We are going faster than Vmax
            // Deceleration would be to high, we increase the speed
            if order2_neg != 0 && order1_pos - self.previous_var < order2_neg {
                order1_pos = self.previous_var + order2_neg;
            }
        }

        // Same for neg
        if self.previous_var > order1_neg  {
            if order2_neg != 0 && order1_neg - self.previous_var < order2_neg {
                order1_neg = self.previous_var + order2_neg;
            }
        }
        else if self.previous_var < order1_neg {
            if order2_pos != 0 && order1_neg - self.previous_var > order2_pos {
                order1_neg = self.previous_var + order2_pos;
            }
        }

        // Can we reach the target position with our speed?
        if /* order1_pos != 0 && */ d > order1_pos {
            self.previous_var = order1_pos;
            self.previous_out += order1_pos;
        }
        else if /* order1_neg != 0 && */ d < order1_neg {
            self.previous_var = order1_neg;
            self.previous_out += order1_neg;
        }
        else {
            self.previous_var = d;
            self.previous_out += d
        }
        self.previous_in = new_value;

        self.previous_out
    }
}

