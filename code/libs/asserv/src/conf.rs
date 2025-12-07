
#[derive(Clone)]
pub struct PidConf {
    /// Gain of Proportionnal module
    pub gain_p: i16,
    /// Gain of Integral module
    pub gain_i: i16,
    /// Gain of Derivate module
    pub gain_d: i16,

    /// In saturation levels
    pub max_in: i32,
    /// Integral saturation levels
    pub max_i: i32,
    /// Out saturation levels
    pub max_out: i32,

    /// Big common divisor for output
    pub out_shift: u8,
}

impl Default for PidConf {
    fn default() -> Self {
        Self {
            gain_p: 1,
            gain_i: 0,
            gain_d: 0,
            max_in: 0,
            max_i: 0,
            max_out: 0,
            out_shift: 0,
        }
    }
}

impl PidConf {
    pub fn set_gains(&mut self, gp: i16, gi: i16, gd: i16) {
        self.gain_p = gp;
        self.gain_i = gi;
        self.gain_d = gd;
    }

    pub fn set_maximums(&mut self, max_in: i32, max_i: i32, max_out: i32) {
        self.max_in = max_in;
        self.max_i = max_i;
        self.max_out = max_out;
    }
}

