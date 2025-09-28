use crate::conf::PidConf;
use crate::maths::{PackXYA, XYA};
use crate::pid::PidFilter;
use crate::quadramp::QuadrampFilter;


pub struct MotorFilter {
    pid_x: PidFilter,
    pid_y: PidFilter,
    pid_a: PidFilter,
    qramp_a: QuadrampFilter,
}

impl MotorFilter {
    pub(crate) fn new() -> Self {
        Self {
            pid_x: PidFilter::default(),
            pid_y: PidFilter::default(),
            pid_a: PidFilter::default(),
            qramp_a: QuadrampFilter::default(),
        }
    }

    /// Filter using position (current) and target (consign), return filter output
    pub(crate) fn filter(&mut self, position: &XYA, target: &XYA) -> XYA {
        XYA {
            x: Self::filter_xy(position.x, target.x, &mut self.pid_x),
            y: Self::filter_xy(position.y, target.y, &mut self.pid_y),
            a: Self::filter_a(position.a, target.a, &mut self.pid_a, &mut self.qramp_a),
        }
    }

    fn filter_xy(position: f32, target: f32, pid: &mut PidFilter) -> f32 {
        let current = rcs_mm_to_cs_unit(position);
        let consign = rcs_mm_to_cs_unit(target);
        let out = pid.filter(consign - current);
        cs_unit_to_rcs_mm(out)
    }

    fn filter_a(position: f32, target: f32, pid: &mut PidFilter, qramp: &mut QuadrampFilter) -> f32 {
        let current = rcs_rad_to_cs_unit(position);
        let consign = rcs_rad_to_cs_unit(target);
        let consign = qramp.filter(consign);
        let out = pid.filter(consign - current);
        cs_unit_to_rcs_rad(out)
    }

    /// Reset PIDs and angle quadramp
    pub(crate) fn reset(&mut self) {
        self.pid_x.reset();
        self.pid_y.reset();
        self.pid_a.reset();
        self.qramp_a.reset_finished_to(0);  //TODO Previously, was resetting to current state
    }

    /// Provide access to PID configurations
    pub fn pid_confs_mut(&mut self) -> PackXYA<&mut PidConf> {
        PackXYA {
            x: &mut self.pid_x.conf,
            y: &mut self.pid_y.conf,
            a: &mut self.pid_a.conf,
        }
    }

    /// Set angle quadramp parameters
    pub(crate) fn set_qramp_a_vars(&mut self, speed: u32, acc: u32) {
        self.qramp_a.set_order1_vars(speed, speed);
        self.qramp_a.set_order2_vars(acc, acc);
    }
}


pub const RCS_MM_TO_CSUNIT: f32 = 1000.0;
pub const RCS_RAD_TO_CSUNIT: f32 = 10000.0;

#[inline]
pub const fn rcs_mm_to_cs_unit(v: f32) -> i32 { (v * RCS_MM_TO_CSUNIT) as i32 }
#[inline]
pub const fn cs_unit_to_rcs_mm(v: i32) -> f32 { v as f32 / RCS_MM_TO_CSUNIT }

#[inline]
pub const fn rcs_rad_to_cs_unit(v: f32) -> i32 { (v * RCS_RAD_TO_CSUNIT) as i32 }
#[inline]
pub const fn cs_unit_to_rcs_rad(v: i32) -> f32 { v as f32 / RCS_RAD_TO_CSUNIT }



