use core::time::Duration;
use crate::conf::PidConf;
use crate::pid::PidFilter;
use crate::ramp::RampFilter;


pub struct MotorFilter {
    ramp_dist: RampFilter,
    ramp_angle: RampFilter,
    pid_dist: PidFilter,
    pid_angle: PidFilter,
}

impl MotorFilter {
    pub(crate) fn new() -> Self {
        Self {
            ramp_dist: RampFilter::default(),
            ramp_angle: RampFilter::default(),
            pid_dist: PidFilter::default(),
            pid_angle: PidFilter::default(),
        }
    }

    /// Filter using position (current) and target (consign), return filtered `(dist, angle)` speed
    pub(crate) fn filter(&mut self, dist: f32, angle: f32, dist_target: f32, dist_angle: f32) -> (f32, f32) {
        let dist_target = self.ramp_dist.filter(dist_target);
        let angle_target = self.ramp_angle.filter(dist_angle);

        let dist_error = rcs_mm_to_cs_unit(dist_target - dist);
        let angle_error = rcs_rad_to_cs_unit(angle_target - angle);

        let dist_speed = self.pid_dist.filter(dist_error);
        let angle_speed = self.pid_angle.filter(angle_error);
        (
            cs_unit_to_rcs_mm(dist_speed),
            cs_unit_to_rcs_rad(angle_speed),
        )
    }

    /// Reset filters
    pub(crate) fn reset(&mut self) {
        self.pid_dist.reset();
        self.pid_angle.reset();
        self.ramp_dist.reset_finished_to(0.0);
        self.ramp_angle.reset_finished_to(0.0);
    }

    /// Provide access to distance PID configurations
    pub fn pid_dist_conf_mut(&mut self) -> &mut PidConf {
        &mut self.pid_dist.conf
    }

    /// Provide access to angle PID configurations
    pub fn pid_angle_conf_mut(&mut self) -> &mut PidConf {
        &mut self.pid_angle.conf
    }

    /// Set linear ramp configuration
    pub(crate) fn set_dist_ramp_conf(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.ramp_dist.configure(speed, acc, time_step);
    }

    /// Set angle ramp configuration
    pub(crate) fn set_angle_ramp_conf(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.ramp_angle.configure(speed, acc, time_step);
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

