use rome::{Message, params};
use super::{Asserv, AsservHardware, TrajectoryOrder};
use super::conf::*;
use crate::rome::AsservRome;
use crate::maths::XYA;


impl<H: AsservHardware> AsservRome for Asserv<H> {
    fn on_rome_message(&mut self, message: &Message) -> bool {
        match *message {
            // Common messages
            Message::AsservSetPosition { x, y, a } => {
                log::info!("ROME: reset_position: {x},{y},{a}");
                self.reset_position(XYA::new(x, y, a));
            }
            Message::AsservGotoXy { x, y } => {
                log::info!("ROME: goto_xy: {x},{y}");
                self.goto_xy(x, y);
            }
            Message::AsservGotoXyRel { dx, dy } => {
                log::info!("ROME: goto_xyrel: {dx},{dy}");
                self.goto_xy_rel(dx, dy);
            }
            Message::AsservGotoA { a } => {
                log::info!("ROME: goto_a: {a}");
                self.goto_a(a);
            }
            Message::AsservGotoARel { da } => {
                log::info!("ROME: goto_a_rel: {da}");
                self.goto_a_rel(da);
            }
            // Common messages not (yet) implemented
            Message::AsservGotoXya { .. } => {
                log::error!("ROME: AsservGotoXya is not implemented");
            }
            Message::AsservActivate(_) => {
                log::error!("ROME: AsservActivate is not implemented");
            }
            // AsservDiff messages
            Message::AsservDiffSetPidConf { pid, gain_p, gain_i, gain_d, max_in, max_i, max_out } => {
                log::info!("ROME: set PID conf ({pid:?})");
                let conf = PidConf { gain_p, gain_i, gain_d, max_in, max_i, max_out, out_shift: 0 };
                match pid {
                    params::AsservDiffSetPidConfPid::Dist => self.set_dist_pid_conf(conf),
                    params::AsservDiffSetPidConfPid::Angle => self.set_angle_pid_conf(conf),
                }
            }
            Message::AsservDiffSetTrajectoryConf {
                a_speed, a_acc, xy_speed, xy_acc, xy_stop_window, xy_aim_angle_window, xy_cruise_angle_window,
                xy_approach_window, a_stop_window, xy_idle_speed, a_idle_speed,
            } => {
                log::info!("ROME: set trajectory conf");
                let conf = TrajectoryConf {
                    a_speed, a_acc, xy_speed, xy_acc, xy_stop_window, xy_aim_angle_window, xy_cruise_angle_window,
                    xy_approach_window, a_stop_window, xy_idle_speed, a_idle_speed,
                };
                self.set_trajectory_conf(conf);
            }
            Message::AsservDiffSetMotorsConf { tick_to_mm, tick_to_rad } => {
                log::info!("ROME: set motors conf");
                let conf = MotorsConf { tick_to_mm, tick_to_rad };
                self.set_motors_conf(conf);
            }
            // Non-asserv messages, not handled
            _ => {
                return false;
            }
        }
        true
    }

    fn asserv_tm_status(&self) -> Message {
        let position = self.cs.position();
        Message::AsservTmStatus {
            x: position.x,
            y: position.y,
            a: position.a,
            idle: self.idle(),
        }
    }

    fn asserv_tm_velocity(&self) -> Option<Message> {
        None
    }
}

/// Support for common differential asserv ROME messages
pub trait AsservDiffRome {
    /// Create an `AsservDiffTmStatus` message from current asserv state
    fn asserv_diff_tm_status(&self) -> Message;
}

impl<H: AsservHardware> AsservDiffRome for Asserv<H> {
    fn asserv_diff_tm_status(&self) -> Message {
        let status = match self.order {
            TrajectoryOrder::Idle => params::AsservDiffTmStatusStatus::Idle,
            TrajectoryOrder::Stop => params::AsservDiffTmStatusStatus::Stop,
            TrajectoryOrder::Xy { .. } => params::AsservDiffTmStatusStatus::Xy,
            TrajectoryOrder::Angle(_) => params::AsservDiffTmStatusStatus::Angle,
        };
        let (vdist, va) = self.cs.speeds();
        Message::AsservDiffTmStatus {
            status,
            dist: self.cs.dist(),
            vdist,
            va,
        }
    }
}

