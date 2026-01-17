use rome::{Message, params};
use super::{Asserv, AsservHardware, RobotSide, TableSide, TrajectoryOrder};
use super::conf::*;
use crate::rome::AsservRome;
use crate::maths::{XY, XYA};


impl From<params::AsservHoloAutosetTableSide> for TableSide {
    fn from(value: params::AsservHoloAutosetTableSide) -> Self {
        match value {
            params::AsservHoloAutosetTableSide::Left => Self::Left,
            params::AsservHoloAutosetTableSide::Right => Self::Right,
            params::AsservHoloAutosetTableSide::Up => Self::Up,
            params::AsservHoloAutosetTableSide::Down => Self::Down,
        }
    }
}

impl From<params::AsservHoloAutosetRobotSide> for RobotSide {
    fn from(value: params::AsservHoloAutosetRobotSide) -> Self {
        match value {
            params::AsservHoloAutosetRobotSide::Left => Self::Left,
            params::AsservHoloAutosetRobotSide::Right => Self::Right,
            params::AsservHoloAutosetRobotSide::Back => Self::Back,
        }
    }
}


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
            Message::AsservGotoXya { x, y, a } => {
                log::info!("ROME: goto_xya is not implemented");
                self.goto_xya(x, y, a);
            }
            // Common messages not (yet) implemented
            Message::AsservActivate(_) => {
                log::error!("ROME: AsservActivate is not implemented");
            }
            // AsservHolo messages
            Message::AsservHoloAutoset { robot_side, table_side, target_x, target_y } => {
                log::info!("ROME: autoset");
                self.autoset(robot_side.into(), table_side.into(), XY::new(target_x, target_y));
            }
            Message::AsservHoloSetPidConf { pid, gain_p, gain_i, gain_d, max_in, max_i, max_out } => {
                log::info!("ROME: set PID conf ({pid:?})");
                let conf = PidConf { gain_p, gain_i, gain_d, max_in, max_i, max_out, out_shift: 0 };
                match pid {
                    params::AsservHoloSetPidConfPid::X => self.set_x_pid_conf(conf),
                    params::AsservHoloSetPidConfPid::Y => self.set_y_pid_conf(conf),
                    params::AsservHoloSetPidConfPid::A => self.set_a_pid_conf(conf),
                }
            }
            Message::AsservHoloSetTrajectoryConf {
                a_speed, a_acc, xy_cruise_speed, xy_cruise_acc, xy_steering_speed, xy_steering_acc,
                xy_stop_speed, xy_stop_acc, xy_steering_window, xy_stop_window, a_stop_window,
                autoset_speed, autoset_wait, autoset_duration,
            } => {
                log::info!("ROME: set trajectory conf");
                let conf = TrajectoryConf {
                    a_speed, a_acc, xy_cruise_speed, xy_cruise_acc, xy_steering_speed, xy_steering_acc,
                    xy_stop_speed, xy_stop_acc, xy_steering_window, xy_stop_window, a_stop_window,
                    autoset_speed, autoset_wait, autoset_duration,
                };
                self.set_trajectory_conf(conf);
            }
            Message::AsservHoloSetMotorsConf { velocities_to_consigns, encoders_to_position } => {
                log::info!("ROME: set motors conf");
                let conf = MotorsConf { velocities_to_consigns, encoders_to_position };
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
            idle: self.done_xy() && self.done_a(),
        }
    }

    fn asserv_tm_velocity(&self) -> Option<Message> {
        None
    }
}

/// Support for common holonomic asserv ROME messages
pub trait AsservHoloRome {
    /// Create an `AsservHoloTmStatus` message from current asserv state
    fn asserv_holo_tm_status(&self) -> Message;
    /// Create an `AsservHoloTmPath` message, return `None` if no path is active
    fn asserv_holo_tm_path(&self) -> Option<Message>;
}

impl<H: AsservHardware> AsservHoloRome for Asserv<H> {
    fn asserv_holo_tm_status(&self) -> Message {
        let status = match self.order {
            TrajectoryOrder::Idle => params::AsservHoloTmStatusStatus::Idle,
            TrajectoryOrder::Path(_) => params::AsservHoloTmStatusStatus::Path,
            TrajectoryOrder::Autoset(_) => params::AsservHoloTmStatusStatus::Autoset,
        };
        Message::AsservHoloTmStatus {
            status,
            carrot_x: self.carrot.x,
            carrot_y: self.carrot.y,
            carrot_a: self.carrot_a,
        }
    }

    fn asserv_holo_tm_path(&self) -> Option<Message> {
        if let TrajectoryOrder::Path(path_data) = &self.order {
            let path_data = path_data.borrow();
            Some(Message::AsservHoloTmPath {
                carrot_speed: path_data.carrot_speed,
                path_index: path_data.index,
                path_size: path_data.size,
            })
        } else {
            None
        }
    }
}

