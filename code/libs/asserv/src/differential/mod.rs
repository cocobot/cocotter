pub mod conf;
mod control_system;
mod motor_filter;

use core::time::Duration;
use crate::maths::{XY, XYA, normalize_radians_pi_pi};
use conf::*;
use control_system::ControlSystem;


#[derive(Default)]
struct AsservInternalConf {
    pub xy_stop_window: f32,
    pub xy_aim_angle_window: f32,
    pub xy_cruise_angle_window: f32,
    pub xy_approach_window: f32,
    pub a_stop_window: f32,
}


/// Trajectory order being processed
enum TrajectoryOrder {
    Stop,
    Xy { xy: XY, aiming: bool },
    Angle(f32),
}


/// Differential asserv processing
///
/// The structure does not communicate directly with hardware.
/// All hardware specific calls are implemented by the provided [AsservHardware].
///
/// Initial configuration can be set using [set_conf()].
/// All configuration items can be updated at runtime, between [update()] calls.
///
/// Accessors are provided to the various underlying objects.
pub struct Asserv<H: AsservHardware> {
    pub cs: ControlSystem<H>,

    // Trajectory parameters
    conf: AsservInternalConf,

    // Trajectory order
    order: TrajectoryOrder,
}

impl<H: AsservHardware> Asserv<H> {
    /// Create an asserv from hardware components
    pub fn new(hardware: H) -> Self {
        let cs = ControlSystem::new(hardware);
        Self {
            cs,
            conf: Default::default(),
            order: TrajectoryOrder::Stop,
        }
    }

    /// Fully configure the asserv and fully reset position()
    pub fn set_conf(&mut self, conf: AsservConf) {
        *self.cs.motor_filter.pid_dist_conf_mut() = conf.pid_dist;
        *self.cs.motor_filter.pid_angle_conf_mut() = conf.pid_angle;
        self.cs.set_encoder_conversion(conf.motors.tick_to_mm, conf.motors.tick_to_rad);

        self.set_a_speed(conf.trajectory.a_speed, conf.trajectory.a_acc, conf.update_period);
        self.set_xy_speed(conf.trajectory.xy_speed, conf.trajectory.xy_acc, conf.update_period);
        self.set_xy_order_windows(conf.trajectory.xy_stop_window, conf.trajectory.xy_aim_angle_window, conf.trajectory.xy_cruise_angle_window, conf.trajectory.xy_approach_window);
        self.set_angle_order_window(conf.trajectory.a_stop_window);

        self.reset_position(XYA::new(0.0, 0.0, 0.0));
    }

    pub fn hardware(&self) -> &H {
        &self.cs.hardware
    }

    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.cs.hardware
    }

    /// Run a single asserv step
    ///
    /// This method must be called periodically.
    pub fn update(&mut self, elapsed: &Duration) {
        if elapsed <= &Duration::ZERO {
            return;  // Should not happen
        }

        self.update_trajectory();
        // Update control system (position, motors)
        self.cs.update(elapsed);
    }


    //
    // State getters (more can be accessed directly from `cs`)
    //

    /// Return true if there is no active order
    pub fn done(&self) -> bool {
        matches!(self.order, TrajectoryOrder::Stop)
    }


    //
    // Movement orders
    //

    /// Go to given linear position, reset angle target
    pub fn goto_xy(&mut self, x: f32, y: f32) {
        let xy = XY::new(x, y);
        self.cs.reset_targets();
        self.order = TrajectoryOrder::Xy{ xy, aiming: true };
    }

    /// Same as [goto_xy()] but position is relative to current one
    pub fn goto_xy_rel(&mut self, dx: f32, dy: f32) {
        let current = self.cs.position();
        self.goto_xy(current.x + dx, current.y + dy);
    }

    /// Go to given angle, reset linear target
    pub fn goto_a(&mut self, a: f32) {
        let current_a = self.cs.position().a;
        let da = normalize_radians_pi_pi(a - current_a);
        self.cs.reset_targets();
        self.order = TrajectoryOrder::Angle(current_a + da);
    }

    /// Same as [goto_a()] but angle is relative to current one
    pub fn goto_a_rel(&mut self, da: f32) {
        self.goto_a(self.cs.position().a + da);
    }


    //
    // Configuration setters
    //

    pub fn set_a_speed(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.cs.set_a_speed(speed, acc, time_step);
    }

    pub fn set_xy_speed(&mut self, speed: f32, acc: f32, time_step: Duration) {
        self.cs.set_xy_speed(speed, acc, time_step);
    }

    pub fn set_xy_order_windows(&mut self, xy_win: f32, aim_da: f32, cruise_da: f32, approach_win: f32) {
        self.conf.xy_stop_window = xy_win;
        self.conf.xy_aim_angle_window = aim_da;
        self.conf.xy_cruise_angle_window = cruise_da;
        self.conf.xy_approach_window = approach_win;
    }

    pub fn set_angle_order_window(&mut self, da: f32) {
        self.conf.a_stop_window = da;
    }

    /// Reset position, target, consigns
    pub fn reset_position(&mut self, xya: XYA) {
        self.cs.motor_filter.reset();
        self.cs.reset_position(xya);
    }


    //
    // Internal methods
    //

    /// Update trajectory management
    fn update_trajectory(&mut self) {
        match &self.order {
            TrajectoryOrder::Stop => {
                // Nothing to do
            }

            TrajectoryOrder::Xy { xy, aiming } => {
                let dxy = xy - &self.cs.position().xy();
                let angle_to_target = dxy.angle();
                let current_a = self.cs.position().a;
                let da = normalize_radians_pi_pi(angle_to_target - current_a );

                if *aiming {
                    if da.abs() <= self.conf.xy_aim_angle_window {
                        // Aiming complete
                        self.cs.reset_targets();
                        self.cs.set_target_dist(self.cs.dist() + dxy.length());
                        self.order = TrajectoryOrder::Xy { xy: *xy, aiming: false };
                    } else {
                        self.cs.set_target_a(current_a + da);
                    }
                } else if in_window_xy(&dxy, self.conf.xy_stop_window) {
                    // Target position reached
                    self.cs.reset_targets();
                    self.order = TrajectoryOrder::Stop;
                } else if da.abs() > self.conf.xy_cruise_angle_window {
                    // Start aiming
                    self.cs.reset_targets();
                    self.cs.set_target_a(current_a + da);
                    self.order = TrajectoryOrder::Xy { xy: *xy, aiming: true };
                } else {
                    // Update targets
                    let len = dxy.length();
                    if len > self.conf.xy_approach_window {
                        self.cs.set_target_a(current_a + da);
                    }
                    self.cs.set_target_dist(self.cs.dist() + len);
                }
            },

            TrajectoryOrder::Angle(a) => {
                // Advance towards target
                self.cs.set_target_a(*a);
                let da = normalize_radians_pi_pi(self.cs.position().a - *a);
                if da.abs() < self.conf.a_stop_window {
                    // Order complete
                    self.order = TrajectoryOrder::Stop;
                }
            },
        }
    }
}


/// Return true if given XY difference is within a tolerance window
fn in_window_xy(dxy: &XY, window: f32) -> bool {
    // Coarse inegality to save computing time
    if dxy.x > window && dxy.y > window {
        false
    } else {
        // Squared inegality, check if robot is in window
        dxy.x * dxy.x + dxy.y * dxy.y < window * window
    }
}
