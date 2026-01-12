use crate::conf::*;
use crate::control_system::ControlSystem;
use crate::maths::{XY, XYA, normalize_radians_pi_pi};


// This struct is very similar to `conf::TrajectoryConf`.
// Field names are different (mostly historical).
// And angular speed/acc are not stored in (they are set on quadramp).
#[derive(Default)]
struct AsservInternalConf {
    pub cruise_speed: f32,
    pub cruise_acc: f32,
    pub steering_speed: f32,
    pub steering_acc: f32,
    pub stop_speed: f32,
    pub stop_acc: f32,
    pub xy_steering_window: f32,
    pub xy_stop_window: f32,
    pub a_stop_window: f32,
    pub autoset_speed: f32,
    pub autoset_wait: u8,
    pub autoset_duration: u8,
}

/// Maximum number of points for a trajectory path
pub const TRAJECTORY_MAX_POINTS: u8 = 15;


/// Game table side
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TableSide {
    Left,
    Right,
    Up,
    Down,
}


/// Robot side, relative to the Y axis
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RobotSide {
    /// Left side (X < 0, along Y axis)
    Left,
    /// Right side (X > 0, along Y axis)
    Right,
    /// Back side (Y < 0, along X axis)
    Back,
}

/// Constant values for autoset configurations, one per [RobotSide] value
struct AutosetConfig {
    /// Base angle of the robot side, in robot's basis
    base_a: f32,
    /// Angle for an autoset on table's left side
    left_a: f32,
    /// Angle for an autoset on table's right side
    right_a: f32,
    /// Angle for an autoset on table's top side
    up_a: f32,
    /// Angle for an autoset on table's bottom side
    down_a: f32,
}

/// Trajectory order being processed
enum TrajectoryOrder {
    Stop,
    Path(std::cell::RefCell<PathData>),
    Autoset(std::cell::RefCell<AutosetData>),
}

/// Information needed for path movement
struct PathData {
    points: [XY; TRAJECTORY_MAX_POINTS as usize],
    size: u8,
    index: u8,
    carrot_speed: f32,
}

impl PathData {
    fn next_point(&self) -> &XY {
        &self.points[self.index as usize]
    }

    fn is_last_point(&self) -> bool {
        self.index + 1 >= self.size
    }
}


/// Information needed for autoset movement
struct AutosetData {
    table_side: TableSide,
    robot_side: RobotSide,
    target: XY,  // Only one coordinate will be used
    state: AutosetState,
}

impl AutosetData {
    const fn config(&self) -> &'static AutosetConfig {
        match self.robot_side {
            RobotSide::Back => &AutosetConfig {
                base_a: -core::f32::consts::FRAC_PI_2,
                left_a: -core::f32::consts::FRAC_PI_2,
                right_a: core::f32::consts::FRAC_PI_2,
                up_a: core::f32::consts::PI,
                down_a: 0.0,
            },
            RobotSide::Right => &AutosetConfig {
                base_a: core::f32::consts::FRAC_PI_6,
                left_a: 5. * core::f32::consts::FRAC_PI_6,
                right_a: -core::f32::consts::FRAC_PI_6,
                up_a: core::f32::consts::PI/3.0,
                down_a: -2. * core::f32::consts::PI/3.0,
            },
            RobotSide::Left => &AutosetConfig {
                base_a: 5. * core::f32::consts::FRAC_PI_6,
                left_a: core::f32::consts::FRAC_PI_6,
                right_a: -5. * core::f32::consts::FRAC_PI_6,
                up_a: -2. * core::f32::consts::FRAC_PI_6,
                down_a: 4. * core::f32::consts::FRAC_PI_6,
            },
        }
    }

    /// Known angle at the end of autoset move
    fn target_a(&self) -> f32 {
        let config = self.config();
        match self.table_side {
            TableSide::Left => config.left_a,
            TableSide::Right => config.right_a,
            TableSide::Up => config.up_a,
            TableSide::Down => config.down_a,
        }
    }

    /// Autoset full target position
    fn target(&self) -> XYA {
        self.target.with_a(self.target_a())
    }

    /// Unit vector of autoset direction
    fn direction(&self) -> XY {
        let a = self.config().base_a;
        XY::new(a.cos(), a.sin())
    }
}

#[derive(Clone, Copy)]
enum AutosetState {
    Heading,
    HeadingWait(u8),
    Move(u8),
    MoveWait(u8),
}

/// Information needed for synced angle movement
struct SyncedAngleData {
    total_distance: f32,
    total_angle: f32,  // normalized
    origin_xy: XY,
    origin_a: f32,
}

impl SyncedAngleData {
    fn new(origin: &XYA, destination: &XYA) -> Self {
        let origin_xy = origin.xy();
        Self {
            total_distance: (destination.xy() - origin_xy).length(),
            total_angle: normalize_radians_pi_pi(destination.a - origin.a),
            origin_xy,
            origin_a: origin.a,
        }
    }

    fn synced_angle(&self, current_xy: &XY) -> f32 {
        let d_current = (current_xy - &self.origin_xy).length();
        let progress = d_current / self.total_distance;
        self.origin_a + progress * self.total_angle
    }
}


/// Robot asserv processing
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

    // Trajectory order (note: angle is handled separately)
    order: TrajectoryOrder,
    carrot: XY,
    carrot_a: f32,

    // Set for synced angle movement
    synced_angle: Option<SyncedAngleData>,
}

impl<H: AsservHardware> Asserv<H> {
    /// Create an asserv from hardware components
    pub fn new(hardware: H) -> Self {
        let cs = ControlSystem::new(hardware);
        Self {
            cs,
            conf: Default::default(),
            order: TrajectoryOrder::Stop,
            carrot: Default::default(),
            carrot_a: 0.0,
            synced_angle: None,
        }
    }

    /// Fully configure the asserv and fully reset position()
    pub fn set_conf(&mut self, conf: AsservConf) {
        {
            let pid_confs = self.cs.motor_filter.pid_confs_mut();
            *pid_confs.x = conf.pid_x;
            *pid_confs.y = conf.pid_y;
            *pid_confs.a = conf.pid_a;
        }

        self.set_a_speed(conf.trajectory.a_speed, conf.trajectory.a_acc);
        self.set_xy_cruise_speed(conf.trajectory.xy_cruise_speed, conf.trajectory.xy_cruise_acc);
        self.set_xy_steering_speed(conf.trajectory.xy_steering_speed, conf.trajectory.xy_steering_acc);
        self.set_xy_stop_speed(conf.trajectory.xy_stop_speed, conf.trajectory.xy_stop_acc);
        self.set_steering_window(conf.trajectory.xy_steering_window);
        self.set_stop_windows(conf.trajectory.xy_stop_window, conf.trajectory.a_stop_window);
        self.set_autoset_speed(conf.trajectory.autoset_speed);
        self.set_autoset_delays(conf.trajectory.autoset_wait, conf.trajectory.autoset_duration);

        self.cs.set_motors_matrix(conf.motors.matrix);
        self.cs.set_motors_inv_matrix(conf.motors.inv_matrix);

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
    pub fn update(&mut self) {
        self.update_synced_angle();
        self.update_trajectory();
        // Update control system (position, motors)
        self.cs.update();
    }


    //
    // State getters (more can be accessed directly from `cs`)
    //

    /// Return true if target linear position is reached
    pub fn done_xy(&self) -> bool {
        matches!(self.order, TrajectoryOrder::Stop)
    }

    /// Return true if target angular position is reached
    pub fn done_a(&self) -> bool {
        let da = (self.carrot_a - self.cs.position().a).abs();
        da < self.conf.a_stop_window
    }

    /// Return true if an autoset is in progress
    pub fn autoset_in_progress(&self) -> bool {
        matches!(self.order, TrajectoryOrder::Autoset(_))
    }


    //
    // Movement orders
    //

    /// Start an autoset procedure
    pub fn autoset(&mut self, robot_side: RobotSide, table_side: TableSide, target: XY) {
        // Set carrot position to current position
        self.set_carrot_xy_consign(self.cs.position().xy());

        let autoset_data = AutosetData {
            table_side,
            robot_side,
            target,
            state: AutosetState::Heading,
        };
        self.goto_a(autoset_data.target_a());
        self.order = TrajectoryOrder::Autoset(autoset_data.into());
    }

    /// Load and run a trajectory path
    pub fn run_path(&mut self, path: &[XY]) {
        if path.is_empty() {
            // Empty path: stop to current position
            self.set_carrot_xy_consign(self.cs.position().xy());
            self.order = TrajectoryOrder::Stop;
        } else {
            // Truncate path len if needed
            let n = (TRAJECTORY_MAX_POINTS as usize).min(path.len());
            let mut path_data = PathData {
                points: Default::default(),
                size: n as u8,
                index: 0,
                carrot_speed: 0.0,
            };
            path_data.points[0..n].copy_from_slice(path);
            self.set_carrot_xy_consign(*path_data.next_point());
            self.order = TrajectoryOrder::Path(path_data.into());
        }

    }

    /// Go to given angle, don't change linear target
    pub fn goto_a(&mut self, a: f32) {
        let robot_a = self.cs.position().a;
        // Compute distance between consign and position modulo 2pi
        let da = normalize_radians_pi_pi(a - robot_a);
        // Update consign
        self.carrot_a = robot_a + da;
        self.cs.set_target_a(self.carrot_a);
    }

    /// Same as [goto_a()] but angle is relative to current one
    pub fn goto_a_rel(&mut self, da: f32) {
        self.goto_a(self.cs.position().a + da);
    }

    /// Go to given linear position, don't change angular target
    pub fn goto_xy(&mut self, x: f32, y: f32) {
        // Create and run a one point path
        self.run_path(&[XY::new(x, y)]);
    }

    /// Same as [goto_xy()] but position is relative to current one
    pub fn goto_xy_rel(&mut self, dx: f32, dy: f32) {
        let current = self.cs.position();
        self.goto_xy(current.x + dx, current.y + dy);
    }

    /// Got to given linear position and angle
    pub fn goto_xya(&mut self, x: f32, y: f32, a: f32) {
        self.goto_xy(x, y);
        self.goto_a(a);
    }

    /// Go to given position and angle, synchronize angle with movement
    ///
    /// Target angle will be reached at the end of linear movement.
    pub fn goto_xy_synced(&mut self, x: f32, y: f32, a: f32) {
        //XXX Note: This does not give a "goto_xy()" order
        self.synced_angle = Some(SyncedAngleData::new(self.cs.position(), &XYA::new(x, y, a)));
    }


    //
    // Configuration setters
    //

    pub fn set_a_speed(&mut self, speed: f32, acc: f32) {
        self.cs.set_a_speed(speed, acc);
    }

    pub fn set_xy_cruise_speed(&mut self, speed: f32, acc: f32) {
        self.conf.cruise_speed = speed;
        self.conf.cruise_acc = acc;
    }

    pub fn set_xy_steering_speed(&mut self, speed: f32, acc: f32) {
        self.conf.steering_speed = speed;
        self.conf.steering_acc = acc;
    }

    pub fn set_xy_stop_speed(&mut self, speed: f32, acc: f32) {
        self.conf.stop_speed = speed;
        self.conf.stop_acc = acc;
    }

    pub fn set_steering_window(&mut self, xywin: f32) {
        self.conf.xy_steering_window = xywin;
    }

    pub fn set_stop_windows(&mut self, xywin: f32, awin: f32) {
        self.conf.xy_stop_window = xywin;
        self.conf.a_stop_window = awin;
    }

    pub fn set_autoset_speed(&mut self, speed: f32) {
        self.conf.autoset_speed = speed;
    }

    pub fn set_autoset_delays(&mut self, wait: u8, duration: u8) {
        self.conf.autoset_wait = wait;
        self.conf.autoset_duration = duration;
    }

    /// Reset position, reset carrot to current position, reset motor consigns
    pub fn reset_position(&mut self, xya: XYA) {
        self.cs.motor_filter.reset();
        self.cs.reset_position(xya);
        self.carrot = self.cs.position().xy();
        self.carrot_a = self.cs.position().a;
    }


    //
    // Internal methods
    //

    /// Apply synced angle constraint
    fn update_synced_angle(&mut self) {
        if let Some(synced_angle) = &self.synced_angle {
            let angle = synced_angle.synced_angle(&self.cs.position().xy());
            self.goto_a(angle);
            if self.done_xy() && self.done_a() {
                self.synced_angle = None;
            }
        }
    }

    /// Update trajectory management
    fn update_trajectory(&mut self) {
        match &self.order {
            TrajectoryOrder::Stop => {
                // Nothing to do
            }

            TrajectoryOrder::Path(path_data) => {
                let is_last_point: bool = path_data.borrow().is_last_point();
                let in_window_xy: bool = {
                    let window = if is_last_point {
                        self.conf.xy_steering_window
                    } else {
                        self.conf.xy_stop_window
                    };
                    self.in_window_xy(path_data.borrow().next_point(), window)
                };

                // Is robot in position?
                if in_window_xy {
                    if is_last_point {
                        // Last point reached: full stop
                        // Set carrot to last position
                        let next_point = *path_data.borrow().next_point();
                        self.set_carrot_xy_consign(next_point);
                        self.order = TrajectoryOrder::Stop;
                        return;
                    }
                    // Switch to next point
                    path_data.borrow_mut().index += 1;
                }

                let point = *path_data.borrow().next_point();
                let mut carrot_speed = path_data.borrow().carrot_speed;

                let (max_speed, max_acc) = match path_data.borrow().is_last_point() {
                    false => (self.conf.steering_speed, self.conf.steering_acc),
                    true => (self.conf.stop_speed, self.conf.stop_acc),
                };

                // Compute squared distance between carrot and target
                let error = point - self.carrot;
                let sq_error_length = error.x * error.x + error.y * error.y;

                // Compute distance at which constant deceleration will bring robot to desired speed
                //   dec_distance = 1/2 (speed_1 + speed_0) × (speed_1 - speed_0) / acc
                //                = average_speed × deceleration_duration
                let dec_distance = 0.5 * (carrot_speed + max_speed) * (carrot_speed - max_speed) / max_acc;

                if sq_error_length < dec_distance * dec_distance {
                    // Deceleration phase
                    carrot_speed = (carrot_speed - max_acc).max(max_speed);
                } else if carrot_speed < self.conf.cruise_speed {
                    // Acceleration phase
                    carrot_speed = (carrot_speed + self.conf.cruise_acc).min(self.conf.cruise_speed);
                } else {
                    // Stable phase: nothing to do
                }

                // Update carrot position
                if sq_error_length < carrot_speed * carrot_speed {
                    self.carrot = point;
                } else {
                    self.carrot += carrot_speed * error.unit();
                }

                // Update carrot speed and consign
                path_data.borrow_mut().carrot_speed = carrot_speed;
                self.set_carrot_xy_consign(self.carrot);
            }

            TrajectoryOrder::Autoset(autoset_data) => {
                let autoset_state = autoset_data.borrow().state;
                match autoset_state {
                    AutosetState::Heading => {
                        // Wait for robot heading OK
                        if self.done_a() {
                            autoset_data.borrow_mut().state = AutosetState::HeadingWait(0);
                        }
                    }
                    AutosetState::HeadingWait(mut count) => {
                        count += 1;
                        let mut autoset_data = autoset_data.borrow_mut();
                        autoset_data.state = if count < self.conf.autoset_wait {
                            AutosetState::HeadingWait(count)
                        } else {
                            self.cs.disable_motor_control();
                            // Set the final target position
                            let position = self.cs.position();
                            match autoset_data.table_side {
                                TableSide::Left | TableSide::Right => {
                                    autoset_data.target.x = position.x;
                                }
                                TableSide::Up | TableSide::Down => {
                                    autoset_data.target.y = position.y;
                                }
                            }
                            AutosetState::Move(0)
                        };
                    }
                    AutosetState::Move(mut count) => {
                        count += 1;
                        let mut autoset_data = autoset_data.borrow_mut();
                        autoset_data.state = if count < self.conf.autoset_duration {
                            let progress = count as f32 / self.conf.autoset_duration as f32;
                            //TODO Why 2.0 factor?!
                            let d = autoset_data.direction() * self.conf.autoset_speed * 1_f32.min(2. * progress);
                            self.cs.set_motors_from_velocities(d.x, d.y, 0.0);
                            AutosetState::Move(count)
                        } else {
                            self.cs.stop_motors();
                            AutosetState::MoveWait(0)
                        };
                    }
                    AutosetState::MoveWait(mut count) => {
                        count += 1;
                        if count < self.conf.autoset_wait {
                            autoset_data.borrow_mut().state = AutosetState::MoveWait(count);
                        } else {
                            // Autoset done
                            let target = autoset_data.borrow().target();
                            self.reset_position(target);
                            self.cs.enable_motor_control();
                            self.order = TrajectoryOrder::Stop;
                        }
                    }
                }
            }
        }
    }

    /// Send carrot position consig to systems
    fn set_carrot_xy_consign(&mut self, xy: XY) {
        self.cs.set_target_xy(xy.x, xy.y);
    }

    /// Return true if robot is close enough to a target point
    fn in_window_xy(&self, target: &XY, window: f32) -> bool {
        let dr = target - &self.cs.position().xy();
        // Coarse inegality to save computing time
        if dr.x > window && dr.y > window {
            false
        } else {
            // Squared inegality, check if robot is in window
            dr.x*dr.x + dr.y*dr.y < window*window
        }
    }
}

