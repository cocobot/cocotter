//! Wall-relative position measurement and realignment using the
//! ground lidars.
//!
//! [`measure_wall`] reads the ground lidars on a given robot face,
//! computes the robot's position relative to a known table wall, and
//! returns it as `(x, y, a)` — each component is `Some` only when the
//! geometry allows it:
//!
//! | wall        | x      | y      | a (heading)           |
//! |-------------|--------|--------|-----------------------|
//! | Left/Right  | Some   | None   | Some (if `BothWithAngle`) |
//! | Up/Down     | None   | Some   | Some (if `BothWithAngle`) |
//!
//! With `Both`, `Low`, or `High` the heading is not recovered —
//! the current asserv heading is used to project the distance
//! reading. `Both` still uses the plane fit for better distance
//! accuracy.

use asserv::holonomic::{RobotSide, TableSide};
use asserv::maths::XYA;
use board_sabotter::SabotterBoard;

use crate::sensors::{GroundLidarPose, Sensors};
use crate::strat::utils::arfast;

//TODO Damien: This should be moved in a shared crate between pami and galipeur
const ASSERV_Y_AT_UP_WALL: f32 = 2000.0;
const ASSERV_Y_AT_DOWN_WALL: f32 = 0.0;
const ASSERV_X_AT_LEFT_WALL: f32 = -1500.0;
const ASSERV_X_AT_RIGHT_WALL: f32 = 1500.0;

fn wrap_pi(x: f32) -> f32 {
    ((x + core::f32::consts::PI).rem_euclid(core::f32::consts::TAU))
        - core::f32::consts::PI
}

/// Which lidar(s) on the module to use.
#[derive(Debug, Clone, Copy)]
pub enum LidarSelect {
    /// Both lidars → heading from plane fit, position from averaged hit points.
    BothWithAngle,
    /// Both lidars → averaged hit points, no heading recovery.
    Both,
    /// Only the lower-mounted lidar (module lane 0).
    Low,
    /// Only the higher-mounted lidar (module lane 1).
    High,
}

/// Robot position as measured against a known table wall.
#[derive(Debug, Clone, Copy, Default)]
pub struct WallMeasurement {
    /// Robot X in table frame. `Some` for Left/Right walls.
    pub x: Option<f32>,
    /// Robot Y in table frame. `Some` for Up/Down walls.
    pub y: Option<f32>,
    /// Robot heading in table frame. `Some` only with `LidarSelect::BothWithAngle`.
    pub a: Option<f32>,
}

/// Compute world-frame hit point offset from robot center.
fn hit_to_world(pose: GroundLidarPose, distance: u16, heading: f32) -> Option<(f32, f32)> {
    if distance == 0 {
        return None;
    }
    let d = distance as f32;
    let px = pose.x + d * pose.theta.cos();
    let py = pose.y + d * pose.theta.sin();
    let c: f32 = heading.cos();
    let s = heading.sin();
    Some((c * px - s * py, s * px + c * py))
}

/// Convert a world-frame hit offset to robot coordinate given a known wall.
fn wall_coordinate(wall: TableSide, hit_wx: f32, hit_wy: f32) -> (Option<f32>, Option<f32>) {
    match wall {
        TableSide::Up => (None, Some(ASSERV_Y_AT_UP_WALL - hit_wy)),
        TableSide::Down => (None, Some(ASSERV_Y_AT_DOWN_WALL - hit_wy)),
        TableSide::Left => (Some(ASSERV_X_AT_LEFT_WALL - hit_wx), None),
        TableSide::Right => (Some(ASSERV_X_AT_RIGHT_WALL - hit_wx), None),
    }
}

const MAX_PLAUSIBLE_MM: f32 = 5000.0;
const MAX_RETRIES: usize = 3;

/// Measure the robot's position by reading the ground lidars on `face`
/// against a known `wall`.
///
/// Blocks until a fresh CAN reading arrives. Returns `None` if the
/// lidar reports zero distance or the config isn't set. Retries up to
/// 3 times if the result exceeds 5 m.
pub fn measure_wall<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    wall: TableSide,
    select: LidarSelect,
    current_pos: XYA,
) -> Option<WallMeasurement> {
    for attempt in 0..MAX_RETRIES {
        if let Some(m) = measure_wall_once(sensors, face, wall, select, current_pos) {
            let plausible = m.x.map_or(true, |v| v.abs() < MAX_PLAUSIBLE_MM)
                && m.y.map_or(true, |v| v.abs() < MAX_PLAUSIBLE_MM);
            
            if plausible {
                return Some(m);
            }
            log::warn!("measure_wall: implausible result (attempt {}/{}), retrying", attempt + 1, MAX_RETRIES);
        }
    }
    log::error!("measure_wall: all {} retries failed for {:?}/{:?}", MAX_RETRIES, face, wall);
    None
}

fn measure_wall_once<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    wall: TableSide,
    select: LidarSelect,
    current_pos: XYA,
) -> Option<WallMeasurement> {
    let module = sensors.ground_lidar_wait(face)?;
    let poses = sensors.ground_lidar_poses(face)?;

    match select {
        LidarSelect::BothWithAngle | LidarSelect::Both => {
            if module.distance_0 == 0 || module.distance_1 == 0 {
                return None;
            }

            // Heading: from plane fit (BothWithAngle) or from asserv (Both).
            let heading = if matches!(select, LidarSelect::BothWithAngle) {
                let d0 = module.distance_0 as f32;
                let d1 = module.distance_1 as f32;
                let p0x = poses[0].x + d0 * poses[0].theta.cos();
                let p0y = poses[0].y + d0 * poses[0].theta.sin();
                let p1x = poses[1].x + d1 * poses[1].theta.cos();
                let p1y = poses[1].y + d1 * poses[1].theta.sin();
                let dx = p1x - p0x;
                let dy = p1y - p0y;
                let len = (dx * dx + dy * dy).sqrt();
                if len < 1e-6 {
                    return None;
                }
                let mut nx = -dy / len;
                let mut ny = dx / len;
                if nx * p0x + ny * p0y < 0.0 {
                    nx = -nx;
                    ny = -ny;
                }
                let plane_angle = ny.atan2(nx);
                wrap_pi(arfast(face, wall) - plane_angle)
            } else {
                current_pos.a
            };

            // Position: average of both hit points rotated to world frame.
            let (wx0, wy0) = hit_to_world(poses[0], module.distance_0, heading)?;
            let (wx1, wy1) = hit_to_world(poses[1], module.distance_1, heading)?;
            let (x, y) = wall_coordinate(wall, (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0);
            let a = if matches!(select, LidarSelect::BothWithAngle) {
                Some(heading)
            } else {
                None
            };
            Some(WallMeasurement { x, y, a })
        }
        single => {
            let (pose, distance) = match single {
                LidarSelect::Low => (poses[0], module.distance_0),
                LidarSelect::High => (poses[1], module.distance_1),
                LidarSelect::Both | LidarSelect::BothWithAngle => unreachable!(),
            };
            let (wx, wy) = hit_to_world(pose, distance, current_pos.a)?;
            let (x, y) = wall_coordinate(wall, wx, wy);
            Some(WallMeasurement { x, y, a: None })
        }
    }
}