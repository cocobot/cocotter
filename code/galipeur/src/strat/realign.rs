//! Wall-relative position measurement and realignment using the
//! ground lidars.
//!
//! [`measure_wall`] reads both ground lidars on a given robot face,
//! applies linear calibration (wall_dist = scale * raw_d + offset),
//! rejects if the two lidars disagree by more than 10 mm, and returns
//! the robot position coordinate (x or y) for the given wall.

use asserv::holonomic::{RobotSide, TableSide};
use board_sabotter::SabotterBoard;

use crate::sensors::{GroundLidarPose, Sensors};

const ASSERV_Y_AT_UP_WALL: f32 = 2000.0;
const ASSERV_X_AT_LEFT_WALL: f32 = -1500.0;
const ASSERV_X_AT_RIGHT_WALL: f32 = 1500.0;

/// Robot position as measured against a known table wall.
#[derive(Debug, Clone, Copy, Default)]
pub struct WallMeasurement {
    /// Robot X in table frame. `Some` for Left/Right walls.
    pub x: Option<f32>,
    /// Robot Y in table frame. `Some` for Up/Down walls.
    pub y: Option<f32>,
}

/// Which lidar(s) on the module to use (for measure_face_distance / measure_edge).
#[derive(Debug, Clone, Copy)]
pub enum LidarSelect {
    BothWithAngle,
    Both,
    Low,
    High,
}

const MAX_PLAUSIBLE_MM: f32 = 5000.0;
const MAX_RETRIES: usize = 3;
const MAX_LIDAR_DISAGREEMENT_MM: f32 = 10.0;

/// Measure the robot's position by reading the ground lidars on `face`
/// against a known `wall`.
///
/// Uses linear calibration: wall_dist = scale * raw_d + offset.
/// Rejects if the two lidars disagree by more than 10 mm.
/// Blocks until a fresh CAN reading arrives. Retries up to 3 times.
pub fn measure_wall<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    wall: TableSide,
) -> Option<WallMeasurement> {
    for attempt in 0..MAX_RETRIES {
        if let Some(m) = measure_wall_once(sensors, face, wall) {
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

/// Face normal angle in body frame (outward direction).
pub fn face_normal_angle(face: RobotSide) -> f32 {
    match face {
        RobotSide::Back => -core::f32::consts::FRAC_PI_2,         
        RobotSide::Left => core::f32::consts::FRAC_PI_6 * 5.0,   
        RobotSide::Right => core::f32::consts::FRAC_PI_6,         
    }
}

/// Angle tangent à la face en body frame (perpendiculaire à la normale, sens trigo).
pub fn face_tangent_angle(face: RobotSide) -> f32 {
    match face {
        RobotSide::Back => 0.0,                                    // normal -90°, tangent 0°
        RobotSide::Left => core::f32::consts::FRAC_PI_3,          // normal 150°, tangent 60°
        RobotSide::Right => -core::f32::consts::FRAC_PI_3,        // normal 30°, tangent -60°
    }
}

/// Hit point in body frame (no world rotation).
fn hit_body(pose: GroundLidarPose, distance: u16) -> Option<(f32, f32)> {
    if distance == 0 {
        return None;
    }
    let d = distance as f32;
    Some((pose.x + d * pose.theta.cos(), pose.y + d * pose.theta.sin()))
}

/// Measure perpendicular distance from robot center to obstacle along
/// the face normal, in body frame (mm).
///
/// Positive = obstacle is in front of the face (outward direction).
/// Retries up to 3 times if implausible (> 5 m).
pub fn measure_face_distance<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    select: LidarSelect,
) -> Option<f32> {
    let normal = face_normal_angle(face);
    measure_projected(sensors, face, select, normal.cos(), normal.sin(), "measure_face_distance")
}

/// Measure lateral offset along a face relative to a stop/edge.
///
/// Returns the tangential component of the hit point in body frame (mm).
/// Positive = offset in the tangent direction (CCW from face normal).
/// Retries up to 3 times if implausible (> 5 m).
pub fn measure_edge<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    select: LidarSelect,
) -> Option<f32> {
    let tangent = face_tangent_angle(face);
    measure_projected(sensors, face, select, tangent.cos(), tangent.sin(), "measure_edge")
}

/// Project body-frame hit point onto an arbitrary direction, with retries.
fn measure_projected<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    select: LidarSelect,
    cos_dir: f32,
    sin_dir: f32,
    label: &str,
) -> Option<f32> {
    for attempt in 0..MAX_RETRIES {
        if let Some(value) = measure_projected_once(sensors, face, select, cos_dir, sin_dir) {
            if value.abs() < MAX_PLAUSIBLE_MM {
                return Some(value);
            }
            log::warn!("{}: implausible {:.1} mm (attempt {}/{})", label, value, attempt + 1, MAX_RETRIES);
        }
    }
    log::error!("{}: all {} retries failed for {:?}", label, MAX_RETRIES, face);
    None
}

fn measure_projected_once<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    select: LidarSelect,
    cos_dir: f32,
    sin_dir: f32,
) -> Option<f32> {
    let module = sensors.ground_lidar_wait(face)?;
    let poses = sensors.ground_lidar_poses(face)?;

    match select {
        LidarSelect::BothWithAngle | LidarSelect::Both => {
            let (bx0, by0) = hit_body(poses[0], module.distance_0)?;
            let (bx1, by1) = hit_body(poses[1], module.distance_1)?;
            let bx = (bx0 + bx1) / 2.0;
            let by = (by0 + by1) / 2.0;
            Some(bx * cos_dir + by * sin_dir)
        }
        single => {
            let (pose, distance) = match single {
                LidarSelect::Low => (poses[1], module.distance_1),
                LidarSelect::High => (poses[0], module.distance_0),
                _ => unreachable!(),
            };
            let (bx, by) = hit_body(pose, distance)?;
            Some(bx * cos_dir + by * sin_dir)
        }
    }
}

fn measure_wall_once<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    face: RobotSide,
    wall: TableSide,
) -> Option<WallMeasurement> {
    let module = sensors.ground_lidar_wait(face)?;
    let calibs = sensors.ground_lidar_calibs(face)?;

    if module.distance_0 == 0 || module.distance_1 == 0 {
        return None;
    }

    let dist_high = calibs[0].scale * module.distance_0 as f32 + calibs[0].offset;
    let dist_low = calibs[1].scale * module.distance_1 as f32 + calibs[1].offset;

    if (dist_high - dist_low).abs() > MAX_LIDAR_DISAGREEMENT_MM {
        log::warn!(
            "measure_wall: lidars disagree: high={:.1} low={:.1} (diff={:.1})",
            dist_high, dist_low, (dist_high - dist_low).abs()
        );
        return None;
    }

    let dist = (dist_high + dist_low) / 2.0;

    let (x, y) = match wall {
        TableSide::Down => (None, Some(dist)),
        TableSide::Up => (None, Some(ASSERV_Y_AT_UP_WALL - dist)),
        TableSide::Left => (Some(ASSERV_X_AT_LEFT_WALL + dist), None),
        TableSide::Right => (Some(ASSERV_X_AT_RIGHT_WALL - dist), None),
    };
    Some(WallMeasurement { x, y })
}