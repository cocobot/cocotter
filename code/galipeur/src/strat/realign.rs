//! Position realignment using the ground lidars.
//!
//! Given a robot face that the strat has *physically* placed roughly
//! parallel to a known table wall, sample the ground lidars on that
//! face, fit the wall plane, and reset the asserv's software pose so
//! it accurately reflects the chassis. The component perpendicular to
//! the wall (`x` for Left/Right walls, `y` for Up/Down walls) and the
//! heading are corrected; the component parallel to the wall is left
//! untouched (the lidars can't resolve it).
//!

use asserv::holonomic::{RobotSide, TableSide};
use board_common::eurobot::TABLE_SIZE;
use board_sabotter::SabotterBoard;

use crate::sensors::Sensors;
use crate::strat::errors::StrategyError;
use crate::strat::utils::{arfast, AsservHelper};

/// Asserv-frame coordinates of the four table edges. In the asserv
/// frame: `+X` points from the Down wall toward the Up wall,
/// `+Y` points from the Right wall toward the Left wall.
const ASSERV_X_AT_UP_WALL: f32 = TABLE_SIZE.y;
const ASSERV_X_AT_DOWN_WALL: f32 = 0.0;
const ASSERV_Y_AT_LEFT_WALL: f32 = TABLE_SIZE.x / 2.0;
const ASSERV_Y_AT_RIGHT_WALL: f32 = -TABLE_SIZE.x / 2.0;

fn wrap_pi(x: f32) -> f32 {
    ((x + core::f32::consts::PI).rem_euclid(core::f32::consts::TAU))
        - core::f32::consts::PI
}

/// Realign the asserv pose using the ground lidars on `face`,
/// against `wall`.
///
/// Pre-condition: the chassis is placed such that `face` is roughly
/// parallel to `wall` (within the lidars' fit window — a few degrees
/// of mis-orientation is fine, but the face should physically be the
/// one closest to the wall).
///
/// Post-condition: the asserv reports a pose whose component normal
/// to the wall and whose heading match the lidar measurement.
///
/// # Geometry
///
/// At alignment the asserv heading is `arfast(face, wall)` (the
/// project's source of truth for "face F is parallel to wall W").
/// `get_plane_offset` reports the residual angle, so:
/// ```text
///     θ_a_new = arfast(face, wall) - reported
/// ```
///
/// The reported `distance` is the perpendicular body-to-wall distance
/// in mm. The wall-normal coordinate of the body is then the wall's
/// known coordinate ± `distance`, sign chosen so that the body sits
/// inside the playing area.
#[allow(unused)]
pub fn realign<B: SabotterBoard + 'static>(
    asserv: &AsservHelper<B>,
    sensors: &Sensors<B>,
    face: RobotSide,
    wall: TableSide,
) -> Result<(), StrategyError> {
    let po = sensors
        .get_plane_offset(face)
        .ok_or(StrategyError::SensorUnavailable)?;
    let current = asserv.position();

    let new_theta = wrap_pi(arfast(face, wall) - po.angle);

    let (new_x, new_y) = match wall {
        TableSide::Up => (ASSERV_X_AT_UP_WALL - po.distance, current.y),
        TableSide::Down => (ASSERV_X_AT_DOWN_WALL + po.distance, current.y),
        TableSide::Left => (current.x, ASSERV_Y_AT_LEFT_WALL - po.distance),
        TableSide::Right => (current.x, ASSERV_Y_AT_RIGHT_WALL + po.distance),
    };

    log::info!(
        "[realign] {face:?}/{wall:?}: angle={:.3} dist={:.1} \
         pose ({:.1}, {:.1}, {:.3}) → ({:.1}, {:.1}, {:.3})",
        po.angle, po.distance,
        current.x, current.y, current.a,
        new_x, new_y, new_theta,
    );
    asserv.reset_position(new_x, new_y, new_theta);
    Ok(())
}
