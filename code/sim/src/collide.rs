//! 2D swept-shape collision between the robot (union of polygons +
//! cylinders) and static AABB obstacles, backed by parry2d.
//!
//! Rotation is left un-blocked (we only sweep translational motion).
//! The robot shape is rebuilt from its `CollisionPrimitive` list each
//! time a fresh `RobotShape` is requested; the per-tick check then just
//! runs `cast_shapes` against each obstacle.

use parry2d::math::{Pose2, Vec2};
use parry2d::query::{cast_shapes, ShapeCastOptions};
use parry2d::shape::{Ball, Compound, ConvexPolygon, Cuboid, SharedShape};

use crate::config::{CollisionPrimitive, Obstacle};

/// Pre-built, cloneable `parry2d` shape describing the robot's collider.
#[derive(Clone)]
pub struct RobotShape {
    inner: SharedShape,
}

impl RobotShape {
    /// Build a compound shape from a list of `CollisionPrimitive`s in the
    /// robot body frame. Degenerate polygons (fewer than 3 points or
    /// non-convex / colinear) are skipped with a warning.
    pub fn from_primitives(primitives: &[CollisionPrimitive]) -> Option<Self> {
        let mut parts: Vec<(Pose2, SharedShape)> = Vec::new();
        for p in primitives {
            match p {
                CollisionPrimitive::Polygon { points_mm, .. } => {
                    let pts: Vec<Vec2> =
                        points_mm.iter().map(|xy| Vec2::new(xy[0], xy[1])).collect();
                    match ConvexPolygon::from_convex_polyline(pts) {
                        Some(poly) => parts.push((Pose2::default(), SharedShape::new(poly))),
                        None => log::warn!(
                            "skipping non-convex/degenerate collision polygon ({} points)",
                            points_mm.len()
                        ),
                    }
                }
                CollisionPrimitive::Cylinder { center_mm, radius_mm, .. } => {
                    let iso = Pose2::from_translation(Vec2::new(center_mm[0], center_mm[1]));
                    parts.push((iso, SharedShape::new(Ball::new(*radius_mm))));
                }
            }
        }
        if parts.is_empty() {
            return None;
        }
        let compound = Compound::new(parts);
        Some(Self { inner: SharedShape::new(compound) })
    }

    /// Fallback: circular shape of `radius_mm`. Used when a robot has no
    /// collision primitives configured.
    pub fn from_circle(radius_mm: f32) -> Self {
        Self { inner: SharedShape::new(Ball::new(radius_mm)) }
    }
}

/// Fraction `t ∈ [0, 1]` of the candidate translation that can be travelled
/// before the robot shape enters any collidable obstacle. Obstacles with
/// `height_mm == 0` are skipped (the ground never blocks).
pub fn sweep_max_fraction(
    robot_pose: (f32, f32, f32),
    motion_world: (f32, f32),
    robot_shape: &RobotShape,
    obstacles: &[Obstacle],
) -> f32 {
    if motion_world.0 == 0.0 && motion_world.1 == 0.0 {
        return 1.0;
    }
    let robot_iso = Pose2::new(Vec2::new(robot_pose.0, robot_pose.1), robot_pose.2);
    let robot_vel = Vec2::new(motion_world.0, motion_world.1);
    let static_vel = Vec2::ZERO;

    let opts = ShapeCastOptions {
        max_time_of_impact: 1.0,
        ..Default::default()
    };

    let mut t_min = 1.0_f32;
    for obs in obstacles {
        if obs.height_mm <= 0.0 {
            continue;
        }
        let [x0, y0, x1, y1] = obs.aabb_mm;
        let (hw, hh) = ((x1 - x0) * 0.5, (y1 - y0) * 0.5);
        if hw <= 0.0 || hh <= 0.0 {
            continue;
        }
        let obs_shape = Cuboid::new(Vec2::new(hw, hh));
        let obs_iso = Pose2::from_translation(Vec2::new((x0 + x1) * 0.5, (y0 + y1) * 0.5));

        match cast_shapes(
            &robot_iso,
            robot_vel,
            robot_shape.inner.0.as_ref(),
            &obs_iso,
            static_vel,
            &obs_shape,
            opts,
        ) {
            Ok(Some(hit)) => {
                // `time_of_impact` is the fraction of the velocity vector
                // travelled before touching the obstacle surface.
                let toi = hit.time_of_impact;
                if toi < t_min {
                    t_min = toi;
                }
            }
            _ => {}
        }
    }
    // Small safety margin so we stop short of touching; otherwise the next
    // tick's check can flip-flop between contact states.
    (t_min - 1e-4).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Finish;

    fn obs(x0: f32, y0: f32, x1: f32, y1: f32, h: f32) -> Obstacle {
        Obstacle {
            name: String::new(),
            aabb_mm: [x0, y0, x1, y1],
            height_mm: h,
            color: None,
            finish: Finish::Smooth,
            texture: None,
            use_playmat: false,
        }
    }

    #[test]
    fn circle_stops_at_wall() {
        let shape = RobotShape::from_circle(10.0);
        let obstacles = vec![obs(100.0, -50.0, 200.0, 50.0, 70.0)];
        let t = sweep_max_fraction((0.0, 0.0, 0.0), (200.0, 0.0), &shape, &obstacles);
        // Ball radius 10, center starts at 0, wall at x=100 → hit at t = 90/200 = 0.45
        assert!((t - 0.45).abs() < 0.02, "{t}");
    }

    #[test]
    fn ground_obstacle_ignored() {
        let shape = RobotShape::from_circle(10.0);
        let obstacles = vec![obs(-1000.0, -1000.0, 1000.0, 1000.0, 0.0)];
        let t = sweep_max_fraction((0.0, 0.0, 0.0), (50.0, 0.0), &shape, &obstacles);
        assert!((t - 1.0).abs() < 1e-3);
    }

    #[test]
    fn compound_triangle_cylinder_builds() {
        let prims = vec![
            CollisionPrimitive::Polygon {
                points_mm: vec![[-100.0, -100.0], [100.0, -100.0], [0.0, 100.0]],
                z_base_mm: 0.0,
                height_mm: 100.0,
            },
            CollisionPrimitive::Cylinder {
                center_mm: [0.0, 0.0],
                radius_mm: 50.0,
                z_base_mm: 100.0,
                height_mm: 50.0,
            },
        ];
        let shape = RobotShape::from_primitives(&prims).expect("compound");
        // Sanity: swept against a far wall returns 1.0 (no hit).
        let obstacles = vec![obs(5000.0, -100.0, 6000.0, 100.0, 70.0)];
        let t = sweep_max_fraction((0.0, 0.0, 0.0), (50.0, 0.0), &shape, &obstacles);
        assert!((t - 1.0).abs() < 1e-3);
    }
}
