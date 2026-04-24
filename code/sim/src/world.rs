//! Shared, thread-safe registry of every moving entity in the simulated
//! world (robots + humans). Used by the per-connection server threads to
//! raycast against other entities' 2D silhouettes.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use sim_protocol::{Pose2D, RobotKind};

use crate::config::CollisionPrimitive;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntityKind {
    Robot(RobotKind),
    Human,
}

#[derive(Clone, Debug)]
pub struct EntitySnapshot {
    pub kind: EntityKind,
    pub pose: Pose2D,
    pub width_mm: f32,
    pub length_mm: f32,
    /// Effective height above the table top, for raycast filtering. A
    /// robot's height is its body height (its feet rest on the table); a
    /// human's height is `max(0, body_height - stand_height)` — how much
    /// of their body pokes above the table.
    pub height_above_table_mm: f32,
    /// Optional more-accurate 2D silhouette for raycast. When set, replaces
    /// the `width_mm × length_mm` AABB and enables per-primitive vertical
    /// filtering. `Arc` so the raycast doesn't copy the primitive list on
    /// every call.
    pub collision: Option<Arc<Vec<CollisionPrimitive>>>,
}

#[derive(Clone, Default)]
pub struct World {
    inner: Arc<Mutex<HashMap<String, EntitySnapshot>>>,
}

impl World {
    pub fn new() -> Self { Self::default() }

    pub fn spawn(&self, id: String, snap: EntitySnapshot) {
        self.inner.lock().unwrap().insert(id, snap);
    }

    pub fn update_pose(&self, id: &str, pose: Pose2D) {
        if let Some(s) = self.inner.lock().unwrap().get_mut(id) {
            s.pose = pose;
        }
    }

    pub fn remove(&self, id: &str) {
        self.inner.lock().unwrap().remove(id);
    }

    /// Segments for every OTHER entity's silhouette visible to a sensor
    /// beam at `sensor_height_mm`. Appended to the static `field_walls`
    /// list.
    ///
    /// - If the entity has `collision` primitives, each primitive is
    ///   filtered by its own `[z_base, z_base + height]` range (so low
    ///   chassis parts don't hide behind a tall lidar beam, for example).
    /// - Otherwise the entity's AABB is used, filtered by
    ///   `height_above_table_mm`.
    pub fn visible_segments(
        &self,
        skip_id: &str,
        sensor_height_mm: f32,
        field_walls: &[[f32; 4]],
    ) -> Vec<[f32; 4]> {
        const CYL_N: usize = 16;
        let mut out = field_walls.to_vec();
        let guard = self.inner.lock().unwrap();
        for (id, s) in guard.iter() {
            if id == skip_id {
                continue;
            }
            if let Some(prims) = &s.collision {
                for p in prims.iter() {
                    if !p.visible_at(sensor_height_mm) {
                        continue;
                    }
                    for seg in p.to_segments_local(CYL_N) {
                        out.push(transform_segment(seg, s.pose));
                    }
                }
            } else {
                if s.height_above_table_mm < sensor_height_mm {
                    continue;
                }
                push_bbox_segments(&mut out, s.pose, s.width_mm, s.length_mm);
            }
        }
        out
    }
}

/// Rotate + translate a robot-local segment into world coordinates.
fn transform_segment(seg: [f32; 4], pose: Pose2D) -> [f32; 4] {
    let c = pose.theta_rad.cos();
    let s = pose.theta_rad.sin();
    let tx = |x: f32, y: f32| -> (f32, f32) {
        (pose.x_mm + x * c - y * s, pose.y_mm + x * s + y * c)
    };
    let (ax, ay) = tx(seg[0], seg[1]);
    let (bx, by) = tx(seg[2], seg[3]);
    [ax, ay, bx, by]
}

fn push_bbox_segments(walls: &mut Vec<[f32; 4]>, pose: Pose2D, w: f32, l: f32) {
    let hx = l / 2.0;
    let hy = w / 2.0;
    let c = pose.theta_rad.cos();
    let s = pose.theta_rad.sin();
    let corner = |dx: f32, dy: f32| -> (f32, f32) {
        (pose.x_mm + dx * c - dy * s, pose.y_mm + dx * s + dy * c)
    };
    let p0 = corner(hx, hy);
    let p1 = corner(hx, -hy);
    let p2 = corner(-hx, -hy);
    let p3 = corner(-hx, hy);
    walls.push([p0.0, p0.1, p1.0, p1.1]);
    walls.push([p1.0, p1.1, p2.0, p2.1]);
    walls.push([p2.0, p2.1, p3.0, p3.1]);
    walls.push([p3.0, p3.1, p0.0, p0.1]);
}
