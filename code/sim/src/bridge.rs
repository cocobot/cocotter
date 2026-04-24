//! One-way channel from the IPC server / human animator threads to the
//! Bevy main-loop app. Each update tells Bevy to spawn, update or despawn
//! an entity.

use flume::{Receiver, Sender};
use sim_protocol::Pose2D;

use crate::world::EntityKind;

#[derive(Debug, Clone)]
pub enum WorldUpdate {
    Spawn {
        id: String,
        kind: EntityKind,
        pose: Pose2D,
        width_mm: f32,
        length_mm: f32,
        /// Full body height, for 3D rendering (robots rendered atop the
        /// stand, humans rendered on the floor).
        body_height_mm: f32,
    },
    UpdatePose { id: String, pose: Pose2D },
    Despawn { id: String },
}

pub fn channel() -> (Sender<WorldUpdate>, Receiver<WorldUpdate>) {
    flume::unbounded()
}
