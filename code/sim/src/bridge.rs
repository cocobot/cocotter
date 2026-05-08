//! One-way channel from the IPC server / human animator threads to the
//! Bevy main-loop app. Each update tells Bevy to spawn, update or despawn
//! an entity.

use flume::{Receiver, Sender};
use sim_protocol::{DebugVolume, Pose2D};

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
    /// Wire-order neopixel strip frame. The sim app maps it onto the
    /// robot's configured fixtures.
    Neopixels { id: String, pixels: Vec<[u8; 3]> },
    /// Ground-lidar hit distances (mm) in the same order as the
    /// configured `ground_lidars`. The sim app clips each beam's
    /// cylinder visual to the hit distance.
    GroundLidarHits { id: String, distances_mm: Vec<f32> },
    /// One LD06 packet's worth of hits in the body frame: 12 rays
    /// uniformly distributed over `[start_angle_deg, end_angle_deg]`.
    /// Pushed alongside the wire `Ld06Bytes` so Bevy can render the
    /// rotor's current slice as 12 translucent beams without parsing
    /// the encoded packet.
    Ld06Hits {
        id: String,
        start_angle_deg: f32,
        end_angle_deg: f32,
        distances_mm: [u16; 12],
    },
    /// Picotter actuator state for galipeur. Sent diff-only by the
    /// sim's picotter emulator. Drives the joint hierarchy in the 3D
    /// renderer (rail translation, arm rotation, clamp servos).
    /// Module index follows `RobotSide::module()`: Left=0, Back=1,
    /// Right=2.
    ActuatorState { id: String, modules: [ModuleActuators; 3] },
    /// Translucent debug volumes pushed by the robot's mock board.
    /// The sim re-renders them as children of the robot entity so
    /// they track the pose; toggle visibility with the `V` shortcut.
    /// Each update replaces the entire previous set for that robot.
    DebugVolumes { id: String, volumes: Vec<DebugVolume> },
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ModuleActuators {
    /// Linear stage position, raw u16 from CAN (0..65535).
    pub translation_position: u16,
    /// Per-arm position, raw u16 from CAN. Arms are 0..4.
    pub arm_positions: [u16; 4],
    /// Per-clamp servo position, raw u16 from CAN. Servos are 0..3
    /// (Rotate, Left, Right per clamp definition).
    pub clamp_positions: [u16; 3],
}

pub fn channel() -> (Sender<WorldUpdate>, Receiver<WorldUpdate>) {
    flume::unbounded()
}
