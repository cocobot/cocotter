//! Articulated joints for robot models.
//!
//! Each joint declared in `[[robot.model.joints]]` (see
//! `crate::config::JointSpec`) becomes a Bevy entity with:
//! - the joint's own .glb loaded via `SceneRoot`,
//! - a `Transform` driven by the actuator state forwarded by
//!   `picotter_emu` (`WorldUpdate::ActuatorState`),
//! - a `JointEntity` tag that captures the binding so the renderer
//!   knows which scalar to read each frame.
//!
//! Joints are parented either to the robot's `base` visual (the static
//! `[robot.model] visual` glb) or to another joint by `name`, building
//! an arbitrary tree. The `Transform` propagation is Bevy's stock
//! parent/child mechanism, so a translation parent automatically
//! drags its child rotations along.
//!
//! M13d wires the hierarchy + asset loading. M13e drives the
//! transforms from `WorldUpdate::ActuatorState`.

use std::collections::HashMap;

use bevy::ecs::query::QueryFilter;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use sim_protocol::RobotKind;

use crate::config::{JointActuator, JointKind, JointSpec, JointStateBinding};

/// Bundled joint resources + transform query, packed into a single
/// `SystemParam` so `drain_bridge` doesn't blow past Bevy's per-system
/// parameter limit.
#[derive(SystemParam)]
pub struct JointsParam<'w, 's> {
    pub assets: Option<Res<'w, JointAssets>>,
    pub entities: ResMut<'w, JointEntities>,
    pub transforms: Query<
        'w,
        's,
        (&'static JointEntity, &'static mut Transform),
        (
            Without<crate::app::SimEntityTag>,
            Without<crate::app::NeopixelLed>,
            Without<crate::app::GroundLidarBeam>,
            // Required so `Ld06Param.beam_transforms` (which mutates
            // Transform on Ld06Beam entities) is provably disjoint with
            // this query. Without it Bevy 0.18's static disjointness
            // checker panics in `drain_bridge` at startup.
            Without<crate::app::Ld06Beam>,
        ),
    >,
}

/// Asset handles for every joint of every robot kind, loaded once at
/// startup. Keyed by `(kind, joint_name)` so multiple robots of the
/// same kind share scene instances.
#[derive(Resource, Default)]
pub struct JointAssets {
    pub by_kind: HashMap<RobotKind, HashMap<String, Handle<Scene>>>,
}

impl JointAssets {
    pub fn handle(&self, kind: RobotKind, name: &str) -> Option<Handle<Scene>> {
        self.by_kind
            .get(&kind)
            .and_then(|m| m.get(name))
            .cloned()
    }
}

/// `(robot_id, joint_or_base_name) → spawned entity`.
#[derive(Resource, Default)]
pub struct JointEntities {
    pub map: HashMap<(String, String), Entity>,
}

impl JointEntities {
    pub fn insert(&mut self, robot_id: &str, joint_name: &str, entity: Entity) {
        self.map
            .insert((robot_id.to_string(), joint_name.to_string()), entity);
    }

    pub fn get(&self, robot_id: &str, joint_name: &str) -> Option<Entity> {
        self.map.get(&(robot_id.to_string(), joint_name.to_string())).copied()
    }

    pub fn remove_robot(&mut self, robot_id: &str) {
        self.map.retain(|(rid, _), _| rid != robot_id);
    }
}

/// Tag + binding kept on every spawned joint entity. The
/// `apply_actuator_state` system reads `binding`, fetches the matching
/// scalar from `WorldUpdate::ActuatorState`, and writes the entity's
/// `Transform` from `(neutral, axis, kind, range)`.
#[derive(Component)]
pub struct JointEntity {
    pub kind: JointKind,
    /// Pivot axis in the parent's local frame (unit vector).
    pub axis: Vec3,
    /// `[u16=0 → out, u16=65535 → out]` mapped on either translation
    /// (mm) or rotation (rad).
    pub range_lo: f32,
    pub range_hi: f32,
    /// Neutral (no-actuator-delta) translation, in metres. Captured at
    /// spawn from `pivot_offset_mm`.
    pub neutral_translation: Vec3,
    pub binding: JointStateBinding,
}

impl JointEntity {
    /// Compute the joint's `Transform` for a given raw `u16` actuator
    /// reading. The parent's transform is composed by Bevy
    /// automatically through the entity hierarchy.
    pub fn transform_for(&self, raw: u16) -> Transform {
        let f01 = (raw as f32) / 65535.0;
        let val = self.range_lo + f01 * (self.range_hi - self.range_lo);
        match self.kind {
            JointKind::Translation => {
                let delta = self.axis * val;
                Transform::from_translation(self.neutral_translation + delta)
            }
            JointKind::Rotation => Transform {
                translation: self.neutral_translation,
                rotation: Quat::from_axis_angle(self.axis, val),
                scale: Vec3::ONE,
            },
        }
    }
}

/// Spawn the configured joints as descendants of `base_entity`. The
/// base entity must already exist. Joints are spawned in declaration
/// order; toml is expected to declare parents before children. Returns
/// the number of joints actually spawned (skips entries whose .glb
/// asset wasn't loaded — which means the file doesn't exist yet).
pub fn spawn_joint_tree(
    commands: &mut Commands,
    robot_id: &str,
    kind: RobotKind,
    base_entity: Entity,
    joints: &[JointSpec],
    assets: &JointAssets,
    registry: &mut JointEntities,
) -> usize {
    // Index parent entities as we go. "base" → the SceneRoot entity
    // for the static visual. Each joint's name → its own pivot.
    let mut by_name: HashMap<String, Entity> = HashMap::new();
    by_name.insert("base".into(), base_entity);
    registry.insert(robot_id, "base", base_entity);

    let mut spawned = 0;
    for j in joints {
        let Some(parent) = by_name.get(&j.parent).copied() else {
            log::warn!(
                "[joints] {robot_id}: joint {:?} parent {:?} not yet spawned — declare parents first in TOML",
                j.name, j.parent
            );
            continue;
        };
        let Some(scene) = assets.handle(kind, &j.name) else {
            log::warn!(
                "[joints] {robot_id}: joint {:?} has no loaded scene — file {:?} missing or kind mismatch",
                j.name, j.file
            );
            continue;
        };

        let neutral = Vec3::new(
            j.pivot_offset_mm[0] * crate::app::MM,
            j.pivot_offset_mm[1] * crate::app::MM,
            j.pivot_offset_mm[2] * crate::app::MM,
        );
        let axis_raw = Vec3::new(j.axis[0], j.axis[1], j.axis[2]);
        let axis = axis_raw.try_normalize().unwrap_or(Vec3::Y);
        let (lo, hi) = match j.kind {
            JointKind::Translation => {
                let r = j.range_mm.unwrap_or([0.0, 0.0]);
                (r[0] * crate::app::MM, r[1] * crate::app::MM)
            }
            JointKind::Rotation => {
                let r = j.range_rad.unwrap_or([0.0, 0.0]);
                (r[0], r[1])
            }
        };

        let tag = JointEntity {
            kind: j.kind,
            axis,
            range_lo: lo,
            range_hi: hi,
            neutral_translation: neutral,
            binding: j.state.clone(),
        };
        // `transform_for(0)` = neutral (M13d behaviour).
        let initial_transform = tag.transform_for(0);

        let entity = commands
            .spawn((
                Name::new(format!("joint:{}/{}", robot_id, j.name)),
                SceneRoot(scene),
                initial_transform,
                tag,
            ))
            .insert(ChildOf(parent))
            .id();

        registry.insert(robot_id, &j.name, entity);
        by_name.insert(j.name.clone(), entity);
        spawned += 1;
    }
    spawned
}

/// Drive every joint's `Transform` from the latest actuator snapshot.
/// Called from `drain_bridge` on `WorldUpdate::ActuatorState`. M13e.
pub fn apply_actuator_state<F: QueryFilter>(
    robot_id: &str,
    modules: &[crate::bridge::ModuleActuators; 3],
    registry: &JointEntities,
    joints: &[JointSpec],
    transforms: &mut Query<(&JointEntity, &mut Transform), F>,
) {
    for j in joints {
        let Some(entity) = registry.get(robot_id, &j.name) else { continue };
        let Ok((tag, mut tr)) = transforms.get_mut(entity) else { continue };
        let m = &modules[tag.binding.module.idx()];
        let raw = match tag.binding.actuator {
            JointActuator::Translation => m.translation_position,
            JointActuator::Arm => *m
                .arm_positions
                .get(tag.binding.index as usize)
                .unwrap_or(&0),
            JointActuator::Clamp => *m
                .clamp_positions
                .get(tag.binding.index as usize)
                .unwrap_or(&0),
        };
        *tr = tag.transform_for(raw);
    }
}
