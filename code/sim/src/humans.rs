//! Random-walking humans around the table to perturb the lidars with a
//! moving outside-the-playfield obstacle.

use std::thread;
use std::time::{Duration, Instant};

use flume::Sender;
use sim_protocol::Pose2D;

use crate::bridge::WorldUpdate;
use crate::config::{FieldConfig, HumansConfig};
use crate::world::{EntityKind, EntitySnapshot, World};

struct Walker {
    id: String,
    pose: Pose2D,
    target_mm: [f32; 2],
    speed_mm_s: f32,
    width_mm: f32,
    length_mm: f32,
    body_height_mm: f32,
    /// One of the 4 bands around the table; each walker stays in theirs so
    /// they never cross the playing area.
    band: Band,
}

#[derive(Clone, Copy, Debug)]
enum Band { North, South, East, West }

/// Start the animator thread. No-op when humans are disabled or no
/// [`HumansConfig::count_max`] was given.
pub fn spawn(
    humans_cfg: HumansConfig,
    field: FieldConfig,
    updates: Sender<WorldUpdate>,
    world: World,
) {
    if !humans_cfg.enabled || humans_cfg.count_max == 0 {
        return;
    }
    thread::Builder::new()
        .name("humans".into())
        .spawn(move || run(humans_cfg, field, updates, world))
        .expect("spawn humans thread");
}

fn run(
    cfg: HumansConfig,
    field: FieldConfig,
    updates: Sender<WorldUpdate>,
    world: World,
) {
    let mut rng = if cfg.seed == 0 {
        fastrand::Rng::new()
    } else {
        fastrand::Rng::with_seed(cfg.seed)
    };

    let count = if cfg.count_min >= cfg.count_max {
        cfg.count_min
    } else {
        cfg.count_min + rng.usize(0..=(cfg.count_max - cfg.count_min))
    };

    // "Roaming" outer rectangle that extends `margin_mm` past the table.
    let margin = cfg.margin_mm;
    let outer = [
        -margin,
        -margin,
        field.width_mm as f32 + margin,
        field.height_mm as f32 + margin,
    ];
    let table = [0.0, 0.0, field.width_mm as f32, field.height_mm as f32];

    let mut walkers: Vec<Walker> = (0..count)
        .map(|i| {
            let band = random_band(&mut rng);
            let start = random_in_band(band, outer, table, &mut rng);
            let target = random_in_band(band, outer, table, &mut rng);
            let body_h = cfg.body_height_min_mm
                + rng.f32() * (cfg.body_height_max_mm - cfg.body_height_min_mm).max(0.0);
            let width = cfg.width_min_mm
                + rng.f32() * (cfg.width_max_mm - cfg.width_min_mm).max(0.0);
            let length = width * (0.6 + rng.f32() * 0.3); // torso is flatter than wide
            let speed = cfg.walk_speed_min_mm_s
                + rng.f32() * (cfg.walk_speed_max_mm_s - cfg.walk_speed_min_mm_s).max(0.0);
            Walker {
                id: format!("human_{:02}", i),
                pose: Pose2D {
                    x_mm: start[0],
                    y_mm: start[1],
                    theta_rad: (target[1] - start[1]).atan2(target[0] - start[0]),
                },
                target_mm: target,
                speed_mm_s: speed,
                width_mm: width,
                length_mm: length,
                body_height_mm: body_h,
                band,
            }
        })
        .collect();

    // Spawn up front so Bevy can provision scenes / meshes once.
    for w in &walkers {
        let above = (w.body_height_mm - field.stand_height_mm).max(0.0);
        world.spawn(
            w.id.clone(),
            EntitySnapshot {
                kind: EntityKind::Human,
                pose: w.pose,
                width_mm: w.width_mm,
                length_mm: w.length_mm,
                height_above_table_mm: above,
                collision: None,
            },
        );
        updates
            .send(WorldUpdate::Spawn {
                id: w.id.clone(),
                kind: EntityKind::Human,
                pose: w.pose,
                width_mm: w.width_mm,
                length_mm: w.length_mm,
                body_height_mm: w.body_height_mm,
            })
            .ok();
    }

    let dt = Duration::from_millis(50);
    let dt_s = dt.as_secs_f32();
    let mut last = Instant::now();
    loop {
        let now = Instant::now();
        let elapsed = now.duration_since(last).as_secs_f32().min(0.5);
        last = now;

        for w in &mut walkers {
            w.step(elapsed.max(dt_s), outer, table, &mut rng);
            world.update_pose(&w.id, w.pose);
            updates
                .send(WorldUpdate::UpdatePose {
                    id: w.id.clone(),
                    pose: w.pose,
                })
                .ok();
        }
        thread::sleep(dt);
    }
}

impl Walker {
    fn step(
        &mut self,
        dt_s: f32,
        outer: [f32; 4],
        table: [f32; 4],
        rng: &mut fastrand::Rng,
    ) {
        let dx = self.target_mm[0] - self.pose.x_mm;
        let dy = self.target_mm[1] - self.pose.y_mm;
        let dist = (dx * dx + dy * dy).sqrt();
        let reach_thresh = 50.0_f32;
        if dist < reach_thresh {
            self.target_mm = random_in_band(self.band, outer, table, rng);
        } else {
            let step = (self.speed_mm_s * dt_s).min(dist);
            self.pose.x_mm += dx / dist * step;
            self.pose.y_mm += dy / dist * step;
            self.pose.theta_rad = dy.atan2(dx);
        }
    }
}

fn random_band(rng: &mut fastrand::Rng) -> Band {
    match rng.u32(0..4) {
        0 => Band::North,
        1 => Band::South,
        2 => Band::East,
        _ => Band::West,
    }
}

/// Random point inside the band, strictly outside the table rectangle.
/// North/South bands include the corners; East/West do not (to avoid
/// double-coverage and ambiguous band membership).
fn random_in_band(
    band: Band,
    outer: [f32; 4],
    table: [f32; 4],
    rng: &mut fastrand::Rng,
) -> [f32; 2] {
    let lerp = |a: f32, b: f32, rng: &mut fastrand::Rng| a + rng.f32() * (b - a);
    match band {
        Band::North => [lerp(outer[0], outer[2], rng), lerp(table[3], outer[3], rng)],
        Band::South => [lerp(outer[0], outer[2], rng), lerp(outer[1], table[1], rng)],
        Band::East => [lerp(table[2], outer[2], rng), lerp(table[1], table[3], rng)],
        Band::West => [lerp(outer[0], table[0], rng), lerp(table[1], table[3], rng)],
    }
}
