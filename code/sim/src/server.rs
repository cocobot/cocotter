//! IPC server: Unix socket listener + per-connection robot handler.
//!
//! Each accepted connection gets its own thread. The thread handles the
//! handshake, loads the robot's kinematics from config, integrates pose on
//! each received `MotorConsignsHolo` and publishes the pose to the Bevy
//! bridge via a `WorldUpdate` channel.

use std::io;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use flume::Sender;
use sim_protocol::{recv_msg, send_msg, Pose2D, RobotKind, SimMsgC2S, SimMsgS2C};

use crate::bridge::WorldUpdate;
use crate::collide::RobotShape;
use crate::config::{BBoxConfig, CollisionPrimitive, Config, KinematicsConfig};
use crate::controls::ConnRegistry;
use crate::kinematics::{DiffState, HoloState};
use crate::ld06_encoder;
use crate::picotter_emu::PicotterEmu;
use crate::raycast;
use crate::world::{EntityKind, EntitySnapshot, World};

/// Build a parry2d shape describing the robot's footprint. When collision
/// primitives are declared in config we use their exact union; otherwise
/// fall back to a bounding ball sized to the bbox long side.
fn robot_shape_from_config(
    bbox: &BBoxConfig,
    collision: Option<&[CollisionPrimitive]>,
) -> RobotShape {
    if let Some(prims) = collision {
        if let Some(shape) = RobotShape::from_primitives(prims) {
            return shape;
        }
    }
    RobotShape::from_circle(0.5 * bbox.width_mm.max(bbox.length_mm))
}

pub fn listen_forever(
    socket_path: &str,
    config: std::sync::Arc<std::sync::RwLock<Config>>,
    updates: Sender<WorldUpdate>,
    world: World,
    registry: ConnRegistry,
    match_state: crate::controls::MatchStateLock,
) -> io::Result<()> {
    if Path::new(socket_path).exists() {
        std::fs::remove_file(socket_path).ok();
    }
    let listener = UnixListener::bind(socket_path)?;
    log::info!("sim listening on {socket_path}");

    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                // Snapshot the live config at handshake time so reloads
                // via the chord's `R` reset propagate to the next robot.
                let cfg = config.read().unwrap().clone();
                let updates = updates.clone();
                let world = world.clone();
                let registry = registry.clone();
                let match_state = match_state.clone();
                thread::Builder::new()
                    .name("sim-conn".into())
                    .spawn(move || {
                        if let Err(e) = handle_connection(s, cfg, updates, world, registry, match_state) {
                            // Most "errors" here are just the robot
                            // process exiting cleanly (UnexpectedEof
                            // when the framing read can't be filled).
                            // Only surface genuinely unexpected I/O
                            // failures.
                            if e.kind() == io::ErrorKind::UnexpectedEof
                                || e.kind() == io::ErrorKind::BrokenPipe
                                || e.kind() == io::ErrorKind::ConnectionReset
                            {
                                log::info!("robot disconnected ({})", e.kind());
                            } else {
                                log::error!("connection ended: {e}");
                            }
                        }
                    })
                    .expect("spawn sim-conn");
            }
            Err(e) => log::error!("accept failed: {e}"),
        }
    }
    Ok(())
}

fn handle_connection(
    stream: UnixStream,
    config: Config,
    updates: Sender<WorldUpdate>,
    world: World,
    registry: ConnRegistry,
    match_state: crate::controls::MatchStateLock,
) -> io::Result<()> {
    let hello: SimMsgC2S = recv_msg(&stream)?;
    let (robot_id, kind, requested_start, requested_side) = match hello {
        SimMsgC2S::Hello {
            version,
            robot_id,
            kind,
            requested_start,
            requested_side,
            ..
        } => {
            if version != sim_protocol::PROTOCOL_VERSION {
                log::warn!(
                    "protocol version mismatch: robot {version}, sim {}",
                    sim_protocol::PROTOCOL_VERSION
                );
            }
            (robot_id, kind, requested_start, requested_side)
        }
        other => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("expected Hello, got {:?}", other),
            ));
        }
    };

    // Refuse the spawn if the match has already started — joining the
    // table mid-match makes no sense and the strat would skip the
    // setup phase. Operator must `K` (kill-all) or `R` (reset) first.
    if match_state.get() != crate::controls::MatchState::Pregame {
        log::warn!(
            "refusing Hello from '{robot_id}': match state is {:?}",
            match_state.get()
        );
        send_msg(
            &stream,
            &SimMsgS2C::Shutdown {
                reason: "match already running — sim is not in Pregame state".into(),
            },
        )
        .ok();
        return Ok(());
    }

    // Default to `Left` when the launcher didn't pass `SIDE` — this
    // matches the existing implicit behaviour for adhoc `cargo run`s.
    let side = requested_side.unwrap_or(crate::config::Side::Left);
    let start = requested_start.unwrap_or_else(|| config.start_pose(kind, side));
    let sim_tick_ms: u16 = 10;
    log::info!(
        "robot '{robot_id}' ({:?}) on {:?} starting at {:?}",
        kind, side, start
    );

    send_msg(&stream, &SimMsgS2C::HelloAck { assigned_start: start, sim_tick_ms })?;
    // Echo the side back: the mock colour pin reads it from the sim
    // client, so the strat's `color.is_high() → Team::Left` mapping
    // works even when the launcher didn't set the `TEAM` env var.
    send_msg(&stream, &SimMsgS2C::Side { side })?;
    let robot_cfg = match kind {
        RobotKind::Galipeur => &config.galipeur,
        RobotKind::Pami => &config.pami,
        // The adversary never connects via IPC: it lives entirely
        // inside the sim and is driven from ZQSD/gamepad. Refuse the
        // handshake — we don't have a `RobotConfig` for it.
        RobotKind::Adversary => {
            log::warn!("refusing Hello from {robot_id}: kind=Adversary is sim-only");
            send_msg(
                &stream,
                &SimMsgS2C::Shutdown {
                    reason: "Adversary kind is sim-only — no IPC".into(),
                },
            )
            .ok();
            return Ok(());
        }
    };
    let bbox = &robot_cfg.bbox;
    let collision = robot_cfg
        .model
        .as_ref()
        .filter(|m| !m.collision.is_empty())
        .map(|m| std::sync::Arc::new(m.collision.clone()));
    let snap = EntitySnapshot {
        kind: EntityKind::Robot(kind),
        pose: start,
        width_mm: bbox.width_mm,
        length_mm: bbox.length_mm,
        // Robots sit on the table: body height == effective height.
        height_above_table_mm: bbox.height_mm,
        collision,
    };
    world.spawn(robot_id.clone(), snap.clone());
    updates
        .send(WorldUpdate::Spawn {
            id: robot_id.clone(),
            kind: snap.kind,
            pose: snap.pose,
            width_mm: snap.width_mm,
            length_mm: snap.length_mm,
            body_height_mm: bbox.height_mm,
        })
        .ok();

    // Register the connection so the "kill all" shortcut can shutdown the
    // stream from the UI thread. `try_clone` gives us an owned handle; the
    // original `stream` stays with this thread for read/write.
    if let Ok(clone) = stream.try_clone() {
        registry.register(robot_id.clone(), clone);
    }

    let result = match kind {
        RobotKind::Galipeur => handle_galipeur(stream, &config, start, sim_tick_ms, &robot_id, &updates, &world),
        RobotKind::Pami => handle_pami(stream, &config, start, sim_tick_ms, &robot_id, &updates, &world),
        // Already short-circuited above before this point.
        RobotKind::Adversary => unreachable!(),
    };

    registry.unregister(&robot_id);
    updates.send(WorldUpdate::Despawn { id: robot_id.clone() }).ok();
    world.remove(&robot_id);
    result
}

fn handle_galipeur(
    stream: UnixStream,
    config: &Config,
    start: Pose2D,
    sim_tick_ms: u16,
    robot_id: &str,
    updates: &Sender<WorldUpdate>,
    world: &World,
) -> io::Result<()> {
    let (v2c, e2p) = match config.galipeur.kinematics.as_ref() {
        Some(KinematicsConfig::Holo {
            velocities_to_consigns,
            encoders_to_position,
        }) => (*velocities_to_consigns, *encoders_to_position),
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "galipeur.kinematics missing or not holonomic",
            ));
        }
    };
    let mut state = HoloState::new(start, v2c, e2p);
    let dt_s = (sim_tick_ms as f32) / 1000.0;
    let robot_shape = robot_shape_from_config(
        &config.galipeur.bbox,
        config.galipeur.model.as_ref().map(|m| m.collision.as_slice()),
    );
    let static_walls = config
        .field
        .obstacle_segments_visible_from(config.galipeur.lidar_height_mm);
    // Ground lidars sit near the chassis base; use the first fixture's Z
    // as a representative sensor height for the per-beam visibility cull.
    // Works while all ground lidars live at the same height (they do).
    let ground_lidar_z = config
        .galipeur
        .ground_lidars
        .first()
        .map(|gl| gl.position_mm[2])
        .unwrap_or(0.0);
    let static_walls_ground = config.field.obstacle_segments_visible_from(ground_lidar_z);
    log::info!(
        "[{robot_id}] ground-lidar z={:.1}mm, walls_visible={} segments, configured_lidars={}",
        ground_lidar_z,
        static_walls_ground.len(),
        config.galipeur.ground_lidars.len(),
    );
    let mut lidar = Ld06Emitter::default();
    let mut emu = PicotterEmu::default();
    let mut last_battery = Instant::now();
    let mut last_pose_log = Instant::now();
    let mut last_ground_lidar = Instant::now();
    let mut lidar_seq: u8 = 0;
    let battery_period = Duration::from_secs(1);
    let pose_log_period = Duration::from_millis(500);
    let ground_lidar_period = Duration::from_millis(250);
    let mut can_frames_in: u64 = 0;
    let mut can_frames_out: u64 = 0;
    // Last `ActuatorState` we forwarded; used for diff-only emission so
    // a steady-state robot doesn't keep flooding the bridge.
    let mut last_actuator_snapshot = emu.snapshot();

    // The asserv starts at (0,0,0) inside the robot; it's the robot's
    // responsibility to call `reset_position(start)` after it reads its
    // assigned start pose from the sim (see galipeur/src/routines.rs for
    // the non-espidf cfg).
    loop {
        // Advance any in-flight picotter movement before handling
        // the next message. Each completion emits a `*Status` frame
        // with `position_reached=true` so the strat unblocks; the
        // interpolated positions feed the renderer through the
        // diff-only `ActuatorState` emission below.
        let now = std::time::Instant::now();
        for reply in emu.tick(now) {
            send_msg(&stream, &reply)?;
            can_frames_out += 1;
        }
        let snap_after_tick = emu.snapshot();
        if snap_after_tick != last_actuator_snapshot {
            last_actuator_snapshot = snap_after_tick;
            updates
                .send(WorldUpdate::ActuatorState {
                    id: robot_id.to_string(),
                    modules: snap_after_tick,
                })
                .ok();
        }

        let msg: SimMsgC2S = recv_msg(&stream)?;
        match msg {
            SimMsgC2S::MotorConsignsHolo { values } => {
                let (enc_delta, gyro_delta) =
                    state.step(values, dt_s, &config.field.obstacles, &robot_shape);
                send_msg(&stream, &SimMsgS2C::EncoderDeltaHolo { delta: enc_delta })?;
                send_msg(&stream, &SimMsgS2C::GyroDelta { d_theta_rad: gyro_delta })?;
                // Cheap sampling log to see what the asserv actually outputs
                // without drowning the terminal (~once per 500 ms).
                if last_pose_log.elapsed() >= pose_log_period {
                    log::debug!(
                        "[{robot_id}] consigns=[{:.1}, {:.1}, {:.1}]  enc_delta=[{:.2}, {:.2}, {:.2}]  gyro={:.4}",
                        values[0], values[1], values[2],
                        enc_delta[0], enc_delta[1], enc_delta[2],
                        gyro_delta
                    );
                }

                // Drain LD06 packets due since the last tick. Rotor runs in
                // continuous time so a single sim tick produces 0, 1, or 2
                // packets depending on the accumulator phase.
                let walls = world.visible_segments(robot_id, config.galipeur.lidar_height_mm, &static_walls);
                for packet in lidar.tick(state.pose, &walls, sim_tick_ms, config.galipeur.lidar_angle_offset_deg) {
                    send_msg(&stream, &SimMsgS2C::Ld06Bytes { bytes: packet.bytes.to_vec() })?;
                    // Re-publish the same hits to Bevy so the `L` toggle
                    // can render the rotor's current slice.
                    updates
                        .send(WorldUpdate::Ld06Hits {
                            id: robot_id.to_string(),
                            start_angle_deg: packet.start_deg,
                            end_angle_deg: packet.end_deg,
                            distances_mm: packet.distances,
                        })
                        .ok();
                }

                if last_battery.elapsed() >= battery_period {
                    last_battery = Instant::now();
                    send_msg(&stream, &PicotterEmu::battery_status_msg())?;
                    // Also emit ground status (all three sensors happy)
                    // so the galipeur doesn't stay in its "not on
                    // ground" init state forever.
                    send_msg(&stream, &PicotterEmu::ground_status_msg())?;
                    can_frames_out += 2;
                }

                if last_ground_lidar.elapsed() >= ground_lidar_period
                    && !config.galipeur.ground_lidars.is_empty()
                {
                    last_ground_lidar = Instant::now();
                    let ground_walls = world.visible_segments(
                        robot_id,
                        ground_lidar_z,
                        &static_walls_ground,
                    );
                    let cos_t = state.pose.theta_rad.cos();
                    let sin_t = state.pose.theta_rad.sin();
                    // Pack ground lidars two per module to fit the
                    // existing LidarStatus frame layout (distance_0,
                    // distance_1). Unused slots stay at 0.
                    const NUM_MODULES: usize = 3;
                    let mut dists: [[u16; 2]; NUM_MODULES] = [[0; 2]; NUM_MODULES];
                    for (i, gl) in config.galipeur.ground_lidars.iter().enumerate() {
                        // galipeur main.rs wires CAN modules so each
                        // module owns one lidar from each trio:
                        //   module 0 = ground_lidars[0] + [3]
                        //   module 1 = ground_lidars[1] + [4]
                        //   module 2 = ground_lidars[2] + [5]
                        // Match that packing here so `get_plane_offset`
                        // sees the right pair on each side.
                        let module = i % 3;
                        let lane = i / 3;
                        if module >= NUM_MODULES {
                            break;
                        }
                        if !gl.enabled {
                            continue;
                        }
                        // Body (bx=right, by=forward) in motor frame,
                        // rotated into strat world by standard R(θ).
                        // Raycast uses strat convention (angle 0 → +Y),
                        // so subtract π/2 from the standard world angle.
                        let bx = gl.position_mm[0];
                        let by = gl.position_mm[1];
                        let world_x = state.pose.x_mm + cos_t * bx - sin_t * by;
                        let world_y = state.pose.y_mm + sin_t * bx + cos_t * by;
                        let world_theta = state.pose.theta_rad + gl.theta_rad
                            - core::f32::consts::FRAC_PI_2;
                        let d = raycast::raycast(
                            (world_x, world_y),
                            world_theta,
                            gl.max_range_mm,
                            &ground_walls,
                        );
                        dists[module][lane] =
                            d.round().clamp(0.0, 65535.0) as u16;
                    }
                    lidar_seq = lidar_seq.wrapping_add(1);
                    for (m, [d0, d1]) in dists.iter().enumerate() {
                        if *d0 == 0 && *d1 == 0 {
                            continue;
                        }
                        send_msg(
                            &stream,
                            &PicotterEmu::lidar_status_msg(m as u8, *d0, *d1, lidar_seq),
                        )?;
                        can_frames_out += 1;
                    }
                    // Forward the hit distances to Bevy so it can clip
                    // the red beam cylinders at the obstacle.
                    let mut per_lidar_hits: Vec<f32> =
                        Vec::with_capacity(config.galipeur.ground_lidars.len());
                    for (i, gl) in config.galipeur.ground_lidars.iter().enumerate() {
                        // galipeur main.rs wires CAN modules so each
                        // module owns one lidar from each trio:
                        //   module 0 = ground_lidars[0] + [3]
                        //   module 1 = ground_lidars[1] + [4]
                        //   module 2 = ground_lidars[2] + [5]
                        // Match that packing here so `get_plane_offset`
                        // sees the right pair on each side.
                        let module = i % 3;
                        let lane = i / 3;
                        if module >= NUM_MODULES {
                            break;
                        }
                        if !gl.enabled {
                            // The visual beam is also skipped on the
                            // Bevy side; keep the indexing 1:1 with the
                            // spawned beams.
                            continue;
                        }
                        per_lidar_hits.push(dists[module][lane] as f32);
                    }
                    log::debug!(
                        "[{robot_id}] ground lidar hits (mm) = {:?}",
                        per_lidar_hits,
                    );
                    updates
                        .send(WorldUpdate::GroundLidarHits {
                            id: robot_id.to_string(),
                            distances_mm: per_lidar_hits,
                        })
                        .ok();
                }

                if last_pose_log.elapsed() >= pose_log_period {
                    last_pose_log = Instant::now();
                    log::debug!(
                        "[{robot_id}] pose x={:.1} y={:.1} θ={:.3}  can_in={} can_out={}",
                        state.pose.x_mm, state.pose.y_mm, state.pose.theta_rad,
                        can_frames_in, can_frames_out,
                    );
                }

                world.update_pose(robot_id, state.pose);
                updates
                    .send(WorldUpdate::UpdatePose { id: robot_id.to_string(), pose: state.pose })
                    .ok();
            }
            SimMsgC2S::MotorsBreak { .. } => { /* ignored */ }
            SimMsgC2S::CanFrame { id, data, len } => {
                can_frames_in += 1;
                for reply in emu.on_can_frame(id, &data, len) {
                    send_msg(&stream, &reply)?;
                    can_frames_out += 1;
                }
                // Position interpolation is advanced by `emu.tick()`
                // at the top of each loop iteration, so no extra
                // diff-check is needed here.
            }
            SimMsgC2S::RomeBytes { .. } => { /* deferred */ }
            SimMsgC2S::MotorConsignsDiff { .. } => {
                log::warn!("[{robot_id}] ignored MotorConsignsDiff on galipeur");
            }
            SimMsgC2S::Hello { .. } => {
                log::warn!("[{robot_id}] unexpected Hello after handshake");
            }
            SimMsgC2S::Ext { tag, .. } => log::debug!("[{robot_id}] Ext {tag} ignored"),
            SimMsgC2S::NeopixelFrame { pixels } => {
                updates
                    .send(WorldUpdate::Neopixels {
                        id: robot_id.to_string(),
                        pixels,
                    })
                    .ok();
            }
            SimMsgC2S::Teleport { pose } => {
                log::info!(
                    "[{robot_id}] teleport to ({:.1}, {:.1}, θ={:.3})",
                    pose.x_mm, pose.y_mm, pose.theta_rad
                );
                state.pose = pose;
                world.update_pose(robot_id, state.pose);
                updates
                    .send(WorldUpdate::UpdatePose {
                        id: robot_id.to_string(),
                        pose: state.pose,
                    })
                    .ok();
            }
            SimMsgC2S::DebugVolumes { volumes } => {
                updates
                    .send(WorldUpdate::DebugVolumes {
                        id: robot_id.to_string(),
                        volumes,
                    })
                    .ok();
            }
        }
    }
}

/// LD06 emitter modeled on the real device's continuous rotor.
///
/// The rotor advances in real time and we sample one 12-point packet every
/// `PACKET_PERIOD_US`. Because the period is not a clean divisor of the
/// revolution period, packet boundaries drift slightly across revolutions —
/// `start_angle` therefore varies between snapshots, matching the pattern in
/// `galipeur/log_lidar.log`. Tearing emerges naturally when the robot moves
/// during a sweep because pose is re-read for each packet.
struct Ld06Emitter {
    /// Continuous rotor angle in degrees, kept in [0, 360).
    phase_deg: f32,
    /// Sim time accrued since the last emission, drained `PACKET_PERIOD_US`
    /// at a time. Decouples packet rate from `sim_tick_ms`.
    accumulator_us: u32,
    /// Per-packet timestamp written to the wire (wraps at u16).
    timestamp_ms: u16,
}

impl Default for Ld06Emitter {
    fn default() -> Self {
        Self { phase_deg: 0.0, accumulator_us: 0, timestamp_ms: 0 }
    }
}

impl Ld06Emitter {
    /// ≈ 2 Hz rotor, matching the LD06 captured in galipeur/log_lidar.log
    /// (revolution period ≈ 500 ms).
    const SPEED_DEG_S: u16 = 720;
    /// Packet sampling period. 11_900 µs × 720°/s ≈ 8.568° per packet
    /// → ~42 packets per revolution → ~504 points/rev (firmware caps at
    /// POINTS_PER_REVOLUTION = 500). 360 / 8.568 = 42.018, so packet
    /// boundaries drift across revolutions and `start_angle` is not aligned
    /// to any fixed grid.
    const PACKET_PERIOD_US: u32 = 11_900;
    const MAX_RANGE_MM: f32 = 10_000.0;

    /// Advance time by `dt_ms` and emit every packet whose sampling window
    /// fits inside the elapsed accumulator. Returns 0..=N packets (typically
    /// 0 or 1 at sim_tick_ms = 10 with the current period).
    fn tick(
        &mut self,
        pose: Pose2D,
        walls: &[[f32; 4]],
        dt_ms: u16,
        lidar_offset_deg: f32,
    ) -> Vec<Ld06EmittedPacket> {
        self.accumulator_us = self
            .accumulator_us
            .saturating_add((dt_ms as u32).saturating_mul(1_000));
        let mut out = Vec::new();
        while self.accumulator_us >= Self::PACKET_PERIOD_US {
            self.accumulator_us -= Self::PACKET_PERIOD_US;
            out.push(self.emit_one(pose, walls, lidar_offset_deg));
        }
        out
    }

    fn emit_one(&mut self, pose: Pose2D, walls: &[[f32; 4]], lidar_offset_deg: f32) -> Ld06EmittedPacket {
        let span_deg =
            Self::SPEED_DEG_S as f32 * (Self::PACKET_PERIOD_US as f32) / 1_000_000.0;
        let step = span_deg / 11.0;
        let start = self.phase_deg;
        let end = (start + span_deg).rem_euclid(360.0);

        let mut distances = [0u16; 12];
        let intensities = [200u8; 12];
        for i in 0..12 {
            let raw_deg = start + step * i as f32;
            // Firmware converts raw CW to body CCW: body = (offset - raw).
            // Strat convention: 0° = +Y, CCW.
            let body_rad = (lidar_offset_deg - raw_deg).to_radians();
            let world_angle = pose.theta_rad + body_rad;
            let d = raycast::raycast(
                (pose.x_mm, pose.y_mm),
                world_angle,
                Self::MAX_RANGE_MM,
                walls,
            );
            distances[i] = d.round().clamp(0.0, 65535.0) as u16;
        }

        let bytes = ld06_encoder::encode_packet(
            start,
            end,
            Self::SPEED_DEG_S,
            self.timestamp_ms,
            distances,
            intensities,
        );

        self.phase_deg = (self.phase_deg + span_deg).rem_euclid(360.0);
        self.timestamp_ms = self
            .timestamp_ms
            .wrapping_add((Self::PACKET_PERIOD_US / 1_000) as u16);
        Ld06EmittedPacket {
            bytes,
            start_deg: start,
            end_deg: end,
            distances,
        }
    }
}

/// One LD06 packet ready to be forwarded to the firmware (`bytes`) and
/// optionally re-published to Bevy for live visualization (`distances`
/// + start/end angles in body-frame degrees).
struct Ld06EmittedPacket {
    bytes: [u8; ld06_encoder::PACKET_SIZE],
    start_deg: f32,
    end_deg: f32,
    distances: [u16; 12],
}

fn handle_pami(
    stream: UnixStream,
    config: &Config,
    start: Pose2D,
    sim_tick_ms: u16,
    robot_id: &str,
    updates: &Sender<WorldUpdate>,
    world: &World,
) -> io::Result<()> {
    let (wb, wd, ticks, max_wv) = match config.pami.kinematics.as_ref() {
        Some(KinematicsConfig::Diff {
            wheel_base_mm,
            wheel_diameter_mm,
            encoder_ticks_per_rev,
            max_wheel_speed_mm_s,
        }) => (*wheel_base_mm, *wheel_diameter_mm, *encoder_ticks_per_rev, *max_wheel_speed_mm_s),
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "pami.kinematics missing or not differential",
            ));
        }
    };
    let mut state = DiffState::new(start, wb, wd, ticks, max_wv);
    let dt_s = (sim_tick_ms as f32) / 1000.0;
    let robot_shape = robot_shape_from_config(
        &config.pami.bbox,
        config.pami.model.as_ref().map(|m| m.collision.as_slice()),
    );
    let static_walls = config
        .field
        .obstacle_segments_visible_from(config.pami.lidar_height_mm);
    let mut last_pose_log = Instant::now();

    loop {
        let msg: SimMsgC2S = recv_msg(&stream)?;
        match msg {
            SimMsgC2S::MotorConsignsDiff { values } => {
                let delta = state.step(values, dt_s, &config.field.obstacles, &robot_shape);
                send_msg(&stream, &SimMsgS2C::EncoderDeltaDiff { delta })?;

                // VLX forward raycast, including other entities' bboxes.
                let walls = world.visible_segments(robot_id, config.pami.lidar_height_mm, &static_walls);
                let d = raycast::raycast(
                    (state.pose.x_mm, state.pose.y_mm),
                    state.pose.theta_rad,
                    4_000.0,
                    &walls,
                );
                send_msg(&stream, &SimMsgS2C::VlxDistance { mm: d.round().clamp(0.0, 65535.0) as u16 })?;

                if last_pose_log.elapsed() >= Duration::from_millis(500) {
                    last_pose_log = Instant::now();
                    log::debug!(
                        "[{robot_id}] pose x={:.1} y={:.1} θ={:.3}",
                        state.pose.x_mm, state.pose.y_mm, state.pose.theta_rad,
                    );
                }
                world.update_pose(robot_id, state.pose);
                updates
                    .send(WorldUpdate::UpdatePose { id: robot_id.to_string(), pose: state.pose })
                    .ok();
            }
            SimMsgC2S::MotorsBreak { .. } => {}
            SimMsgC2S::MotorConsignsHolo { .. } => {
                log::warn!("[{robot_id}] ignored MotorConsignsHolo on pami");
            }
            SimMsgC2S::CanFrame { .. } => {} // pami has no CAN
            SimMsgC2S::RomeBytes { .. } => {} // deferred
            SimMsgC2S::Hello { .. } => {
                log::warn!("[{robot_id}] unexpected Hello after handshake");
            }
            SimMsgC2S::Ext { tag, .. } => log::debug!("[{robot_id}] Ext {tag} ignored"),
            SimMsgC2S::NeopixelFrame { .. } => { /* pami has no neopixels */ }
            SimMsgC2S::Teleport { pose } => {
                log::info!(
                    "[{robot_id}] teleport to ({:.1}, {:.1}, θ={:.3})",
                    pose.x_mm, pose.y_mm, pose.theta_rad
                );
                state.pose = pose;
                world.update_pose(robot_id, state.pose);
                updates
                    .send(WorldUpdate::UpdatePose {
                        id: robot_id.to_string(),
                        pose: state.pose,
                    })
                    .ok();
            }
            SimMsgC2S::DebugVolumes { volumes } => {
                updates
                    .send(WorldUpdate::DebugVolumes {
                        id: robot_id.to_string(),
                        volumes,
                    })
                    .ok();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn pose_origin() -> Pose2D {
        Pose2D { x_mm: 0.0, y_mm: 0.0, theta_rad: 0.0 }
    }

    fn decode_start_end(packet: &[u8; ld06_encoder::PACKET_SIZE]) -> (f32, f32) {
        let start = u16::from_le_bytes([packet[4], packet[5]]) as f32 * 0.01;
        let end = u16::from_le_bytes([packet[42], packet[43]]) as f32 * 0.01;
        (start, end)
    }

    /// 1 s of sim time @ 10 ms ticks should produce ~84 packets (period ≈
    /// 11.9 ms), span ≈ 8.568°, two full revolutions, and many distinct
    /// `start_angle` values — the rigid 0/30/60/… grid of the previous
    /// implementation is gone.
    #[test]
    fn emitter_pacing_drifts_and_completes_revolutions() {
        let mut emitter = Ld06Emitter::default();
        let walls: Vec<[f32; 4]> = Vec::new();
        let pose = pose_origin();

        let mut starts = Vec::new();
        let mut spans = Vec::new();
        let mut wraps = 0usize;
        let mut prev_start = 0.0_f32;

        for _ in 0..100 {
            for packet in emitter.tick(pose, &walls, 10) {
                let (start, end) = decode_start_end(&packet.bytes);
                let span = (end - start + 360.0).rem_euclid(360.0);
                if !starts.is_empty() && start < prev_start - 180.0 {
                    wraps += 1;
                }
                spans.push(span);
                starts.push(start);
                prev_start = start;
            }
        }

        assert!(
            (80..=90).contains(&starts.len()),
            "packet count = {}, expected ~84",
            starts.len()
        );

        let unique: HashSet<u32> = starts.iter().map(|s| (s * 100.0) as u32).collect();
        assert!(
            unique.len() >= 30,
            "only {} distinct start_angles over {} packets — drift not happening",
            unique.len(),
            starts.len()
        );

        let mean_span = spans.iter().sum::<f32>() / spans.len() as f32;
        assert!(
            (mean_span - 8.568).abs() < 0.1,
            "mean span = {mean_span}, expected ≈ 8.568°"
        );

        assert!(
            wraps >= 1,
            "no full revolution observed in 1 s (wraps={wraps})"
        );
    }
}
