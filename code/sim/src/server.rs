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
    config: Config,
    updates: Sender<WorldUpdate>,
    world: World,
    registry: ConnRegistry,
) -> io::Result<()> {
    if Path::new(socket_path).exists() {
        std::fs::remove_file(socket_path).ok();
    }
    let listener = UnixListener::bind(socket_path)?;
    log::info!("sim listening on {socket_path}");

    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                let cfg = config.clone();
                let updates = updates.clone();
                let world = world.clone();
                let registry = registry.clone();
                thread::Builder::new()
                    .name("sim-conn".into())
                    .spawn(move || {
                        if let Err(e) = handle_connection(s, cfg, updates, world, registry) {
                            log::error!("connection ended: {e}");
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
) -> io::Result<()> {
    let hello: SimMsgC2S = recv_msg(&stream)?;
    let (robot_id, kind, requested_start) = match hello {
        SimMsgC2S::Hello { version, robot_id, kind, requested_start, .. } => {
            if version != sim_protocol::PROTOCOL_VERSION {
                log::warn!(
                    "protocol version mismatch: robot {version}, sim {}",
                    sim_protocol::PROTOCOL_VERSION
                );
            }
            (robot_id, kind, requested_start)
        }
        other => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("expected Hello, got {:?}", other),
            ));
        }
    };

    let start = requested_start.unwrap_or_else(|| config.default_start_for(kind));
    let sim_tick_ms: u16 = 10;
    log::info!("robot '{robot_id}' ({:?}) starting at {:?}", kind, start);

    send_msg(&stream, &SimMsgS2C::HelloAck { assigned_start: start, sim_tick_ms })?;
    let robot_cfg = match kind {
        RobotKind::Galipeur => &config.galipeur,
        RobotKind::Pami => &config.pami,
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
        Some(KinematicsConfig::Holo { velocities_to_consigns, encoders_to_position }) => {
            (*velocities_to_consigns, *encoders_to_position)
        }
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
    let mut lidar = Ld06Emitter::default();
    let mut emu = PicotterEmu::default();
    let mut last_battery = Instant::now();
    let mut last_pose_log = Instant::now();
    let battery_period = Duration::from_secs(1);
    let pose_log_period = Duration::from_millis(500);
    let mut can_frames_in: u64 = 0;
    let mut can_frames_out: u64 = 0;

    // The asserv starts at (0,0,0) inside the robot; it's the robot's
    // responsibility to call `reset_position(start)` after it reads its
    // assigned start pose from the sim (see galipeur/src/routines.rs for
    // the non-espidf cfg).
    loop {
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

                // Emit one LD06 packet per tick (~30° of the 360° sweep).
                let walls = world.visible_segments(robot_id, config.galipeur.lidar_height_mm, &static_walls);
                let packet = lidar.emit_packet(state.pose, &walls, sim_tick_ms);
                send_msg(&stream, &SimMsgS2C::Ld06Bytes { bytes: packet.to_vec() })?;

                if last_battery.elapsed() >= battery_period {
                    last_battery = Instant::now();
                    send_msg(&stream, &PicotterEmu::battery_status_msg())?;
                    // Also emit ground status (all three sensors happy)
                    // so the galipeur doesn't stay in its "not on
                    // ground" init state forever.
                    send_msg(&stream, &PicotterEmu::ground_status_msg())?;
                    can_frames_out += 2;
                }

                if last_pose_log.elapsed() >= pose_log_period {
                    last_pose_log = Instant::now();
                    log::info!(
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
        }
    }
}

/// Rotates around the robot at ~300 deg/s (≈ one revolution every 1.2 s,
/// close to the real LD06). Emits one 30°-wide packet per 10 ms sim tick.
struct Ld06Emitter {
    current_angle_deg: f32,
    timestamp_ms: u16,
}

impl Default for Ld06Emitter {
    fn default() -> Self {
        Self { current_angle_deg: 0.0, timestamp_ms: 0 }
    }
}

impl Ld06Emitter {
    const PACKET_ANGLE_SPAN: f32 = 30.0;
    const SPEED_DEG_S: u16 = 3000;
    const MAX_RANGE_MM: f32 = 10_000.0;

    fn emit_packet(&mut self, pose: Pose2D, walls: &[[f32; 4]], dt_ms: u16) -> [u8; 47] {
        let start = self.current_angle_deg;
        let end = (start + Self::PACKET_ANGLE_SPAN) % 360.0;
        let step = Self::PACKET_ANGLE_SPAN / 11.0;

        let mut distances = [0u16; 12];
        let intensities = [200u8; 12];
        for i in 0..12 {
            let local_deg = start + step * i as f32;
            let world_angle = pose.theta_rad + local_deg.to_radians();
            let d = raycast::raycast(
                (pose.x_mm, pose.y_mm),
                world_angle,
                Self::MAX_RANGE_MM,
                walls,
            );
            distances[i] = d.round().clamp(0.0, 65535.0) as u16;
        }

        let packet = ld06_encoder::encode_packet(
            start,
            end,
            Self::SPEED_DEG_S,
            self.timestamp_ms,
            distances,
            intensities,
        );

        self.current_angle_deg = end;
        self.timestamp_ms = self.timestamp_ms.wrapping_add(dt_ms);
        packet
    }
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
                    log::info!(
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
        }
    }
}
