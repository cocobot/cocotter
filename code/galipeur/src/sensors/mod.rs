mod ld06;

pub use ld06::TopLidarSnapshot;

use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};
use board_sabotter::{BatteryLevel, BatteryReader, SabotterBoard, SabotterUart};
use cancaner::CanMessage;
use flume::Sender;

use asserv::holonomic::RobotSide;
use crate::can::GalipeurCan;
use crate::led::LedMessage;
use crate::meca::RobotSideModule;
use crate::watched::Watched;

const BATTERY_LOW_MV: u16 = 14_830; // 4S LiPo discharged threshold
const NUM_MODULES: usize = 3;
const NUM_GROUND_SENSORS: usize = 3;

/// Position and orientation of one ground lidar in the robot frame
#[derive(Debug, Clone, Copy)]
pub struct GroundLidarPose {
    pub x: f32,     // mm
    pub y: f32,     // mm
    pub theta: f32, // radians, direction the lidar points
}

/// Ground sensor threshold configuration
pub struct GroundConf {
    pub thresholds: [u16; NUM_GROUND_SENSORS],
}

/// Configuration for all ground lidar modules (2 poses per module)
pub struct GroundLidarConf {
    pub modules: [[GroundLidarPose; 2]; NUM_MODULES],
}

/// Configuration for the 360 top lidar (LD06)
pub struct TopLidarConf {
    /// Angle offset in degrees between lidar's angle 0 and robot's angle 0
    pub angle_offset: f32,
}

/// Result of plane offset computation from a ground lidar module
#[derive(Debug, Clone, Copy)]
pub struct PlaneOffset {
    /// Angular offset of wall vs expected face normal (radians). None if geometry is degenerate.
    pub angle: Option<f32>,
    /// Perpendicular distance from robot center to the surface (mm)
    pub distance: f32,
}

/// Ground lidar data for one module (2 lidars)
#[derive(Debug, Clone, Copy, Default)]
pub struct GroundLidarModule {
    pub distance_0: u16,
    pub sq_0: u16,
    pub distance_1: u16,
    pub sq_1: u16,
}

pub struct Sensors<B: SabotterBoard> {
    can: GalipeurCan<B>,
    ground_lidar_modules: [Watched<GroundLidarModule>; NUM_MODULES],
    ground_lidar_conf: Arc<OnceLock<GroundLidarConf>>,
    top_lidar: Watched<TopLidarSnapshot>,
}

impl<B: SabotterBoard> Clone for Sensors<B> {
    fn clone(&self) -> Self {
        Self {
            can: self.can.clone(),
            ground_lidar_modules: self.ground_lidar_modules.clone(),
            ground_lidar_conf: self.ground_lidar_conf.clone(),
            top_lidar: self.top_lidar.clone(),
        }
    }
}

impl<B: SabotterBoard + 'static> Sensors<B> {
    pub fn new(board: &mut B, can: GalipeurCan<B>, led_sender: Sender<LedMessage>, top_lidar_conf: TopLidarConf) -> Self {
        let ground_lidar_modules: [Watched<GroundLidarModule>; NUM_MODULES] = [
            Watched::default(),
            Watched::default(),
            Watched::default(),
        ];

        let battery_led_sender = led_sender.clone();
        let lidar_cb = ground_lidar_modules.clone();
        can.add_callback(move |msg| {
            match msg {
                CanMessage::BatteryStatus { voltage_mv, modules_mask: _ } => {
                    if *voltage_mv < BATTERY_LOW_MV {
                        battery_led_sender.send(LedMessage::LowPowerBattery).ok();
                    }
                }
                CanMessage::GroundValue { sensor, value, threshold } => {
                    //only used in debug when requested by sabotter for calibration
                    log::info!("GroundValue: sensor: {} value: {}/{}", sensor, value, threshold);
                }
                CanMessage::GroundStatus { detection_mask } => {
                    battery_led_sender.send(LedMessage::GroundSensor(
                        detection_mask & 0b001 != 0,
                        detection_mask & 0b010 != 0,
                        detection_mask & 0b100 != 0)
                    ).ok();
                }
                CanMessage::LidarStatus { module, distance_0, sq_0, distance_1, sq_1 } => {
                    let idx = *module as usize;
                    log::info!("LidarStatus: module: {} distance_0: {} sq_0: {} distance_1: {} sq_1: {}", module, distance_0, sq_0, distance_1, sq_1);
                    if idx < NUM_MODULES {
                        lidar_cb[idx].update(|m| {
                            m.distance_0 = *distance_0;
                            m.sq_0 = *sq_0;
                            m.distance_1 = *distance_1;
                            m.sq_1 = *sq_1;
                        });
                    }
                }
                _ => {}
            }
        });

        let top_lidar: Watched<TopLidarSnapshot> = Watched::default();
        let top_lidar_thread = top_lidar.clone();
        let mut battery_reader = board.battery_reader().unwrap();
        let uart = board.lidar_uart().unwrap();

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            use esp_idf_svc::hal::cpu::Core;
            ThreadSpawnConfiguration {
                pin_to_core: Some(Core::Core1),
                ..Default::default()
            }
            .set()
            .unwrap();
        }

        std::thread::Builder::new()
            .name("sensors".into())
            .stack_size(16384)
            .spawn(move || {
                let mut check_battery = || {
                    let BatteryLevel { mv, .. } = battery_reader.read_vbatt();
                    if mv < BATTERY_LOW_MV {
                        log::warn!("Battery low! {:.0} mV", mv);
                        led_sender.send(LedMessage::LowLogicBattery).ok();
                    }
                };

                let mut ld06_scan = ld06::TopLidarScan::new(top_lidar_conf.angle_offset);
                let mut buffer = [0u8; ld06::PACKET_SIZE];
                let mut offset = 0usize;
                let mut last_battery_check = Instant::now();

                loop {
                    // Periodic battery check
                    if last_battery_check.elapsed() >= Duration::from_secs(1) {
                        last_battery_check = Instant::now();
                        check_battery();
                    }

                    // Fill buffer
                    match uart.read(&mut buffer[offset..]) {
                        Ok(n) if n > 0 => offset += n,
                        _ => {
                            // Timeout or error: check battery and retry
                            if last_battery_check.elapsed() >= Duration::from_secs(1) {
                                last_battery_check = Instant::now();
                                check_battery();
                            }
                            continue;
                        }
                    }

                    if offset < ld06::PACKET_SIZE {
                        continue;
                    }

                    // Full buffer: check sync
                    if buffer[0] == 0x54 && ld06::verify_crc(&buffer) {
                        if let Some(packet) = ld06::parse_packet(&buffer) {
                            if let Some(snapshot) = ld06_scan.process_packet(&packet) {                                
                                top_lidar_thread.update(|s| *s = snapshot);
                            }
                        }
                        offset = 0;
                    } else {
                        // Not synced: find next 0x54 in buffer and shift
                        let skip = buffer[1..].iter().position(|&b| b == 0x54)
                            .map(|p| p + 1)
                            .unwrap_or(ld06::PACKET_SIZE);
                        let remaining = ld06::PACKET_SIZE - skip;
                        buffer.copy_within(skip.., 0);
                        offset = remaining;
                    }
                }
            })
            .expect("spawn sensors");

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            use esp_idf_svc::hal::cpu::Core;
            ThreadSpawnConfiguration {
                pin_to_core: Some(Core::Core0),
                ..Default::default()
            }
            .set()
            .unwrap();
        }

        Self {
            can,
            ground_lidar_modules,
            ground_lidar_conf: Arc::new(OnceLock::new()),
            top_lidar,
        }
    }

    /// Set sensor configuration (can only be called once)
    pub fn set_conf(&self, ground_lidar_conf: GroundLidarConf, ground_conf: GroundConf) {
        let _ = self.ground_lidar_conf.set(ground_lidar_conf);
        for (sensor, &threshold) in ground_conf.thresholds.iter().enumerate() {
            self.can.send(&CanMessage::SetGroundThreshold {
                sensor: sensor as u8,
                threshold,
            });
        }
    }

    /// Get raw ground lidar data for a robot side (last cached value)
    pub fn ground_lidar(&self, side: RobotSide) -> GroundLidarModule {
        self.can.send(&CanMessage::SetLidarEnable { enable: true });
        self.ground_lidar_modules[side.module() as usize].get()
    }

    /// Get raw ground lidar data for all 3 modules
    pub fn ground_lidar_all(&self) -> [GroundLidarModule; NUM_MODULES] {
        [
            self.ground_lidar_modules[0].get(),
            self.ground_lidar_modules[1].get(),
            self.ground_lidar_modules[2].get(),
        ]
    }

    /// Get the latest top lidar scan snapshot
    pub fn top_lidar_scan(&self) -> TopLidarSnapshot {
        self.top_lidar.get_clone()
    }

    /// Wait for the next top lidar revolution, return the scan snapshot
    pub fn wait_top_lidar_scan(&self) -> Option<TopLidarSnapshot> {
        self.top_lidar.wait_next_clone()
    }

    /// Wait for a fresh ground lidar measurement on this side, then compute plane offset.
    ///
    /// Blocks until a new CAN LidarStatus arrives for the requested module.
    /// Returns `None` if measurements are invalid or config is not set.
    pub fn get_plane_offset(&self, side: RobotSide) -> Option<PlaneOffset> {
        let idx = side.module() as usize;
        let module = self.ground_lidar_modules[idx].wait_next()?;

        // Need valid measurements from both lidars
        if module.distance_0 == 0 || module.distance_1 == 0 {
            return None;
        }

        log::info!("GroundLidarModule for {:?}: distance_0={} sq_0={} distance_1={} sq_1={}",
            side, module.distance_0, module.sq_0, module.distance_1, module.sq_1);

        let conf = self.ground_lidar_conf.get()?.modules[idx];

        let pose0 = conf[0];
        let pose1 = conf[1];
        let r0 = module.distance_0 as f32;
        let r1 = module.distance_1 as f32;

        // Expected outward normal direction for this face (in robot frame)
        let phi = match side {
            RobotSide::Left  => core::f32::consts::FRAC_PI_3,    //  +60°
            RobotSide::Right => -core::f32::consts::FRAC_PI_3,   //  -60°
            RobotSide::Back  => core::f32::consts::PI,           //  180°
        };
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();

        // For each lidar, compute:
        //   a_i = perpendicular distance from robot center to wall through beam i
        //       = r_i * cos(θ_i - φ) + x_i * cos(φ) + y_i * sin(φ)
        //   b_i = tangential position of hit point (sensitivity to angular offset δ)
        //       = r_i * sin(θ_i - φ) - x_i * sin(φ) + y_i * cos(φ)
        //
        // The wall angle δ (relative to expected face normal) satisfies:
        //   δ ≈ (a_0 - a_1) / (b_1 - b_0)  when |b_1 - b_0| >> |a_0 - a_1|
        // The perpendicular distance:
        //   D ≈ (a_0 + a_1) / 2

        let a0 = r0 * (pose0.theta - phi).cos() + pose0.x * cos_phi + pose0.y * sin_phi;
        let b0 = r0 * (pose0.theta - phi).sin() - pose0.x * sin_phi + pose0.y * cos_phi;
        let a1 = r1 * (pose1.theta - phi).cos() + pose1.x * cos_phi + pose1.y * sin_phi;
        let b1 = r1 * (pose1.theta - phi).sin() - pose1.x * sin_phi + pose1.y * cos_phi;

        let distance = (a0 + a1) / 2.0;
        if distance < 0.0 {
            return None;
        }

        // Angular offset: only trust it if tangential baseline is large enough
        let baseline = (b1 - b0).abs();
        let angle = if baseline > 20.0 {
            Some((a0 - a1) / (b1 - b0))
        } else {
            log::warn!("get_plane_offset({side:?}): degenerate geometry (baseline={baseline:.1}mm)");
            None
        };

        Some(PlaneOffset { angle, distance })
    }
}
