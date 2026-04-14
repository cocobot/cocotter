use std::sync::{Arc, OnceLock};
use std::time::Duration;
use board_sabotter::SabotterBoard;
use cancaner::CanMessage;
use flume::Sender;
use board_sabotter::SabotterAdc;

use asserv::holonomic::RobotSide;
use crate::can::GalipeurCan;
use crate::led::LedMessage;
use crate::meca::RobotSideModule;
use crate::watched::Watched;

const BATTERY_LOW_MV: f32 = 14_830.0; // 4S LiPo discharged threshold
const NUM_MODULES: usize = 3;
const NUM_GROUND_SENSORS: usize = 3;

/// Position and orientation of one lidar in the robot frame
#[derive(Debug, Clone, Copy)]
pub struct LidarPose {
    pub x: f32,     // mm
    pub y: f32,     // mm
    pub theta: f32, // radians, direction the lidar points
}

/// Ground sensor threshold configuration
pub struct GroundConf {
    pub thresholds: [u16; NUM_GROUND_SENSORS],
}

/// Configuration for all lidar modules (2 poses per module)
pub struct LidarConf {
    pub modules: [[LidarPose; 2]; NUM_MODULES],
}

/// Result of plane offset computation from a lidar module
#[derive(Debug, Clone, Copy)]
pub struct PlaneOffset {
    /// Normal direction of the detected surface in robot frame (radians)
    pub angle: f32,
    /// Perpendicular distance from robot center to the surface (mm)
    pub distance: f32,
}

/// Lidar data for one module (2 lidars)
#[derive(Debug, Clone, Copy, Default)]
pub struct ModuleLidar {
    pub distance_0: u16,
    pub sq_0: u16,
    pub distance_1: u16,
    pub sq_1: u16,
}

#[derive(Clone)]
pub struct Sensors<B: SabotterBoard> {
    can: GalipeurCan<B>,
    lidar_modules: [Watched<ModuleLidar>; NUM_MODULES],
    lidar_conf: Arc<OnceLock<LidarConf>>,
}

impl<B: SabotterBoard + 'static> Sensors<B> {
    pub fn new(board: &mut B, can: GalipeurCan<B>, led_sender: Sender<LedMessage>) -> Self {
        let lidar_modules: [Watched<ModuleLidar>; NUM_MODULES] = [
            Watched::default(),
            Watched::default(),
            Watched::default(),
        ];

        let battery_led_sender = led_sender.clone();
        let lidar_cb = lidar_modules.clone();
        can.add_callback(move |msg| {
            match msg {
                CanMessage::BatteryStatus { voltage_mv, modules_mask: _ } => {
                    if *voltage_mv < (BATTERY_LOW_MV as u16) {
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
                    log::info!("Ground mask {}", detection_mask);
                }
                CanMessage::LidarStatus { module, distance_0, sq_0, distance_1, sq_1 } => {
                    let idx = *module as usize;
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

        if let Some(mut battery_adc) = board.battery_adc() {
            std::thread::spawn(move || {
                const VBATT_RL_KOHMS: f32 = 6.8;
                const VBATT_RH_KOHMS: f32 = 100.0;
                const ADC_INPUT_IMP_KOHMS: f32 = 35.5; // measured empirically (DB_12 attenuation)
                const RL_EFF_KOHMS: f32 = VBATT_RL_KOHMS * ADC_INPUT_IMP_KOHMS / (VBATT_RL_KOHMS + ADC_INPUT_IMP_KOHMS);
                loop {
                    if let Ok(raw) = battery_adc.read() {
                        let mv = raw as f32 * (1.0 + VBATT_RH_KOHMS / RL_EFF_KOHMS);
                        if mv < BATTERY_LOW_MV {
                            log::warn!("Battery low! {:.0} mV", mv);
                            led_sender.send(LedMessage::LowLogicBattery).ok();
                        }
                    }
                    std::thread::sleep(Duration::from_secs(1));
                }
            });
        }

        Self {
            can,
            lidar_modules,
            lidar_conf: Arc::new(OnceLock::new()),
        }
    }

    /// Set sensor configuration (can only be called once)
    pub fn set_conf(&self, lidar_conf: LidarConf, ground_conf: GroundConf) {
        let _ = self.lidar_conf.set(lidar_conf);
        for (sensor, &threshold) in ground_conf.thresholds.iter().enumerate() {
            self.can.send(&CanMessage::SetGroundThreshold {
                sensor: sensor as u8,
                threshold,
            });
        }
    }

    /// Get raw lidar data for a robot side (last cached value)
    pub fn lidar(&self, side: RobotSide) -> ModuleLidar {
        self.lidar_modules[side.module() as usize].get()
    }

    /// Wait for a fresh lidar measurement on this side, then compute plane offset.
    ///
    /// Blocks until a new CAN LidarStatus arrives for the requested module.
    /// Returns `None` if measurements are invalid or config is not set.
    pub fn get_plane_offset(&self, side: RobotSide) -> Option<PlaneOffset> {
        let idx = side.module() as usize;
        let module = self.lidar_modules[idx].wait_next()?;

        // Need valid measurements from both lidars
        if module.distance_0 == 0 || module.distance_1 == 0 {
            return None;
        }

        let conf = self.lidar_conf.get()?.modules[idx];

        let pose0 = conf[0];
        let pose1 = conf[1];
        let d0 = module.distance_0 as f32;
        let d1 = module.distance_1 as f32;

        // Hit points in robot frame
        let p0x = pose0.x + d0 * pose0.theta.cos();
        let p0y = pose0.y + d0 * pose0.theta.sin();
        let p1x = pose1.x + d1 * pose1.theta.cos();
        let p1y = pose1.y + d1 * pose1.theta.sin();

        // Direction along the surface
        let dx = p1x - p0x;
        let dy = p1y - p0y;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-6 {
            return None;
        }

        // Outward normal (perpendicular to surface, pointing away from robot)
        let mut nx = -dy / len;
        let mut ny = dx / len;

        // Perpendicular distance from origin to the line
        let mut dist = nx * p0x + ny * p0y;

        // Ensure distance is positive (normal points outward)
        if dist < 0.0 {
            nx = -nx;
            ny = -ny;
            dist = -dist;
        }

        Some(PlaneOffset {
            angle: ny.atan2(nx),
            distance: dist,
        })
    }
}
