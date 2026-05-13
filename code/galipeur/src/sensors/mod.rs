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
use crate::opponent_detection::OpponentDetection;
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
    pub fn new(board: &mut B, can: GalipeurCan<B>, led_sender: Sender<LedMessage>, top_lidar_conf: TopLidarConf, opponent_detection: OpponentDetection) -> Self {
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
                    //log::info!("LidarStatus: module: {} distance_0: {} sq_0: {} distance_1: {} sq_1: {}", module, distance_0, sq_0, distance_1, sq_1);
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
                priority: 10,
                pin_to_core: Some(Core::Core1),
                ..Default::default()
            }
            .set()
            .unwrap();
        }
            std::thread::sleep(Duration::from_millis(1000));

       //    can.send(&CanMessage::SetLidarEnable { enable: true });
       //loop {
       //    can.send(&CanMessage::RequestGroundValue { sensor: 0 });
       //    can.send(&CanMessage::RequestGroundValue { sensor: 1 });
       //    can.send(&CanMessage::RequestGroundValue { sensor: 2 });
       //    std::thread::sleep(Duration::from_millis(100));
       //}

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
                            // Feed opponent detection on every packet (~40Hz)
                            let chunk = ld06::packet_points(&packet, top_lidar_conf.angle_offset);
                            opponent_detection.feed(&chunk);

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

    /// power off ground lidar data for all 3 modules
    pub fn ground_lidar_power_off(&self) {
        self.can.send(&CanMessage::SetLidarEnable { enable: false });
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

    /// Wait for a fresh ground lidar reading on `face`.
    ///
    /// Blocks until a new CAN LidarStatus arrives for the requested module.
    pub fn ground_lidar_wait(&self, face: RobotSide) -> Option<GroundLidarModule> {
        let idx = face.module() as usize;
        self.ground_lidar_modules[idx].wait_next()
    }

    /// Get the calibrated poses for the two lidars on `face`.
    pub fn ground_lidar_poses(&self, face: RobotSide) -> Option<[GroundLidarPose; 2]> {
        let idx = face.module() as usize;
        Some(self.ground_lidar_conf.get()?.modules[idx])
    }
}
