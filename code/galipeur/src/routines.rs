use std::time::{Duration, Instant};
use asserv::holonomic::{Asserv, rome::AsservHoloRome};
use asserv::rome::AsservRome;
use board_common::Periodicity;
use board_sabotter::SabotterBoard;
use cancaner::CanMessage;
use flume::{Receiver, Sender};
use sch16t::Sch16t;
use crate::led::{LedMessage, Leds};
use crate::movement::MovementLowLevelHardware;
use crate::meca::Meca;
use crate::can::{GalipeurCan, ota_relay::CanOtaRelayHandler};
use crate::strat::Strat;
use crate::sensors::{Sensors, TopLidarConf};

/// Everything needed for PAMI routines
///
/// Update state from multiple peripherals.
/// States are updated when calling `idle()` or `step_idle()`.
pub struct GalipeurRoutines<B: SabotterBoard> {
    pub asserv: Asserv<MovementLowLevelHardware<B>>,
    pub meca: Meca<B>,

    // ROME sender/receiver
    pub rome_tx: Sender<Box<[u8]>>,
    pub rome_rx: Receiver<Box<[u8]>>,

    //leds
    led_sender: Sender<LedMessage>,

    //can interface
    can: GalipeurCan<B>,

    // Sensors
    pub sensors: Sensors<B>,

    // Lidar telemetry flags
    lidar_tm_ground: bool,
    lidar_tm_top: bool,

    // Periodicity states
    asserv_periodicity: Periodicity,
    asserv_tm_periodicity: Periodicity,
    meca_periodicity: Periodicity,
    meca_tm_periodicity: Periodicity,
    lidar_tm_periodicity: Periodicity,
}

impl<B: SabotterBoard + 'static> GalipeurRoutines<B> {
    /// Initialize with default state values and peripherals from board
    ///
    /// Peripherals must be available on the board.
    /// The asserv must be configured manually, using `asserv.set_conf()`.
    pub fn new(
        board: &mut B,
        top_lidar_conf: TopLidarConf,
    ) -> Self {
        // Setup gyro, asserv
        let mut gyro = Sch16t::new(board.imu_spi().unwrap(), 0);
        gyro.init().unwrap();
        let asserv_hardware = MovementLowLevelHardware::new(gyro, board.motors().unwrap());

        // Setup CAN interface
        let can_interface = GalipeurCan::new(board.can().unwrap());

        // Setup Rome
        let picotter_ota = CanOtaRelayHandler::new(can_interface.clone());
        let (rome_tx, rome_rx) = board.rome("Galipeur".into(), vec![Box::new(picotter_ota)]).unwrap();


        // Setup Led feedback
        let leds = Leds::new::<B>(board);
        let led_sender = leds.sender();

        // Setup meca
        let meca = Meca::new(can_interface.clone());

        // Setup sensors
        let sensors = Sensors::new(board, can_interface.clone(), led_sender.clone(), top_lidar_conf);

        // Setup strat
        Strat::init(board, led_sender.clone(), sensors.clone());

        Self {
            asserv: Asserv::new(asserv_hardware),
            meca,

            rome_tx,
            rome_rx,

            led_sender,

            can: can_interface,

            sensors,

            lidar_tm_ground: false,
            lidar_tm_top: false,

            asserv_periodicity: Periodicity::new(Duration::from_millis(10)),
            asserv_tm_periodicity: Periodicity::new(Duration::from_millis(500)),
            meca_periodicity: Periodicity::new(Duration::from_millis(1000)),
            meca_tm_periodicity: Periodicity::new(Duration::from_millis(1000)),
            lidar_tm_periodicity: Periodicity::new(Duration::from_millis(2000)),
        }
    }

    /// Intialize states and peripherals
    pub fn init(&mut self) {
    }

    #[allow(dead_code)]
    pub fn ground_sensor_calibration(self) -> ! {
        loop {
            self.can.send(&CanMessage::RequestGroundValue { sensor: 0 });
            self.can.send(&CanMessage::RequestGroundValue { sensor: 1 });
            self.can.send(&CanMessage::RequestGroundValue { sensor: 2 });
            std::thread::sleep(Duration::from_millis(100))
        }
    }

    /// Execute one round of idle updates
    ///
    /// This method must be called regularly.
    pub fn idle(&mut self, now: &Instant) {
        let t0 = Instant::now();

        // Process ROME input messages
        let rome_messages: Vec<_> = self.rome_rx.try_iter().collect();
        if !rome_messages.is_empty() {
            self.led_sender.send(LedMessage::RomeActivity).ok();
        }
        for data in rome_messages {
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !self.on_rome_message(&message) && !self.asserv.on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }
//
        // Update asserv, send asserv telemetry
        if self.asserv_periodicity.update(now) {
            self.asserv.update();
        }
        if self.asserv_tm_periodicity.update(now) {
            if let Err(err) = self.rome_tx.send(self.asserv.asserv_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Err(err) = self.rome_tx.send(self.asserv.asserv_holo_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Some(message) = self.asserv.asserv_holo_tm_path() {
                if let Err(err) = self.rome_tx.send(message.encode()) {
                    log::error!("ROME send error: {:?}", err);
                }
            }
        }
//
        //// Send lidar telemetry
        if self.lidar_tm_periodicity.update(now) {
            if self.lidar_tm_ground {
                let modules = self.sensors.ground_lidar_all();
                let _ = self.rome_tx.send(rome::Message::GroundLidarTm {
                    d0_0: modules[0].distance_0,
                    sq0_0: modules[0].sq_0,
                    d0_1: modules[0].distance_1,
                    sq0_1: modules[0].sq_1,
                    d1_0: modules[1].distance_0,
                    sq1_0: modules[1].sq_0,
                    d1_1: modules[1].distance_1,
                    sq1_1: modules[1].sq_1,
                    d2_0: modules[2].distance_0,
                    sq2_0: modules[2].sq_0,
                    d2_1: modules[2].distance_1,
                    sq2_1: modules[2].sq_1,
                }.encode());
            }
            if self.lidar_tm_top {
                let scan = self.sensors.top_lidar_scan();
                let chunk_size = 80;
                let num_chunks = (scan.points.len() + chunk_size - 1) / chunk_size;
                for (i, chunk) in scan.points.chunks(chunk_size).enumerate() {
                    let mut angles = [0u16; 80];
                    let mut distances = [0u16; 80];
                    let mut intensities = [0u8; 80];
                    for (j, &(angle, distance, intensity)) in chunk.iter().enumerate() {
                        angles[j] = (angle * 100.0) as u16;
                        distances[j] = distance;
                        intensities[j] = intensity;
                    }
                    let _ = self.rome_tx.send(rome::Message::TopLidarTm {
                        chunk_index: i as u8,
                        num_chunks: num_chunks as u8,
                        count: chunk.len() as u8,
                        angles,
                        distances,
                        intensities,
                    }.encode());
                }
            }
        }
        //// Update meca, send meca telemetry
        //if self.meca_periodicity.update(now) {
        //    let meca_state = self.meca.get_state();
        //    for (i_module, module) in meca_state.arms.iter().enumerate() {
        //        for (i_arm, arm) in module.iter().enumerate() {
        //            let _ = self.rome_tx.send(rome::Message::MecaArmTmState {
        //                module: i_module as u8,
        //                arm: i_arm as u8,
        //                position: arm.position,
        //                //TODO: update rome with full telemetry
        //                color: rome::params::MecaArmTmStateColor::Unknown,
        //                pump: arm.pump,
        //                valve: arm.valve,
        //                servo_error: arm.error,
        //                torque_enabled: arm.flags.torque_enabled,
        //                moving: arm.flags.moving,
        //                // Note: position_reached == !moving
        //                pump_current: arm.pump_current,
        //            }.encode());
        //        }
        //    }
        //    if self.meca_tm_periodicity.update(now) {
        //        for (i_tr, translation) in meca_state.translations.iter().enumerate() {
        //            let _ = self.rome_tx.send(rome::Message::MecaArmTmTranslation {
        //                module: i_tr as u8,
        //                position: translation.position,
        //                error: translation.error,
        //            }.encode());
        //        }
        //    }
        //}
        let total = t0.elapsed();
        if total > Duration::from_millis(8) {
            self.led_sender.send(LedMessage::IdleLoopTooSlow).ok();
            log::warn!("idle too slow: {:?}", total);
        }
    }

    fn on_rome_message(&mut self, message: &rome::Message) -> bool {
        match *message {
            rome::Message::LidarTmActivate { ground, top } => {
                log::info!("ROME: LidarTmActivate ground={ground} top={top}");
                self.lidar_tm_ground = ground;
                self.lidar_tm_top = top;
                
                if ground {
                    //non blocking call to enable ground lidar if previously deactivated
                    self.sensors.ground_lidar(asserv::holonomic::RobotSide::Back);
                }
            }
            _ => return false,
        }
        true
    }

    /// Wait for the next asserv step, the run `idle()` and return the associated instant
    pub fn step_idle(&mut self) -> Instant {
        let mut now = Instant::now();
        let next_instant = self.asserv_periodicity.next();
        if let Some(duration) = next_instant.checked_duration_since(now) {
            std::thread::sleep(duration);
            now = *next_instant;
        }
        self.idle(&now);

        now
    }
}

