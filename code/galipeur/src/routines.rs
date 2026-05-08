use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use asserv::holonomic::{Asserv, RobotSide, TableSide, TrajectoryEvent, rome::AsservHoloRome};
use asserv::rome::AsservRome;
use board_common::{Periodicity, Team};
use board_sabotter::SabotterBoard;
use cancaner::{CanMessage, ClampServo};
use flume::{Receiver, Sender};
use sch16t::Sch16t;
use crate::led::{LedMessage, Leds};
use crate::movement::MovementLowLevelHardware;
use crate::meca::{CleatSide, Meca};
use crate::can::{GalipeurCan, ota_relay::CanOtaRelayHandler};
use crate::opponent_detection::{OpponentDetection, OpponentDetectionConf, Zone};
use crate::strat::Strat;
use crate::sensors::{Sensors, TopLidarConf};
use crate::strat::utils::arfast;

/// Everything needed for PAMI routines
///
/// Update state from multiple peripherals.
/// States are updated when calling `idle()` or `step_idle()`.
pub struct GalipeurRoutines<B: SabotterBoard> {
    pub asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>,
    pub opponent_detection: OpponentDetection,
    pub meca: Meca<B>,

    // ROME sender/receiver
    pub rome_tx: Sender<Box<[u8]>>,
    pub rome_rx: Receiver<Box<[u8]>>,
    pub rlogger: Sender<String>,

    //leds
    led_sender: Sender<LedMessage>,

    //can interface
    can: GalipeurCan<B>,

    // Sensors
    pub sensors: Sensors<B>,

    // Flags to eneable or disable telemetry
    meca_tm_full: bool,

    // Periodicity states
    asserv_tm_periodicity: Periodicity,
    meca_tm_periodicity: Periodicity,
}

impl<B: SabotterBoard + 'static> GalipeurRoutines<B> {
    /// Initialize with default state values and peripherals from board
    ///
    /// Peripherals must be available on the board.
    /// The asserv must be configured manually, using `asserv.set_conf()`.
    pub fn new(
        board: &mut B,
        top_lidar_conf: TopLidarConf,
        opponent_detection_conf: OpponentDetectionConf,
    ) -> Self {
        // Setup gyro, asserv
        let mut gyro = Sch16t::new(board.imu_spi().unwrap(), 0);
        gyro.init().unwrap();
        let asserv_hardware = MovementLowLevelHardware::new(gyro, board.motors().unwrap());

        // Setup CAN interface
        let can_interface = GalipeurCan::new(board.can().unwrap());

        // Setup Rome
        let picotter_ota = CanOtaRelayHandler::new(can_interface.clone());
        let (rome_tx, rlogger, rome_rx) = board.rome("Galipeur".into(), vec![Box::new(picotter_ota)]).unwrap();
        rome::info!(rlogger, "ROME initialized");

        // Setup Led feedback
        let leds = Leds::new::<B>(board);
        let led_sender = leds.sender();

        // Setup meca
        let meca = Meca::new(can_interface.clone(), led_sender.clone());

        // Setup opponent detection
        let opponent_detection = OpponentDetection::new(opponent_detection_conf, led_sender.clone());

        // Setup sensors
        let sensors = Sensors::new(board, can_interface.clone(), led_sender.clone(), top_lidar_conf, opponent_detection.clone());

        // Setup asserv
        let asserv = Arc::new(Mutex::new(Asserv::new(asserv_hardware)));

        // Setup trajectory callback for opponent detection
        {
            let od = opponent_detection.clone();
            asserv.lock().unwrap().trajectory_callback = Some(Box::new(move |event| {
                match event {
                    TrajectoryEvent::Translation { from, to, robot_a } => {
                        let dx = to.x - from.x;
                        let dy = to.y - from.y;
                        let length = (dx * dx + dy * dy).sqrt();
                        let direction = dy.atan2(dx) - robot_a;
                        let conf = od.conf();
                        let zone = Zone::Corridor {
                            half_width_mm: conf.corridor_half_width_mm,
                            length_mm: length,
                            direction_rad: direction,
                            stop_until_mm: conf.corridor_stop_until_mm,
                        };
                        if !od.preflight(zone) {
                            return false;
                        }
                        od.update_zone(zone);
                        true
                    }
                    TrajectoryEvent::Rotation => {
                        let zone = Zone::Cylinder { radius_mm: od.conf().rotation_radius_mm };
                        if !od.preflight(zone) {
                            return false;
                        }
                        od.update_zone(zone);
                        true
                    }
                    TrajectoryEvent::Done => {
                        od.update_zone(Zone::Inactive);
                        true
                    }
                }
            }));
        }

        // Setup strat
        Strat::init(board, led_sender.clone(), sensors.clone(), meca.clone(), asserv.clone(), rlogger.clone(), opponent_detection.clone());

        Self {
            asserv,
            opponent_detection,
            meca,

            rome_tx,
            rome_rx,
            rlogger,

            led_sender,
            can: can_interface,
            sensors,

            meca_tm_full: false,

            asserv_tm_periodicity: Periodicity::new(Duration::from_millis(500)),
            meca_tm_periodicity: Periodicity::new(Duration::from_millis(1000)),
        }
    }

    /// Intialize states, spawn asserv thread
    pub fn init(&mut self) {
        let asserv = self.asserv.clone();
        let od = self.opponent_detection.clone();

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            use esp_idf_svc::hal::cpu::Core;
            ThreadSpawnConfiguration {
                priority: 20,
                pin_to_core: Some(Core::Core1),
                ..Default::default()
            }
            .set()
            .unwrap();
        }

        std::thread::Builder::new()
            .name("asserv".into())
            .spawn(move || {
                loop {
                    let pos = {
                        let mut a = asserv.lock().unwrap();
                        a.update();
                        *a.cs.position()
                    };
                    od.update_robot_position(pos);
                    std::thread::sleep(Duration::from_millis(10));
                }
            })
            .expect("spawn asserv");

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            ThreadSpawnConfiguration::default().set().unwrap();
        }
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
        // Process ROME input messages
        let rome_messages: Vec<_> = self.rome_rx.try_iter().collect();
        if !rome_messages.is_empty() {
            self.led_sender.send(LedMessage::RomeActivity).ok();
        }
        for data in rome_messages {
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !self.on_rome_message(&message) & !self.asserv.lock().unwrap().on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }

        // Send asserv telemetry
        if self.asserv_tm_periodicity.update(now) {
            let asserv = self.asserv.lock().unwrap();
            if let Err(err) = self.rome_tx.send(asserv.asserv_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Err(err) = self.rome_tx.send(asserv.asserv_holo_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Some(message) = asserv.asserv_holo_tm_path() {
                if let Err(err) = self.rome_tx.send(message.encode()) {
                    log::error!("ROME send error: {:?}", err);
                }
            }
        }

        // Send meca telemetry
        if self.meca_tm_periodicity.update(now) {
            if self.meca_tm_full {
                for side in 0..3u8 {
                    for arm in 0..4u8 {
                        let s = self.meca.proxy.arm_watcher(side, arm).get();
                        let _ = self.rome_tx.send(rome::Message::MecaTmArmFullState {
                            side,
                            arm,
                            position: s.position,
                            color: rome::params::MecaTmArmFullStateColor::Unknown,
                            pump: s.pump,
                            valve: s.valve,
                            servo_error: s.error,
                            torque_enabled: s.flags.torque_enabled,
                            moving: s.flags.moving,
                            // Note: position_reached == !moving
                            pump_current: s.pump_current,
                        }.encode());
                    }

                    let watcher = self.meca.proxy.translation_watcher(side).get();
                    let _ = self.rome_tx.send(rome::Message::MecaTmSideTranslation {
                        side: side as u8,
                        position: watcher.position,
                        error: watcher.error,
                    }.encode());
                }
            } else {
                for (i, side_state) in self.meca.clone_state().into_iter().enumerate() {
                    fn convert_team(team: Team) -> u8 {
                        match team {
                            Team::None => 0,
                            Team::Left => 1,
                            Team::Right => 2,
                        }
                    }
                    fn convert_stage(teams: &[Team; 4]) -> [u8; 4] {
                        [
                            convert_team(teams[0]),
                            convert_team(teams[1]),
                            convert_team(teams[2]),
                            convert_team(teams[3]),
                        ]
                    }

                    let _ = self.rome_tx.send(rome::Message::MecaTmSideState {
                        side: i as u8,
                        ready_to_take: side_state.ready_to_take,
                        lower_stage: convert_stage(&side_state.lower_stage),
                        upper_stage: convert_stage(&side_state.upper_stage),
                    }.encode());
                }
            }
        }
    }

    fn on_rome_message(&mut self, message: &rome::Message) -> bool {
        match *message {
            rome::Message::MecaSetTmLevel(level) => {
                match level {
                    rome::params::MecaSetTmLevelParam::Default => { self.meca_tm_full = false; },
                    rome::params::MecaSetTmLevelParam::Full => { self.meca_tm_full = true; },
                }
                true
            }
            rome::Message::MecaPrepareTake { side, cleat_up } => {
                log::info!("ROME: meca prepare take");
                let cleat_up = match cleat_up {
                    rome::params::MecaPrepareTakeCleatUp::None => CleatSide::None,
                    rome::params::MecaPrepareTakeCleatUp::Left => CleatSide::Left,
                    rome::params::MecaPrepareTakeCleatUp::Right => CleatSide::Right,
                    rome::params::MecaPrepareTakeCleatUp::Both => CleatSide::Both,
                };
                match side {
                    rome::params::MecaPrepareTakeSide::Left  => { self.meca.prepare_direct_take(Some(asserv::holonomic::RobotSide::Left), cleat_up); }
                    rome::params::MecaPrepareTakeSide::Right => { self.meca.prepare_direct_take(Some(asserv::holonomic::RobotSide::Right), cleat_up); }
                    rome::params::MecaPrepareTakeSide::Back  => { self.meca.prepare_direct_take(Some(asserv::holonomic::RobotSide::Back), cleat_up);  }
                }
                true
            }
            rome::Message::MecaTake { side } => {
                log::info!("ROME: meca take");
                match side {
                    rome::params::MecaTakeSide::Left  => { self.meca.direct_take(asserv::holonomic::RobotSide::Left); }
                    rome::params::MecaTakeSide::Right => { self.meca.direct_take(asserv::holonomic::RobotSide::Right); }
                    rome::params::MecaTakeSide::Back  => { self.meca.direct_take(asserv::holonomic::RobotSide::Back);  }
                }
                true
            }
            rome::Message::MecaPrepareRelease { side } => {
                log::info!("ROME: meca prepare release");
                match side {
                    rome::params::MecaPrepareReleaseSide::Left  => { self.meca.prepare_release(Some(asserv::holonomic::RobotSide::Left)); }
                    rome::params::MecaPrepareReleaseSide::Right => { self.meca.prepare_release(Some(asserv::holonomic::RobotSide::Right)); }
                    rome::params::MecaPrepareReleaseSide::Back  => { self.meca.prepare_release(Some(asserv::holonomic::RobotSide::Back));  }
                }
                true
            }
            rome::Message::MecaRelease { side } => {
                log::info!("ROME: meca release");
                match side {
                    rome::params::MecaReleaseSide::Left  => { self.meca.release(asserv::holonomic::RobotSide::Left); }
                    rome::params::MecaReleaseSide::Right => { self.meca.release(asserv::holonomic::RobotSide::Right); }
                    rome::params::MecaReleaseSide::Back  => { self.meca.release(asserv::holonomic::RobotSide::Back);  }
                }
                true
            }
            rome::Message::MecaEndOfMatch  => {
                log::info!("ROME: meca end of match");
                self.meca.end_of_match();
                true
            }
            rome::Message::MecaRawSetServo { module, id, position } => {
                match id {
                    10..14 => {
                        self.meca.proxy.set_torque(module, id -10, true);
                        self.meca.proxy.set_arm_position(module, id - 10, position, 50);
                    }
                    20 => {
                        self.meca.proxy.set_clamp_torque(module, ClampServo::Rotate, true);
                        self.meca.proxy.set_clamp_position(module, ClampServo::Rotate, position, 50);
                    }
                    21 => {
                        self.meca.proxy.set_clamp_torque(module, ClampServo::Left, true);
                        self.meca.proxy.set_clamp_position(module, ClampServo::Left, position, 50);
                    }
                    22 => {
                        self.meca.proxy.set_clamp_torque(module, ClampServo::Right, true);
                        self.meca.proxy.set_clamp_position(module, ClampServo::Right, position, 50);
                    }
                    30 => {
                        self.meca.proxy.set_translation(module, position, 50);
                    }
                    _ =>  {}
                }
                true
            }
            rome::Message::GotoXY {x, y} => {
                let mut asserv = self.asserv.lock().unwrap();
                asserv.goto_xy(x, y);

                true
            }
            rome::Message::GotoXYRel {x, y} => {
                let mut asserv = self.asserv.lock().unwrap();
                asserv.goto_xy_rel(x, y);

                true
            }
            rome::Message::GotoASide { robot, table } => {
                let robot_side = match robot {
                    rome::params::GotoASideRobot::Back => RobotSide::Back,
                    rome::params::GotoASideRobot::Left => RobotSide::Left,
                    rome::params::GotoASideRobot::Right => RobotSide::Right,                    
                };
                let table_side = match table {
                    rome::params::GotoASideTable::Up => TableSide::Up,
                    rome::params::GotoASideTable::Left => TableSide::Left,
                    rome::params::GotoASideTable::Right => TableSide::Right, 
                    rome::params::GotoASideTable::Down => TableSide::Down,
                };
                let mut asserv = self.asserv.lock().unwrap();
                asserv.goto_a(arfast(robot_side, table_side));

                true
            }
            _ => false
        }
    }

}

