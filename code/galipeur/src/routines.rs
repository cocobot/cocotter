use std::sync::{Arc, Mutex};
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
    pub asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>,
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
        let meca = Meca::new(can_interface.clone(), led_sender.clone());

        // Setup sensors
        let sensors = Sensors::new(board, can_interface.clone(), led_sender.clone(), top_lidar_conf);

        // Setup asserv
        let asserv = Arc::new(Mutex::new(Asserv::new(asserv_hardware)));

        // Setup strat
        Strat::init(board, led_sender.clone(), sensors.clone(), meca.clone() ,asserv.clone());

        Self {
            asserv,
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
        // Process ROME input messages
        let rome_messages: Vec<_> = self.rome_rx.try_iter().collect();
        if !rome_messages.is_empty() {
            self.led_sender.send(LedMessage::RomeActivity).ok();
        }
        //for data in rome_messages {
        //    match rome::Message::decode(&data) {
        //        Err(err) => log::error!("ROME RX error: {err:?}"),
        //        Ok(message) => {
        //            if !self.on_rome_message(&message) && !self.asserv.on_rome_message(&message) {
        //                log::warn!("ROME: ignored message: {}", message.message_id());
        //            }
        //        },
        //    }
        //}
        
        // Update asserv, send asserv telemetry
        if self.asserv_periodicity.update(now) {
            self.asserv.lock().unwrap().update();
        }
        //if self.asserv_tm_periodicity.update(now) {
        //    if let Err(err) = self.rome_tx.send(self.asserv.asserv_tm_status().encode()) {
        //        log::error!("ROME send error: {:?}", err);
        //    }
        //    if let Err(err) = self.rome_tx.send(self.asserv.asserv_holo_tm_status().encode()) {
        //        log::error!("ROME send error: {:?}", err);
        //    }
        //    if let Some(message) = self.asserv.asserv_holo_tm_path() {
        //        if let Err(err) = self.rome_tx.send(message.encode()) {
        //            log::error!("ROME send error: {:?}", err);
        //        }
        //    }
        //}

        // Update meca, send meca telemetry
        if self.meca_periodicity.update(now) {
           //for module in 0..3u8 {
           //    for arm in 0..4u8 {
           //        let s = self.meca.proxy.arm_watcher(module, arm).get();
           //        let _ = self.rome_tx.send(rome::Message::MecaArmTmState {
           //            module,
           //            arm,
           //            position: s.position,
           //            //TODO: update rome with full telemetry
           //            color: rome::params::MecaArmTmStateColor::Unknown,
           //            pump: s.pump,
           //            valve: s.valve,
           //            servo_error: s.error,
           //            torque_enabled: s.flags.torque_enabled,
           //            moving: s.flags.moving,
           //            // Note: position_reached == !moving
           //            pump_current: s.pump_current,
           //        }.encode());
           //    }
           //}
           //if self.meca_tm_periodicity.update(now) {
           //    for (i_tr, translation) in meca_state.translations.iter().enumerate() {
           //        let _ = self.rome_tx.send(rome::Message::MecaArmTmTranslation {
           //            module: i_tr as u8,
           //            position: translation.position,
           //            error: translation.error,
           //        }.encode());
           //    }
           //}
        }
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

