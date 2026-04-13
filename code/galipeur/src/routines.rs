use std::time::{Duration, Instant};
use asserv::holonomic::{Asserv, rome::AsservHoloRome};
use asserv::rome::AsservRome;
use board_common::Periodicity;
use board_sabotter::{SabotterBoard, SabotterLeds};
use cancaner::Color as CanColor;
use embedded_hal::digital::StatefulOutputPin;
use flume::{Receiver, Sender};
use sch16t::Sch16t;
use crate::movement::MovementLowLevelHardware;
use crate::meca::Meca;


/// Everything needed for PAMI routines
///
/// Update state from multiple peripherals.
/// States are updated when calling `idle()` or `step_idle()`.
pub struct GalipeurRoutines<B: SabotterBoard> {
    pub asserv: Asserv<MovementLowLevelHardware<B>>,
    pub meca: Meca<B>,

    // Peripherals
    pub sabotter_leds: SabotterLeds<B::OutputPin, B::ExOutputPin>,

    // ROME sender/receiver
    pub rome_tx: Sender<Box<[u8]>>,
    pub rome_rx: Receiver<Box<[u8]>>,

    // Periodicity states
    asserv_periodicity: Periodicity,
    asserv_tm_periodicity: Periodicity,
    meca_periodicity: Periodicity,
    meca_tm_periodicity: Periodicity,
}

impl<B: SabotterBoard> GalipeurRoutines<B> {
    /// Initialize with default state values and peripherals from board
    ///
    /// Peripherals must be available on the board.
    /// The asserv must be configured manually, using `asserv.set_conf()`.
    pub fn new(
        board: &mut B,
        (rome_tx, rome_rx): (Sender<Box<[u8]>>, Receiver<Box<[u8]>>),
    ) -> Self {
        // Setup gyro, asserv
        let mut gyro = Sch16t::new(board.imu_spi().unwrap(), 0);
        gyro.init().unwrap();
        let asserv_hardware = MovementLowLevelHardware::new(gyro, board.motors().unwrap());

        // Setup meca
        let meca = Meca::new(board.can().unwrap());

        Self {
            asserv: Asserv::new(asserv_hardware),
            meca,

            rome_tx,
            rome_rx,

            sabotter_leds: board.leds().unwrap(),

            asserv_periodicity: Periodicity::new(Duration::from_millis(10)),
            asserv_tm_periodicity: Periodicity::new(Duration::from_millis(100)),
            meca_periodicity: Periodicity::new(Duration::from_millis(100)),
            meca_tm_periodicity: Periodicity::new(Duration::from_millis(500)),
        }
    }

    /// Intialize states and peripherals
    pub fn init(&mut self) {
    }


    /// Execute one round of idle updates
    ///
    /// This method must be called regularly.
    pub fn idle(&mut self, now: &Instant) {

        // Process ROME input messages
        for data in self.rome_rx.try_iter() {
            let _ = self.sabotter_leds.com.toggle();
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !self.asserv.on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }

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

        // Update meca, send meca telemetry
        if self.meca_periodicity.update(now) {
            let meca_state = self.meca.get_state();
            for (i_module, module) in meca_state.arms.iter().enumerate() {
                for (i_arm, arm) in module.iter().enumerate() {
                    let _ = self.rome_tx.send(rome::Message::MecaArmTmState {
                        module: i_module as u8,
                        arm: i_arm as u8,
                        position: arm.position,
                        color: match arm.color {
                            CanColor::Unknown => rome::params::MecaArmTmStateColor::Unknown,
                            CanColor::Yellow => rome::params::MecaArmTmStateColor::Yellow,
                            CanColor::Blue => rome::params::MecaArmTmStateColor::Blue,
                        },
                        pump: arm.pump,
                        valve: arm.valve,
                        servo_error: arm.error,
                        torque_enabled: arm.flags.torque_enabled,
                        moving: arm.flags.moving,
                        // Note: position_reached == !moving
                        pump_current: arm.pump_current,
                    }.encode());
                }
            }
            if self.meca_tm_periodicity.update(now) {
                for (i_tr, translation) in meca_state.translations.iter().enumerate() {
                    let _ = self.rome_tx.send(rome::Message::MecaArmTmTranslation {
                        module: i_tr as u8,
                        position: translation.position,
                        error: translation.error,
                    }.encode());
                }
            }
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

