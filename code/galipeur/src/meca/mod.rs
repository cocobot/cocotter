mod primitives;
mod proxy;

use board_common::Team;
use log::warn;
pub use primitives::MecaPrimitives;
pub use proxy::{ArmStatus, ClampStatus, ColorRawStatus, MecaProxy, TranslationStatus, Watcher};
use cancaner::ClampServo;

use asserv::holonomic::RobotSide;
use board_sabotter::SabotterBoard;
use flume::Sender;
use std::time::Duration;

use crate::can::GalipeurCan;
use crate::led::LedMessage;

/// Mapping RobotSide → CAN module index
pub trait RobotSideModule {
    fn module(self) -> u8;
}

impl RobotSideModule for RobotSide {
    fn module(self) -> u8 {
        match self {
            RobotSide::Left => 0,
            RobotSide::Back => 1,
            RobotSide::Right => 2,
        }
    }
}

/// High-level meca interface — holds the low-level proxy and the primitive layer.
pub struct Meca<B: SabotterBoard> {
    pub proxy: MecaProxy<B>,
    pub primitives: MecaPrimitives<B>,
    led_tx: Sender<LedMessage>,
}

impl<B: SabotterBoard> Clone for Meca<B> {
    fn clone(&self) -> Self {
        Self {
            proxy: self.proxy.clone(),
            primitives: self.primitives.clone(),
            led_tx: self.led_tx.clone(),
        }
    }
}

impl<B: SabotterBoard> Meca<B> {
    pub fn new(can: GalipeurCan<B>, led_tx: Sender<LedMessage>) -> Self {
        let proxy = MecaProxy::new(can);
        let primitives = MecaPrimitives::new(proxy.clone());
        Self { proxy, primitives, led_tx }
    }

    // --- Init ---

    pub fn pre_init(&self) {
        if !self.proxy.ping(Duration::from_millis(3000)) {
            log::error!("Failed to ping Meca");
            return;
        }
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, false);
            }
            self.proxy.set_clamp_torque(module, ClampServo::Left, false);
            self.proxy.set_clamp_torque(module, ClampServo::Right, false);
            self.proxy.set_clamp_torque(module, ClampServo::Rotate, false);
        }
    }

    pub fn init(&self) {
        // Raise all arms to the rest (up) position on every module.
        for module in 0..3 {
            self.primitives.arms_up(module, &[0, 1, 2, 3]);     
            self.primitives.clamp_rotate_pickup(module);     
            self.primitives.clamp_open(module);  
            self.primitives.translation_spread(module);
        }
    }

    #[warn(allow_dead_code)]
    pub fn calibration_position(&self) {        
        for module in 0..3 {
            self.primitives.arms_down(module, &[0, 1, 2, 3]);     
            self.primitives.clamp_close(module);  
            self.primitives.clamp_rotate_hold(module);                   
            self.primitives.translation_spread(0);
        }
    }

    #[warn(allow_dead_code)]
    pub fn calobration_check_color(&self) {   
        for module in 0..3 {     
            let teams = self.primitives.read_arms_teams(module);
            if teams.iter().any(|&t| t != Team::None) {
                self.led_tx.send(LedMessage::MecaColors { module, teams }).ok();   
            }        
        }
    }


    pub fn test_algo(&self, module: u8) {
        let mv = &self.primitives;

        // Setup
        self.proxy.set_color_led_pwm(255);
        mv.clamp_rotate_pickup(module);
        mv.clamp_open(module);
        mv.translation_spread(module);

        // Phase 1 — Prepare grab
        mv.arms_pre_grab(module, &[0, 1, 2, 3]);
        mv.translation_close(module);

        // Phase 2 — grab
        mv.arms_down(module, &[0, 1, 2, 3]);
        mv.grabs(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));
        mv.arms_up(module, &[0, 1, 2, 3]);

        // Phase 3 — read colors
        let teams = mv.read_arms_teams(module);
        self.led_tx.send(LedMessage::MecaColors { module, teams }).ok();
        self.proxy.set_color_led_pwm(0);

        // Phase 4 — transfer to the clamp (all 4 at once)
        mv.clamp_close(module);
        mv.releases(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(500));

        // Phase 5 — store
        mv.clamp_rotate_hold(module);
        mv.end_releases(module, &[0, 1, 2, 3]);

    }
}
