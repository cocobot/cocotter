mod primitives;
mod proxy;
mod state;

use board_common::Team;
pub use primitives::MecaPrimitives;
pub use proxy::{ArmStatus, ClampStatus, ColorRawStatus, MecaProxy, TranslationStatus, Watcher};
use cancaner::ClampServo;

use asserv::holonomic::RobotSide;
use board_sabotter::SabotterBoard;
use flume::Sender;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::can::GalipeurCan;
use crate::led::LedMessage;
use crate::meca::state::MecaState;

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
    state: Arc<Mutex<MecaState>>,
}

impl<B: SabotterBoard> Clone for Meca<B> {
    fn clone(&self) -> Self {
        Self {
            proxy: self.proxy.clone(),
            primitives: self.primitives.clone(),
            led_tx: self.led_tx.clone(),
            state: self.state.clone(),
        }
    }
}

impl<B: SabotterBoard> Meca<B> {
    pub fn new(can: GalipeurCan<B>, led_tx: Sender<LedMessage>) -> Self {
        let proxy = MecaProxy::new(can);
        let primitives = MecaPrimitives::new(proxy.clone());
        Self { proxy, primitives, led_tx, state: Arc::new(Mutex::new(MecaState::default())) }
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

    pub fn init(&self, team: Team) {
        {
            let mut state = self.state.lock().unwrap();
            state.set_own_color(team);
        }

        // Raise all arms to the rest (up) position on every module.
        for module in 0..3 {
            self.primitives.arms_up(module, &[0, 1, 2, 3]);     
            self.primitives.clamp_rotate_pickup(module);     
            self.primitives.clamp_open(module);  
            self.primitives.translation_spread(module);
        }
    }

    fn side_to_module(side: RobotSide) -> u8 {
        match side {
            RobotSide::Left => 0,
            RobotSide::Back => 1,
            RobotSide::Right => 2,
        }
    }

    //TODO: move do-* functions to thread to avoid blocking strat while doing meca actions
    fn do_transfer_to_clamp(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_state_mut(module);
            side_state.transfer_to_clamp();
        }
        
        self.primitives.clamp_open(module);
        self.primitives.clamp_rotate_pickup(module);
        self.primitives.arms_up(module, &[0, 1, 2, 3]);
        self.primitives.translation_close(module);
        
        self.primitives.clamp_close(module);
        self.primitives.releases(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.clamp_rotate_hold(module);
        self.primitives.end_releases(module, &[0, 1, 2, 3]);
    }

    fn do_prepare_direct_take(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_state_mut(module);
            side_state.ready_to_take(true);
        }

        self.proxy.set_color_led_pwm(255);
        self.primitives.translation_spread(module);
        self.primitives.arms_pre_grab(module, &[0, 1, 2, 3]);
    }

    fn do_transfer_to_lower_stage(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_state_mut(module);
            side_state.transfer_to_lower_stage();
        }

        self.primitives.arms_up(module, &[0, 1, 2, 3]);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        self.primitives.clamp_rotate_pickup(module);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.clamp_open(module);
    }

    fn do_idle(&self, module: u8) {
        self.primitives.translation_spread(module);
        self.primitives.arms_up(module, &[0, 1, 2, 3]);
    }

    pub fn prepare_direct_take(&self, prefered_side: Option<RobotSide>) -> Option<RobotSide> {
        let state = self.state.lock().unwrap();

        //check if prefere side is available
        if let Some(prefered_side) = prefered_side {
            let side_state = state.get_side_state(Self::side_to_module(prefered_side));
            if side_state.is_lower_stage_empty() {
                drop(state);
                
                self.do_prepare_direct_take(prefered_side);
                return Some(prefered_side);
            }
            if side_state.is_upper_stage_empty() {
                drop(state);

                self.do_transfer_to_clamp(prefered_side);
                self.do_prepare_direct_take(prefered_side);
                return Some(prefered_side);
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = state.get_side_state(Self::side_to_module(side));
            if side_state.is_lower_stage_empty() {
                drop(state);

                self.do_prepare_direct_take(side);
                return Some(side);
            }
            if side_state.is_upper_stage_empty() {
                drop(state);
                self.do_transfer_to_clamp(side);
                self.do_prepare_direct_take(side);
                return Some(side);
            }
        }

        None
    }

    pub fn direct_take(&self, side: RobotSide) -> bool {       
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_state_mut(module);

            let is_lower_empty = side_state.is_lower_stage_empty();
            let is_upper_empty = side_state.is_upper_stage_empty();
            let is_ready_to_take = side_state.is_ready_to_take();
            side_state.ready_to_take(false);
            drop(state);

            if !is_ready_to_take {
                log::warn!("Direct take: side {:?} is not ready to take....", side);

                if !is_lower_empty {
                    if !is_upper_empty {
                        log::warn!("Direct take: both stages of side {:?} are full, cannot take", side);
                        return false;
                    }
                    self.do_transfer_to_clamp(side);
                }
                self.do_prepare_direct_take(side);
            }
        }

        self.primitives.arms_down(module, &[0, 1, 2, 3]);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.arms_up(module, &[0, 1, 2, 3]);

        let teams = self.primitives.read_arms_teams(module);
        self.led_tx.send(LedMessage::MecaColors { module, teams }).ok();

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_state_mut(module);
            side_state.set_lower_stage(teams);
        }

        true
    }

    pub fn prepare_release(&self, prefered_side: Option<RobotSide>) -> Option<RobotSide> {
        let state = self.state.lock().unwrap();

        //check if prefere side is available
        if let Some(prefered_side) = prefered_side {
            let side_state = state.get_side_state(Self::side_to_module(prefered_side));
            if !side_state.is_lower_stage_empty() {
                return Some(prefered_side);
            }
            if !side_state.is_upper_stage_empty() {
                drop(state);

                self.do_transfer_to_lower_stage(prefered_side);
                return Some(prefered_side);
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = state.get_side_state(Self::side_to_module(side));
            if !side_state.is_lower_stage_empty() {
                return Some(side);
            }
            if !side_state.is_upper_stage_empty() {
                drop(state);
                
                self.do_transfer_to_lower_stage(side);
                return Some(side);
            }
        }

        None
    }

    pub fn release(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        let (own_color, arm_colors) = {
            let mut state = self.state.lock().unwrap();
            let own_color = state.get_own_color();
            let side_state = state.get_side_state_mut(module);
            
            (own_color, side_state.set_lower_stage([Team::None; 4]))
        };

        let good_color_arms = arm_colors.iter().enumerate().filter_map(|(i, &t)| if t == own_color { Some(i as u8) } else { None }).collect::<Vec<_>>();

        self.primitives.translation_spread(module);  
        self.primitives.arms_pre_grab(module, &good_color_arms);      
        self.primitives.releases(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.end_releases(module, &[0, 1, 2, 3]);

        self.do_idle(module);
    }


    #[allow(dead_code)]
    pub fn calibration_position(&self) {        
        for module in 0..3 {
            self.primitives.arms_down(module, &[0, 1, 2, 3]);     
            self.primitives.clamp_close(module);  
            self.primitives.clamp_rotate_hold(module);                   
            self.primitives.translation_spread(0);
        }
    }

    #[allow(dead_code)]
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
