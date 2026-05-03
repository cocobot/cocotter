use asserv::holonomic::RobotSide;
use board_common::Team;
use board_sabotter::SabotterBoard;
use flume::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::led::LedMessage;
use crate::meca::primitives::MecaPrimitives;
use crate::meca::proxy::MecaProxy;
use crate::meca::state::MecaState;
use crate::meca::CleatSide;

pub enum MecaAction {
    PrepareDirectTake {
        prefered_side: Option<RobotSide>,
        reply: Sender<Option<RobotSide>>,
        cleat_up: CleatSide,
    },
    DirectTake {
        side: RobotSide,
        reply: Sender<bool>,
    },
    PrepareRelease {
        prefered_side: Option<RobotSide>,
        reply: Sender<Option<RobotSide>>,
    },
    Release {
        side: RobotSide,
        reply: Sender<()>,
    },
}

pub struct MecaWorker<B: SabotterBoard> {
    proxy: MecaProxy<B>,
    primitives: MecaPrimitives<B>,
    led_tx: Sender<LedMessage>,
    state: Arc<Mutex<MecaState>>,
    rx: Receiver<MecaAction>,
}

impl<B: SabotterBoard> MecaWorker<B> {
    pub fn new(
        proxy: MecaProxy<B>,
        primitives: MecaPrimitives<B>,
        led_tx: Sender<LedMessage>,
        state: Arc<Mutex<MecaState>>,
        rx: Receiver<MecaAction>,
    ) -> Self {
        Self { proxy, primitives, led_tx, state, rx }
    }

    pub fn run(self) {
        loop {
            let Ok(action) = self.rx.recv() else { break };
            self.handle(action);
        }
    }

    fn handle(&self, action: MecaAction) {
        match action {
            MecaAction::PrepareDirectTake { prefered_side, reply , cleat_up} => {
                let chosen = self.compute_prepare_direct_take(prefered_side);
                reply.send(chosen).ok();
                if let Some(side) = chosen {
                    self.do_prepare_direct_take(side, cleat_up);
                }
            }
            MecaAction::DirectTake { side, reply } => {
                let result = self.do_direct_take(side);
                reply.send(result).ok();

                self.do_read_color(side);
            }
            MecaAction::PrepareRelease { prefered_side, reply } => {
                let chosen = self.compute_prepare_release(prefered_side);
                reply.send(chosen).ok();
                if let Some(side) = chosen {
                    self.do_transfer_to_lower_stage_if_needed(side);
                }
            }
            MecaAction::Release { side, reply } => {
                self.do_release(side);
                reply.send(()).ok();
            }
        }
    }

    fn side_to_module(side: RobotSide) -> u8 {
        match side {
            RobotSide::Left => 0,
            RobotSide::Back => 1,
            RobotSide::Right => 2,
        }
    }

    fn compute_prepare_direct_take(&self, prefered_side: Option<RobotSide>) -> Option<RobotSide> {
        let state = self.state.lock().unwrap();

        if let Some(prefered_side) = prefered_side {
            let side_state = state.get_side(Self::side_to_module(prefered_side));
            if side_state.is_lower_stage_empty() {
                return Some(prefered_side);
            }
            if side_state.is_upper_stage_empty() {
                drop(state);
                self.do_transfer_to_clamp(prefered_side);
                return Some(prefered_side);
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = state.get_side(Self::side_to_module(side));
            if side_state.is_lower_stage_empty() {
                return Some(side);
            }
            if side_state.is_upper_stage_empty() {
                drop(state);
                self.do_transfer_to_clamp(side);
                return Some(side);
            }
        }

        None
    }

    fn compute_prepare_release(&self, prefered_side: Option<RobotSide>) -> Option<RobotSide> {
        let state = self.state.lock().unwrap();

        if let Some(prefered_side) = prefered_side {
            let side_state = state.get_side(Self::side_to_module(prefered_side));
            if !side_state.is_lower_stage_empty() {
                return Some(prefered_side);
            }
            if !side_state.is_upper_stage_empty() {
                return Some(prefered_side);
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = state.get_side(Self::side_to_module(side));
            if !side_state.is_lower_stage_empty() {
                return Some(side);
            }
            if !side_state.is_upper_stage_empty() {
                return Some(side);
            }
        }

        None
    }

    fn do_transfer_to_clamp(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);
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

    fn do_prepare_direct_take(&self, side: RobotSide, cleat_up: CleatSide) {
        let module = Self::side_to_module(side);

        let is_upper_empty = {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);
            log::info!("SET READY TO TAKE");
            side_state.ready_to_take(true);

            side_state.is_upper_stage_empty()
        };

        if is_upper_empty {
            self.primitives.clamp_rotate_pickup(module);
            self.primitives.clamp_open(module);
        }

        self.proxy.set_color_led_pwm(255);
        self.primitives.translation_spread(module);
        match cleat_up {
            CleatSide::None => self.primitives.arms_pre_grab(module, &[0, 1, 2, 3]),
            CleatSide::Both => {
                self.primitives.arms_pre_grab(module, &[1, 2,]);
                self.primitives.arms_cleat_up(module, &[0, 3]);
            }
            CleatSide::Left => {
                self.primitives.arms_pre_grab(module, &[1, 2, 3]);
                self.primitives.arms_cleat_up(module, &[0]);
            }
            CleatSide::Right => {
                self.primitives.arms_pre_grab(module, &[0, 1, 2, 3]);
                self.primitives.arms_cleat_up(module, &[3]);
            }
        }
    }

    fn do_transfer_to_lower_stage(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);
            side_state.transfer_to_lower_stage();
        }

        self.primitives.arms_up(module, &[0, 1, 2, 3]);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        self.primitives.clamp_rotate_pickup(module);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.clamp_open(module);
    }

    fn do_transfer_to_lower_stage_if_needed(&self, side: RobotSide) {
        let needs_transfer = {
            let state = self.state.lock().unwrap();
            let side_state = state.get_side(Self::side_to_module(side));
            log::info!("Test {} {}", side_state.is_lower_stage_empty(), side_state.is_upper_stage_empty());
            side_state.is_lower_stage_empty() && !side_state.is_upper_stage_empty()
        };
        if needs_transfer {
            self.do_transfer_to_lower_stage(side);
        }
    }

    fn do_direct_take(&self, side: RobotSide) -> bool {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);

            let is_lower_empty = side_state.is_lower_stage_empty();
            let is_upper_empty = side_state.is_upper_stage_empty();
            let is_ready_to_take = side_state.is_ready_to_take();
            log::info!("RESET READY TO TAKE");
            side_state.ready_to_take(false);
            drop(state);

            log::info!("Test {} {} {}", is_lower_empty, is_upper_empty, is_ready_to_take);

            if !is_ready_to_take || !is_lower_empty {
                log::warn!("Direct take: side {:?} is not ready to take....", side);

                if !is_lower_empty {
                    if !is_upper_empty {
                        log::warn!("Direct take: both stages of side {:?} are full, cannot take", side);
                        return false;
                    }
                    self.do_transfer_to_clamp(side);
                }
                self.do_prepare_direct_take(side, CleatSide::Both);
            }
        }

        self.primitives.arms_down(module, &[0, 1, 2, 3]);
        self.primitives.translation_close(module);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));

        self.primitives.arms_up(module, &[0, 1, 2, 3]);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);

            //Color is not important we will read it later
            side_state.set_lower_stage([Team::Left, Team::Left, Team::Right, Team::Right]);
        }

        true
    }

    fn do_read_color(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        std::thread::sleep(Duration::from_millis(1000));
        let teams = self.primitives.read_arms_teams(module);
        self.led_tx.send(LedMessage::MecaColors { module, teams }).ok();
        {
            let mut state = self.state.lock().unwrap();
            let side_state = state.get_side_mut(module);

            //Color is not important we will read it later
            side_state.set_lower_stage([Team::Left, Team::Left, Team::Right, Team::Right]);
        }
    }

    fn do_release(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        self.do_transfer_to_lower_stage_if_needed(side);

        let (own_color, arm_colors) = {
            let mut state = self.state.lock().unwrap();
            let own_color = state.get_own_color();
            let side_state = state.get_side_mut(module);
            (own_color, side_state.set_lower_stage([Team::None; 4]))
        };

        let good_color_arms = arm_colors
            .iter()
            .enumerate()
            .filter_map(|(i, &t)| if t == own_color { Some(i as u8) } else { None })
            .collect::<Vec<_>>();

        self.primitives.translation_spread(module);
        self.primitives.arms_pre_grab(module, &good_color_arms);
        self.primitives.releases(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.end_releases(module, &[0, 1, 2, 3]);

        self.primitives.translation_spread(module);
        self.primitives.arms_up(module, &[0, 1, 2, 3]);
    }
}
