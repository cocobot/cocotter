use asserv::holonomic::RobotSide;
use board_common::Team;
use board_sabotter::SabotterBoard;
use flume::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::led::LedMessage;
use crate::meca::primitives::{ALL_ARMS, MecaPrimitives};
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
    // Not a direct action, change worker's state
    SetOwnColor(Team),
}

pub struct MecaWorker<B: SabotterBoard> {
    proxy: MecaProxy<B>,
    primitives: MecaPrimitives<B>,
    led_tx: Sender<LedMessage>,
    state: Arc<Mutex<MecaState>>,
    rx: Receiver<MecaAction>,
    own_color: Team,
}

impl<B: SabotterBoard> MecaWorker<B> {
    pub fn new(
        proxy: MecaProxy<B>,
        primitives: MecaPrimitives<B>,
        led_tx: Sender<LedMessage>,
        state: Arc<Mutex<MecaState>>,
        rx: Receiver<MecaAction>,
    ) -> Self {
        Self { proxy, primitives, led_tx, state, rx, own_color: Team::None }
    }

    pub fn run(mut self) {
        loop {
            let Ok(action) = self.rx.recv() else { break };
            self.handle(action);
        }
    }

    fn handle(&mut self, action: MecaAction) {
        match action {
            MecaAction::PrepareDirectTake { prefered_side, reply , cleat_up} => {
                let chosen = self.compute_prepare_direct_take(prefered_side);
                if let Some((side, need_transfer)) = chosen {
                    reply.send(Some(side)).ok();
                    if need_transfer {
                        self.do_transfer_to_clamp(side);
                    }
                    self.do_prepare_direct_take(side, cleat_up);
                }
                else {
                    reply.send(None).ok();
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
                    self.do_prepare_release_angle(side);
                }
            }
            MecaAction::Release { side, reply } => {
                self.do_release(side);
                reply.send(()).ok();
            }
            MecaAction::SetOwnColor(color) => {
                self.own_color = color;
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

    fn compute_prepare_direct_take(&self, prefered_side: Option<RobotSide>) -> Option<(RobotSide, bool)> {
        let state = self.state.lock().unwrap();

        if let Some(prefered_side) = prefered_side {
            let side_state = &state[Self::side_to_module(prefered_side) as usize];
            if side_state.is_lower_stage_empty() {
                return Some((prefered_side, false));
            }
            if side_state.is_upper_stage_empty() {
                drop(state);
                return Some((prefered_side, true));
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = &state[Self::side_to_module(side) as usize];
            if side_state.is_lower_stage_empty() {
                return Some((side, false));
            }
            if side_state.is_upper_stage_empty() {
                drop(state);
                return Some((side, true));
            }
        }

        None
    }

    fn compute_prepare_release(&self, prefered_side: Option<RobotSide>) -> Option<RobotSide> {
        let state = self.state.lock().unwrap();

        if let Some(prefered_side) = prefered_side {
            let side_state = &state[Self::side_to_module(prefered_side) as usize];
            if !side_state.is_lower_stage_empty() {
                return Some(prefered_side);
            }
            if !side_state.is_upper_stage_empty() {
                return Some(prefered_side);
            }
        }

        for side in [RobotSide::Left, RobotSide::Back, RobotSide::Right] {
            let side_state = &state[Self::side_to_module(side) as usize];
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

        let upper_stage_up = {
            let mut state = self.state.lock().unwrap();
            let side_state = &mut state[module as usize];

            let upper_stage_up = side_state.is_upper_stage_up();
            side_state.transfer_to_clamp();
            side_state.upper_stage_up(true);

            upper_stage_up
        };
        self.led_tx.send(LedMessage::MecaColors { module, teams: [Team::None; 4] }).ok();

        if upper_stage_up {
            self.primitives.arms_pre_release_good_color(module, ALL_ARMS);        
            self.primitives.clamp_open(module);
            self.primitives.clamp_rotate_pickup(module);
        }

        self.primitives.clamp_open(module);
        self.primitives.clamp_rotate_pickup(module);
        self.primitives.arms_up(module, ALL_ARMS);
        self.primitives.translation_close(module);

        self.primitives.clamp_close(module);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.arms_up_for_clamp_release(module, ALL_ARMS);
        self.primitives.releases(module, ALL_ARMS);
        std::thread::sleep(Duration::from_millis(500));
        self.primitives.clamp_rotate_hold(module);
        std::thread::sleep(Duration::from_millis(500));
        self.primitives.end_releases(module, ALL_ARMS);
    }

    fn do_prepare_direct_take(&self, side: RobotSide, cleat_up: CleatSide) {
        let module = Self::side_to_module(side);
        log::info!("Prepare DT {}", module);

        let (is_upper_empty, side_to_reset) = {
            let mut state = self.state.lock().unwrap();
            
            let mut side_to_reset = None;
            for i in 0..3 {
                if module != i {
                    let other_side_state = &mut state[i as usize];
                    if other_side_state.is_ready_to_take() {
                        side_to_reset = Some(i);
                    }
                }
            }

            let side_state = &mut state[module as usize];
            side_state.ready_to_take(true);

            (side_state.is_upper_stage_empty(), side_to_reset)
        };
        log::info!("UE {} TR {:?}", is_upper_empty, side_to_reset);

        if let Some(side_to_reset) = side_to_reset {
            self.primitives.arms_up(side_to_reset, &[0, 1, 2, 3]);
            self.primitives.translation_close(side_to_reset);
            {
                let mut state = self.state.lock().unwrap();
                let side_to_reset_state = &mut state[side_to_reset as usize];
                side_to_reset_state.ready_to_take(false);
            }
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

        log::info!("Is upper empty: {}", is_upper_empty);
        if is_upper_empty {
            self.primitives.clamp_rotate_pickup(module);
            self.primitives.clamp_open(module);
            {
                let mut state = self.state.lock().unwrap();
                let side_to_reset_state = &mut state[module as usize];
                side_to_reset_state.upper_stage_up(false);
            }
        }
    }

    fn do_transfer_to_lower_stage(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        let teams = {
            let mut state = self.state.lock().unwrap();
            state[module as usize].upper_stage_up(false);
            state[module as usize].transfer_to_lower_stage()
        };
        self.led_tx.send(LedMessage::MecaColors { module, teams: teams }).ok();

        self.primitives.translation_close(module);
        self.primitives.arms_up(module, &[0, 1, 2, 3]);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        self.primitives.clamp_rotate_pickup(module);
        std::thread::sleep(Duration::from_millis(250));
        self.primitives.clamp_open(module);
    }

    fn do_prepare_release_angle(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        let (needs_transfer, upper_stage_empty, upper_stage_is_up) = {
            let state = self.state.lock().unwrap();
            let side_state = &state[Self::side_to_module(side) as usize];
            log::info!("Test {} {}", side_state.is_lower_stage_empty(), side_state.is_upper_stage_empty());
            (side_state.is_lower_stage_empty() && !side_state.is_upper_stage_empty(), side_state.is_upper_stage_empty(), side_state.is_upper_stage_up())
        };
        if needs_transfer {
            self.do_transfer_to_lower_stage(side);
        }

        log::info!("Give space to arm ? {} {}", upper_stage_empty, upper_stage_is_up);

        if upper_stage_empty || !upper_stage_is_up {
            log::info!("Give space to arm {} {}", upper_stage_empty, upper_stage_is_up);
            self.primitives.arms_give_space_from_clamp_rotation(module, &[0, 1, 2, 3]);
            std::thread::sleep(Duration::from_millis(500));
            self.primitives.clamp_close(module);
            self.primitives.clamp_rotate_hold(module);

            {
                let mut state = self.state.lock().unwrap();
                let side_state = &mut state[module as usize];
                side_state.upper_stage_up(true);
            }
        }

        self.primitives.arms_pre_release_bad_color(module, &[0, 1, 2, 3]);
    }

    fn do_direct_take(&self, side: RobotSide) -> bool {
        let module = Self::side_to_module(side);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = &mut state[Self::side_to_module(side) as usize];

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

                {
                    let mut state = self.state.lock().unwrap();
                    let side_state = &mut state[Self::side_to_module(side) as usize];
                    side_state.ready_to_take(false);
                }
            }
        }

        self.primitives.arms_down(module, &[0, 1, 2, 3]);
        self.primitives.translation_close(module);
        self.primitives.grabs(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(250));

        self.primitives.arms_up(module, &[0, 1, 2, 3]);

        {
            let mut state = self.state.lock().unwrap();
            let side_state = &mut state[module as usize];

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
            //Color is not important we will read it later
            state[module as usize].set_lower_stage(teams);
        }
    }

    fn do_release(&self, side: RobotSide) {
        let module = Self::side_to_module(side);

        self.do_prepare_release_angle(side);

        let arm_colors = {
            let mut state = self.state.lock().unwrap();
            state[module as usize].set_lower_stage([Team::None; 4])
        };
        self.led_tx.send(LedMessage::MecaColors { module, teams: [Team::None; 4] }).ok();

        let good_color_arms: Vec<_> = arm_colors
            .iter()
            .enumerate()
            .filter_map(|(i, &t)| if t == self.own_color { Some(i as u8) } else { None })
            .collect();

        let bad_color_arms: Vec<_> = arm_colors
            .iter()
            .enumerate()
            .filter_map(|(i, &t)| if t != self.own_color { Some(i as u8) } else { None })
            .collect();

        self.primitives.translation_spread(module);
        self.primitives.arms_pre_release_good_color(module, &good_color_arms);
        std::thread::sleep(Duration::from_millis(250));

        self.primitives.slow_releases(module, &bad_color_arms, Duration::from_millis(250));
        self.primitives.releases(module, &[0, 1, 2, 3]);
        std::thread::sleep(Duration::from_millis(500));
        self.primitives.end_releases(module, &[0, 1, 2, 3]);

        self.primitives.translation_spread(module);
        self.primitives.arms_up(module, &[0, 1, 2, 3]);
    }
}
