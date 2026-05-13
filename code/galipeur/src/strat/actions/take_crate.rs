use asserv::holonomic::{RobotSide, TableSide};
use asserv::maths::XY;
use board_sabotter::SabotterBoard;

use crate::meca::CleatSide;
use crate::strat::actions::END_OF_MATCH;
use crate::strat::errors::StrategyError;
use crate::strat::planner::{Action, WorldState};
use crate::strat::utils::arfast;
use crate::strat::Strat;

use board_common::Team;

use super::{CTR_HELD, MAX_CRATES};

/// Spot positions for take actions (spot_flag → (x, y)).
fn spot_position(spot: u8) -> (f32, f32) {
    match spot {
        0 => (1350.0, 1200.0),
        1 => (-1350.0, 1200.0),
        2 => (1350.0, 400.0),
        3 => (-1350.0, 1200.0),
        4 => (350.0, 800.0),
        5 => (-350.0, 800.0),
        6 => (400.0, 175.0),
        7 => (400.0, 175.0),
        _ => panic!("unknown take spot {}", spot),
    }
}

pub struct TakeCrateAction {
    label: String,
    x: f32,
    y: f32,
    side: TableSide,
    spot_flag: u8,
    team: Team,
}

impl TakeCrateAction {
    const PRETAKE_DISTANCE: f32 = 275.0;
    const RISK: f32 = 0.1;

    pub fn new(spot_flag: u8, side: TableSide, team: Team) -> Self {
        let (x, y) = spot_position(spot_flag);
        let side_suffix = match side {
            TableSide::Up => "_u",
            TableSide::Down => "_d",
            TableSide::Left => "_l",
            TableSide::Right => "_r",
        };
        Self {
            label: format!("take_{}{}", spot_flag, side_suffix),
            x,
            y,
            side,
            spot_flag,
            team,
        }
    }

    fn approach_offset(&self) -> (f32, f32) {
        Self::side_offset(self.side)
    }

    pub fn side_offset(side: TableSide) -> (f32, f32) {
        match side {
            TableSide::Down => (0.0, Self::PRETAKE_DISTANCE),
            TableSide::Up => (0.0, -Self::PRETAKE_DISTANCE),
            TableSide::Left => (Self::PRETAKE_DISTANCE, 0.0),
            TableSide::Right => (-Self::PRETAKE_DISTANCE, 0.0),
        }
    }

    pub fn execute<B: SabotterBoard + 'static>(strat: &Strat<B>, x: f32, y: f32, face: Option<RobotSide>, side: TableSide) -> Result<(), StrategyError> {
        let face = match strat.meca.prepare_direct_take(face, CleatSide::Both) {
            Some(face) => face,
            None => return Err(StrategyError::StupidOrder),
        };

        let (ox, oy) = Self::side_offset(side);
        strat.asserv.goto_xya(x + ox, y + oy, arfast(face, side))?;
        strat.approach_and_take(face, side)
    }
}

/// How quickly a spot's availability decays over the match (higher = more contested).
/// Spots on our wall are safe (low decay), spots on opponent wall are exposed (high decay).
fn spot_decay_rate(spot: u8, team: Team) -> f32 {
    // x > 0 → Right's side, x < 0 → Left's side.
    // Our side = low decay (safe), opponent side = high decay (exposed).
    let ours = team == Team::Right;
    match spot {
        0 | 2 => if ours { 0.3 } else { 0.8 },  // wall x > 0
        1 | 3 => if ours { 0.8 } else { 0.3 },  // wall x < 0
        4     => if ours { 0.4 } else { 0.6 },  // centre x > 0
        5     => if ours { 0.6 } else { 0.4 },  // centre x < 0
        6     => if ours { 0.5 } else { 0.7 },  // bottom x > 0
        7     => if ours { 0.7 } else { 0.5 },  // bottom x < 0
        _ => 0.5,
    }
}

/// Maximum table diagonal (mm), used to normalise opponent distance.
const MAX_TABLE_DIST: f32 = 3600.0;
/// Total match duration (seconds).
const MATCH_DURATION: f32 = 120.0;

impl<B: SabotterBoard + 'static> Action<B> for TakeCrateAction {
    fn label(&self) -> &str { &self.label }
    fn position(&self) -> XY {
        let (ox, oy) = self.approach_offset();
        XY::new(self.x + ox, self.y + oy)
    }
    fn base_duration(&self) -> f32 { 5.0 }
    fn secured_delta(&self, _state: &WorldState) -> i32 { 0 }
    fn potential_delta(&self, _state: &WorldState) -> i32 { 3 * 4 + 5 }
    fn base_risk(&self, _state: &WorldState) -> f32 { Self::RISK }

    fn success_probability(&self, state: &WorldState, opponent_pos: Option<XY>) -> f32 {
        let elapsed_fraction = 1.0 - (state.time_remaining / MATCH_DURATION).clamp(0.0, 1.0);
        let decay = spot_decay_rate(self.spot_flag, self.team);
        let time_prob = 1.0 - decay * elapsed_fraction;

        let opponent_factor = match opponent_pos {
            Some(opp) => {
                let (ox, oy) = self.approach_offset();
                let pos = XY::new(self.x + ox, self.y + oy);
                let dist = (pos - opp).length();
                let closeness = (1.0 - (dist / MAX_TABLE_DIST).min(1.0)).powi(2);
                1.0 - closeness * 0.5
            }
            None => 0.85,
        };

        (time_prob * opponent_factor).clamp(0.05, 1.0)
    }

    fn is_available(&self, state: &WorldState) -> bool {
        !state.has_flag(self.spot_flag) && state.counters[CTR_HELD] < MAX_CRATES && !state.has_flag(END_OF_MATCH)
    }

    fn apply(&self, state: &mut WorldState) {
        let delta = <Self as Action<B>>::potential_delta(self, state);
        state.set_flag(self.spot_flag);
        state.counters[CTR_HELD] += 4;
        state.potential_points += delta;
    }

    fn run(&self, strat: &Strat<B>) -> Result<(), StrategyError> {
        Self::execute(strat, self.x, self.y, None, self.side)
    }
}
