use asserv::holonomic::{RobotSide, TableSide};
use asserv::maths::XY;
use board_sabotter::SabotterBoard;

use crate::strat::actions::END_OF_MATCH;
use crate::strat::errors::StrategyError;
use crate::strat::planner::{Action, WorldState};
use crate::strat::utils::arfast;
use crate::strat::Strat;

use super::CTR_HELD;

/// Spot positions for release actions (spot_flag → (x, y)).
fn spot_position(spot: u8) -> (f32, f32) {
    match spot {
        10 => (1400.0, 800.0),
        11 => (-1400.0, 800.0),
        12 => (800.0, 100.0),
        13 => (-800.0, 100.0),
        14 => (700.0, 800.0),
        15 => (-700.0, 800.0),
        _ => panic!("unknown release spot {}", spot),
    }
}

pub struct ReleaseAction {
    label: String,
    x: f32,
    y: f32,
    side: TableSide,
    spot_flag: u8,
}

impl ReleaseAction {
    const PRERELEASE_DISTANCE: f32 = 230.0;
    const RISK: f32 = 0.3;

    pub fn new(spot_flag: u8, side: TableSide) -> Self {
        let (x, y) = spot_position(spot_flag);
        let side_suffix = match side {
            TableSide::Up => "_u",
            TableSide::Down => "_d",
            TableSide::Left => "_l",
            TableSide::Right => "_r",
        };
        Self {
            label: format!("release_{}{}", spot_flag, side_suffix),
            x,
            y,
            side,
            spot_flag,
        }
    }

    fn approach_offset(&self) -> (f32, f32) {
        Self::side_offset(self.side)
    }

    pub fn side_offset(side: TableSide) -> (f32, f32) {
        match side {
            TableSide::Down => (0.0, Self::PRERELEASE_DISTANCE),
            TableSide::Up => (0.0, -Self::PRERELEASE_DISTANCE),
            TableSide::Left => (Self::PRERELEASE_DISTANCE, 0.0),
            TableSide::Right => (-Self::PRERELEASE_DISTANCE, 0.0),
        }
    }

    pub fn execute<B: SabotterBoard + 'static>(strat: &Strat<B>, x: f32, y: f32, face: Option<RobotSide>, side: TableSide) -> Result<(), StrategyError> {
        let face = match strat.meca.prepare_release(face) {
            Some(face) => face,
            None => return Err(StrategyError::StupidOrder),
        };

        let (ox, oy) = Self::side_offset(side);
        strat.asserv.goto_xya(x + ox, y + oy, arfast(face, side))?;
        strat.release(face, side)
    }
}

/// Maximum table diagonal (mm), used to normalise opponent distance.
const MAX_TABLE_DIST: f32 = 3600.0;

impl<B: SabotterBoard + 'static> Action<B> for ReleaseAction {
    fn label(&self) -> &str { &self.label }
    fn position(&self) -> XY {
        let (ox, oy) = self.approach_offset();
        XY::new(self.x + ox, self.y + oy)
    }
    fn base_duration(&self) -> f32 { 4.0 }
    fn secured_delta(&self, _state: &WorldState) -> i32 { 3 * 4 + 2 }
    fn potential_delta(&self, _state: &WorldState) -> i32 { -(3 * 4 + 2) }
    fn base_risk(&self, _state: &WorldState) -> f32 { Self::RISK }

    fn success_probability(&self, _state: &WorldState, opponent_pos: Option<XY>) -> f32 {
        let opponent_factor = match opponent_pos {
            Some(opp) => {
                let (ox, oy) = self.approach_offset();
                let pos = XY::new(self.x + ox, self.y + oy);
                let dist = (pos - opp).length();
                let closeness = (1.0 - (dist / MAX_TABLE_DIST).min(1.0)).powi(2);
                1.0 - closeness * 0.2
            }
            None => 0.95,
        };
        opponent_factor.clamp(0.05, 1.0)
    }

    fn is_available(&self, state: &WorldState) -> bool {
        !state.has_flag(self.spot_flag) && state.counters[CTR_HELD] > 0 && !state.has_flag(END_OF_MATCH)
    }

    fn apply(&self, state: &mut WorldState) {
        let secured = <Self as Action<B>>::secured_delta(self, state);
        state.set_flag(self.spot_flag);
        let release_count = state.counters[CTR_HELD].min(4);
        state.counters[CTR_HELD] -= release_count;
        state.secured_points += secured;
        state.potential_points -= secured;
    }

    fn run(&self, strat: &Strat<B>) -> Result<(), StrategyError> {
        Self::execute(strat, self.x, self.y, None, self.side)
    }
}
