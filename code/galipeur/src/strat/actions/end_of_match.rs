use asserv::maths::XY;
use board_sabotter::SabotterBoard;

use crate::strat::actions::END_OF_MATCH;
use crate::strat::errors::StrategyError;
use crate::strat::planner::{Action, WorldState};
use crate::strat::Strat;

pub struct EndOfMatchAction {
    pub x: f32,
    pub y: f32,
}

impl<B: SabotterBoard + 'static> Action<B> for EndOfMatchAction {
    fn label(&self) -> &str { "end_of_match" }
    fn position(&self) -> XY { XY::new(self.x, self.y) }
    fn base_duration(&self) -> f32 { 1.5 }
    fn secured_delta(&self, _state: &WorldState) -> i32 { 10 }
    fn potential_delta(&self, _state: &WorldState) -> i32 { 0 }
    fn base_risk(&self, _state: &WorldState) -> f32 { 0.0 }

    fn is_available(&self, state: &WorldState) -> bool {
        !state.has_flag(END_OF_MATCH)
    }

    fn apply(&self, state: &mut WorldState) {
        let secured = <Self as Action<B>>::secured_delta(self, state);
        state.set_flag(END_OF_MATCH);
        state.secured_points += secured;
    }

    fn run(&self, strat: &Strat<B>) -> Result<(), StrategyError> {
        strat.end_of_match();
    }
}
