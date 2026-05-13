pub mod state;
mod search;
pub mod debug;

pub use search::plan;
pub use state::WorldState;

use asserv::maths::XY;
use board_sabotter::SabotterBoard;

use crate::strat::errors::StrategyError;
use crate::strat::Strat;

pub type ActionId = usize;

/// Trait for an action that is both plannable (simulated during search)
/// and executable (run on the real robot).
///
/// The planner only calls `is_available()`, `apply()`, and the query
/// methods. `run()` is called by `Strat` after the planner has chosen
/// the best sequence.
pub trait Action<B: SabotterBoard> {
    /// Human-readable label for debug / log output.
    fn label(&self) -> &str;

    /// Position on the table where the action takes place (mm).
    fn position(&self) -> XY;

    /// Estimated duration of the action itself, excluding travel (seconds).
    fn base_duration(&self) -> f32;

    /// Change in secured points when this action completes.
    fn secured_delta(&self, state: &WorldState) -> i32;

    /// Change in potential points (positive = accumulate, negative = convert to secured).
    fn potential_delta(&self, state: &WorldState) -> i32;

    /// Static risk factor (0.0 = safe, 1.0 = dangerous corner / dead-end).
    fn base_risk(&self, state: &WorldState) -> f32;

    /// Probability (0.0–1.0) that the action's target is still available.
    /// Accounts for time elapsed and opponent proximity.
    /// Multiplies the expected point value in the scoring formula.
    fn success_probability(&self, _state: &WorldState, _opponent_pos: Option<XY>) -> f32 { 1.0 }

    /// Whether this action can be performed in the given world state.
    fn is_available(&self, state: &WorldState) -> bool;

    /// Simulate the effects on the world state (flags, counters, points).
    /// Called by the planner during search — must not have side effects.
    fn apply(&self, state: &mut WorldState);

    /// Execute the action for real on the robot hardware.
    fn run(&self, strat: &Strat<B>) -> Result<(), StrategyError>;
}

/// Result of the planner search.
pub struct Plan {
    /// Ordered list of action indices to execute.
    pub sequence: Vec<ActionId>,
    /// Secured points at the end of the sequence.
    pub expected_score: i32,
    /// Time remaining after executing the full sequence (seconds).
    pub time_remaining: f32,
}
