use std::cmp::Ordering;
use std::collections::BinaryHeap;

use asserv::maths::XY;
use board_sabotter::SabotterBoard;

use super::state::WorldState;
use super::{Action, ActionId, Plan};

/// Maximum search nodes before returning the best result found so far.
const MAX_NODES: usize = 5000;

/// Minimum time remaining (seconds) to consider adding more actions.
const MIN_TIME_BUFFER: f32 = 5.0;

/// Weight applied to the risk penalty.
const RISK_WEIGHT: f32 = 10.0;


struct SearchNode {
    state: WorldState,
    sequence: Vec<ActionId>,
    /// Risk-adjusted score used for search ordering.
    adjusted_score: f32,
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.adjusted_score == other.adjusted_score
    }
}
impl Eq for SearchNode {}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.adjusted_score
            .partial_cmp(&other.adjusted_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Find the best action sequence to maximise score within the remaining time.
///
/// - `travel_cost`: estimates travel time (seconds) between two positions,
///   typically computed via A* on the PathGraph.
/// - `opponent_pos`: last known opponent position, if any.
pub fn plan<B: SabotterBoard>(
    actions: &[Box<dyn Action<B>>],
    initial_state: WorldState,
    travel_cost: &dyn Fn(XY, XY) -> f32,
    opponent_pos: Option<XY>,
) -> Plan {
    log::info!(
        "Planner: starting from ({:.0}, {:.0}), time {:.1}s, {} actions",
        initial_state.position.x, initial_state.position.y,
        initial_state.time_remaining,
        actions.len()
    );

    let initial_score = initial_state.score();
    let initial_secured = initial_state.secured_points;
    let initial_time = initial_state.time_remaining;

    let mut heap = BinaryHeap::new();
    heap.push(SearchNode {
        state: initial_state,
        sequence: Vec::new(),
        adjusted_score: initial_score,
    });

    let mut best_sequence: Vec<ActionId> = Vec::new();
    let mut best_adjusted: f32 = initial_score;
    let mut best_secured: i32 = initial_secured;
    let mut best_time_remaining: f32 = initial_time;
    let mut nodes_explored: usize = 0;

    while let Some(node) = heap.pop() {
        nodes_explored += 1;
        if nodes_explored >= MAX_NODES {
            log::warn!("Planner: hit MAX_NODES limit ({}), result may be suboptimal", MAX_NODES);
            break;
        }

        if node.adjusted_score > best_adjusted {
            best_adjusted = node.adjusted_score;
            best_secured = node.state.secured_points;
            best_time_remaining = node.state.time_remaining;
            best_sequence = node.sequence.clone();
        }

        if node.state.time_remaining < MIN_TIME_BUFFER {
            continue;
        }

        for (idx, action) in actions.iter().enumerate() {
            if node.sequence.contains(&idx) {
                continue;
            }

            if !action.is_available(&node.state) {
                log::trace!("  [{}] {} — not available", idx, action.label());
                continue;
            }

            let travel_time = travel_cost(node.state.position, action.position());
            let total_time = travel_time + action.base_duration();

            log::trace!(
                "  [{}] {} — travel {:.1}s + base {:.1}s = {:.1}s (remaining {:.1}s)",
                idx, action.label(), travel_time, action.base_duration(), total_time, node.state.time_remaining
            );

            if total_time > node.state.time_remaining - MIN_TIME_BUFFER {
                log::trace!("    → skipped: not enough time");
                continue;
            }

            // Simulate
            let mut new_state = node.state.clone();
            new_state.time_remaining -= total_time;
            new_state.position = action.position();
            action.apply(&mut new_state);

            // Risk-adjusted score
            let raw_score = new_state.score();
            let success_prob = action.success_probability(&node.state, opponent_pos);
            let risk = action.base_risk(&node.state);
            let adjusted_score = success_prob * raw_score - risk * RISK_WEIGHT;

            log::trace!(
                "    → raw {:.1}, prob {:.2}, risk {:.2}, adjusted {:.1}",
                raw_score, success_prob, risk, adjusted_score
            );

            let mut new_sequence = node.sequence.clone();
            new_sequence.push(idx);

            heap.push(SearchNode {
                state: new_state,
                sequence: new_sequence,
                adjusted_score,
            });
        }
    }

    log::info!(
        "Planner: explored {} nodes, best secured {} (adjusted {:.1}), sequence length {}, time remaining {:.1}s",
        nodes_explored,
        best_secured,
        best_adjusted,
        best_sequence.len(),
        best_time_remaining,
    );

    Plan {
        sequence: best_sequence,
        expected_score: best_secured,
        time_remaining: best_time_remaining,
    }
}
