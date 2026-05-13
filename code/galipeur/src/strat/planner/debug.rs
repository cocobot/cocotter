use board_sabotter::SabotterBoard;

use super::{Action, Plan};

/// Send debug volumes to the simulator showing the current plan.
///
/// - Current action: green cylinder + label
/// - Future actions: blue cylinders + labels
/// - No-op on ESP32 (handled by debug_viz internals).
#[cfg(not(target_os = "espidf"))]
pub fn visualize_plan<B: SabotterBoard>(
    actions: &[Box<dyn Action<B>>],
    plan: &Plan,
    current_step: usize,
) {
    use sim_protocol::{DebugVolume, DebugVolumeFrame, SimMsgC2S};

    let Some(sim) = sim_client::try_global() else {
        return;
    };

    let mut volumes = Vec::new();

    for (i, &action_id) in plan.sequence.iter().enumerate() {
        let action = &actions[action_id];
        let pos = action.position();

        let (rgba, radius) = if i == current_step {
            ([0.0, 1.0, 0.0, 0.4], 60.0)
        } else if i > current_step {
            ([0.0, 0.4, 1.0, 0.25], 40.0)
        } else {
            continue;
        };

        volumes.push(DebugVolume::Cylinder {
            center_mm: [pos.x, pos.y, 0.0],
            radius_mm: radius,
            height_mm: 200.0,
            rgba,
            frame: DebugVolumeFrame::Table,
        });

        volumes.push(DebugVolume::Text {
            position_mm: [pos.x, pos.y, 250.0],
            text: format!("#{} {}", i, action.label()),
            size_mm: 30.0,
            rgba: [1.0, 1.0, 1.0, 0.9],
            frame: DebugVolumeFrame::Table,
        });
    }

    let _ = sim.send(SimMsgC2S::DebugVolumes { volumes });
}

#[cfg(target_os = "espidf")]
pub fn visualize_plan<B: SabotterBoard>(
    _actions: &[Box<dyn Action<B>>],
    _plan: &Plan,
    _current_step: usize,
) {
}

/// Clear plan debug volumes.
pub fn clear_viz() {
    board_sabotter::debug_viz::clear_preflight_viz();
}
