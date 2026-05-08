//! Debug visualization helpers — sends debug volumes to the simulator.
//! All functions are no-ops on target (espidf).

#[cfg(not(target_os = "espidf"))]
use sim_protocol::{DebugVolume, DebugVolumeFrame, SimMsgC2S};

/// Display a corridor preflight zone in the simulator (body frame).
#[cfg(not(target_os = "espidf"))]
pub fn send_preflight_corridor(
    half_width_mm: f32,
    length_mm: f32,
    direction_rad: f32,
    stop_until_mm: f32,
) {
    let Some(sim) = sim_client::try_global() else { return };
    let mut volumes = Vec::new();

    // Stop zone (red)
    volumes.push(DebugVolume::Box {
        center_mm: [
            direction_rad.cos() * stop_until_mm / 2.0,
            direction_rad.sin() * stop_until_mm / 2.0,
            150.0,
        ],
        half_size_mm: [stop_until_mm / 2.0, half_width_mm, 150.0],
        yaw_rad: direction_rad,
        rgba: [1.0, 0.0, 0.0, 0.25],
        frame: DebugVolumeFrame::Body,
    });

    // Slow zone (orange, full corridor)
    volumes.push(DebugVolume::Box {
        center_mm: [
            direction_rad.cos() * length_mm / 2.0,
            direction_rad.sin() * length_mm / 2.0,
            150.0,
        ],
        half_size_mm: [length_mm / 2.0, half_width_mm, 150.0],
        yaw_rad: direction_rad,
        rgba: [1.0, 0.5, 0.0, 0.15],
        frame: DebugVolumeFrame::Body,
    });

    let _ = sim.send(SimMsgC2S::DebugVolumes { volumes });
}

/// Display a cylinder preflight zone in the simulator (body frame).
#[cfg(not(target_os = "espidf"))]
pub fn send_preflight_cylinder(radius_mm: f32) {
    let Some(sim) = sim_client::try_global() else { return };
    let volumes = vec![DebugVolume::Cylinder {
        center_mm: [0.0, 0.0, 0.0],
        radius_mm,
        height_mm: 300.0,
        rgba: [1.0, 0.0, 0.0, 0.25],
        frame: DebugVolumeFrame::Body,
    }];
    let _ = sim.send(SimMsgC2S::DebugVolumes { volumes });
}

/// Clear debug volumes.
#[cfg(not(target_os = "espidf"))]
pub fn clear_preflight_viz() {
    let Some(sim) = sim_client::try_global() else { return };
    let _ = sim.send(SimMsgC2S::DebugVolumes { volumes: vec![] });
}

#[cfg(target_os = "espidf")]
pub fn send_preflight_corridor(
    _half_width_mm: f32,
    _length_mm: f32,
    _direction_rad: f32,
    _stop_until_mm: f32,
) {}

#[cfg(target_os = "espidf")]
pub fn send_preflight_cylinder(_radius_mm: f32) {}

#[cfg(target_os = "espidf")]
pub fn clear_preflight_viz() {}
