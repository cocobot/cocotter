//! `hud-btn` — a small uppercase button used in the footer ("? full
//! cheatsheet"), the LogsPanel header, and dock panel close. Reference:
//! `.hud-btn` in `hud_02.jsx:78-85`.
//!
//! Spec: transparent background, 1px BORDER, padding `3px 7px`, radius
//! 4px, mono 10.5px, color text_dim. The border uses `Outline` (rendered
//! outside the node) to keep thickness uniform on all sides — Bevy 0.18
//! renders `Node.border` slightly thicker on bottom/right with rounded
//! corners.

use bevy::prelude::*;

use super::super::tokens;

/// Returns the bundle for an `hud-btn` chrome (without inner content).
/// Caller spawns the bundle then attaches children for the button label
/// (text and/or kbd badges).
pub fn bundle() -> impl Bundle {
    (
        Node {
            border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_BTN)),
            padding: UiRect::axes(Val::Px(7.0), Val::Px(3.0)),
            align_items: AlignItems::Center,
            column_gap: Val::Px(6.0),
            ..default()
        },
        BackgroundColor(Color::NONE),
        Outline::new(Val::Px(1.0), Val::Px(0.0), tokens::BORDER),
    )
}
