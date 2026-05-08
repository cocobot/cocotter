//! Style helpers — small constructors for repeated Bevy UI bundles
//! (cards, borders, radii). Keeps the per-widget code focused on
//! layout instead of re-stating the panel chrome on every spawn.

#![allow(dead_code)]

use bevy::prelude::*;

use super::tokens;

/// Standard `hud-panel` chrome: background + border color + 1px border
/// (caller sets it on `Node.border`) + 6px radius.
pub fn panel_bg() -> BackgroundColor {
    BackgroundColor(tokens::PANEL_BG)
}

pub fn panel_border() -> BorderColor {
    BorderColor::all(tokens::BORDER)
}

pub fn radius_panel() -> BorderRadius {
    BorderRadius::all(Val::Px(tokens::RADIUS_PANEL))
}
