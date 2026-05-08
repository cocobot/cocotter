//! `hud-panel` chrome — the standard card surface used by the top
//! strip, dock bar, dock panel, footer (variant), cheatsheet modal.
//!
//! Reference: `.hud-panel` in `hud_02.jsx` (background + 1px border +
//! 6px radius; the original `backdrop-filter: blur(8px)` is dropped, we
//! compensate with a slightly more opaque background — see
//! `tokens::PANEL_BG`).
//!
//! In Bevy 0.18 `BorderRadius` is a field of `Node`, not a separate
//! Component, so callers fold the radius value into their own `Node`
//! init (`border_radius: card::radius_panel()`) and pair it with the
//! `(BackgroundColor, BorderColor)` returned here.

use bevy::prelude::*;

use super::super::tokens;

pub fn chrome() -> (BackgroundColor, BorderColor) {
    (
        BackgroundColor(tokens::PANEL_BG),
        BorderColor::all(tokens::BORDER),
    )
}

/// Bevy 0.18's `Node.border` + rounded corners has a renderer artifact
/// that makes bottom/right edges render slightly thicker than top/left
/// at 1px thickness. `Outline` (rendered *outside* the node, doesn't
/// take layout space) is uniform thickness and follows `border_radius`,
/// so we use it for the card chrome instead.
pub fn outline() -> Outline {
    Outline::new(Val::Px(1.0), Val::Px(0.0), tokens::BORDER)
}

pub fn radius_panel() -> BorderRadius {
    BorderRadius::all(Val::Px(tokens::RADIUS_PANEL))
}

#[allow(dead_code)]
pub fn radius_pill() -> BorderRadius {
    BorderRadius::all(Val::Px(tokens::RADIUS_PILL))
}
