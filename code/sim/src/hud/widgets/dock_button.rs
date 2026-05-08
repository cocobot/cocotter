//! `DockButton` — 44×44 icon button used in the left dock. Reference:
//! `DockButton` in `hud_03.jsx:77-124`.
//!
//! Each button has:
//!  - SVG icon centered (we use pre-rasterized PNGs in
//!    `sim/assets/icons/` — Bevy doesn't render SVG natively).
//!  - Optional count chip in top-right (mockup `count` field).
//!  - Kbd badge in bottom-right (mockup `kbd` field).
//!
//! The button itself has 3 visual states (default / hover / on)
//! mirrored from the mockup's inline-style switching.

use bevy::prelude::*;
use bevy::text::FontWeight;

use super::super::tokens::{self, HudFonts};

// Note: the mockup spec'd a top-right count chip per button but the
// designer (and the user) ultimately decided it was clutter. We don't
// spawn it.

/// Identifies which dock pane a button activates.
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
pub enum DockKind {
    Logs,
    Watch,
    Robots,
    Help,
}

impl DockKind {
    pub fn icon_asset(self) -> &'static str {
        match self {
            DockKind::Logs => "icons/logs.png",
            DockKind::Watch => "icons/watch.png",
            DockKind::Robots => "icons/robots.png",
            DockKind::Help => "icons/help.png",
        }
    }
}

/// Marker on the button root entity. Pairs with `DockKind`.
#[derive(Component)]
pub struct DockButton;

/// Spawn a single dock button. `kind` is attached as a Component so the
/// dock state machine can dispatch clicks; `kbd_label` is the static
/// keyboard hint shown in the bottom-right badge.
pub fn spawn(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    asset_server: &AssetServer,
    kind: DockKind,
    kbd_label: &str,
) {
    let icon: Handle<Image> = asset_server.load(kind.icon_asset());
    parent
        .spawn((
            DockButton,
            kind,
            Button,
            Node {
                width: Val::Px(44.0),
                height: Val::Px(44.0),
                border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_PANEL)),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                position_type: PositionType::Relative,
                ..default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|btn| {
            // Icon (PNG rasterized from mockup SVG, white-tinted).
            btn.spawn((
                Node {
                    width: Val::Px(20.0),
                    height: Val::Px(20.0),
                    ..default()
                },
                ImageNode {
                    image: icon,
                    color: tokens::TEXT,
                    ..default()
                },
            ));
            // Kbd badge bottom-right (`-2, -2`).
            btn.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(-2.0),
                    right: Val::Px(-2.0),
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(1.0)),
                    border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_KBD)),
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::Center,
                    ..default()
                },
                BackgroundColor(Color::srgb(0.0470, 0.0627, 0.0627)), // #0c1010
                Outline::new(Val::Px(1.0), Val::Px(0.0), tokens::BORDER),
            ))
            .with_children(|b| {
                b.spawn((
                    Text::new(kbd_label),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 9.0,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ));
            });
        });
}
