//! Left dock — vertical bar of 4 icon buttons (`logs`, `watch`,
//! `robots`, `help`) plus a dock panel that opens to their right when
//! one is active. References:
//!   - dock bar `hud_03.jsx:280-287` (live variant `04.jsx:34-42`)
//!   - dock panel header `hud_03.jsx:289-306`
//!
//! The panel body is intentionally a placeholder for now; phases 5-7
//! will fill it with `LogsPanel` / `WatchPanel` / `RobotsBySide`.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_panorbit_camera::PanOrbitCamera;

use super::tokens::{self, HudFonts};
use super::{logs_panel, robots_panel, watch_panel};
use super::widgets::card;
use super::widgets::dock_button::{self, DockButton, DockKind};
use super::widgets::hud_btn;
use super::widgets::kbd;
use super::{HudKvState, HudLogState, HudMockData, HudRobotsState, HudRoot};

/// Resource: which dock pane is currently active. `None` = closed.
/// Initialised to `Some(Logs)` to match `useState("logs")` in
/// `04.jsx:105`.
#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActiveDock(pub Option<DockKind>);

impl Default for ActiveDock {
    fn default() -> Self {
        // Closed by default — the user opens it explicitly with
        // 1/2/3/? when they want to inspect things.
        Self(None)
    }
}

#[derive(Component)]
pub struct DockBar;

#[derive(Component)]
pub struct DockPanel;

#[derive(Component)]
pub struct DockPanelTitle;

/// Marker on each per-tab body wrapper so `update_dock_visuals` can
/// toggle Display::Flex/None as the active dock changes.
#[derive(Component, Clone, Copy)]
pub struct DockPaneBody(pub DockKind);

/// Marker on the `[esc] close` button in the dock panel header.
#[derive(Component)]
pub struct DockCloseButton;

pub fn setup_dock(
    mut commands: Commands,
    fonts: Res<HudFonts>,
    asset_server: Res<AssetServer>,
    log_state: Res<HudLogState>,
    kv_state: Res<HudKvState>,
    robots_state: Res<HudRobotsState>,
    mock: Res<HudMockData>,
    root: Single<Entity, With<HudRoot>>,
) {
    // ── Dock bar ────────────────────────────────────────────────────
    let bar = commands
        .spawn((
            DockBar,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(130.0),
                left: Val::Px(12.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(5.0)),
                row_gap: Val::Px(3.0),
                border_radius: card::radius_panel(),
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
        ))
        .id();
    commands.entity(*root).add_child(bar);

    commands.entity(bar).with_children(|b| {
        dock_button::spawn(b, &fonts, &asset_server, DockKind::Logs, "1");
        dock_button::spawn(b, &fonts, &asset_server, DockKind::Watch, "2");
        dock_button::spawn(b, &fonts, &asset_server, DockKind::Robots, "3");
        // Divider 1px, margin 3px 6px (mockup spec).
        b.spawn((
            Node {
                height: Val::Px(1.0),
                margin: UiRect {
                    top: Val::Px(3.0),
                    bottom: Val::Px(3.0),
                    left: Val::Px(6.0),
                    right: Val::Px(6.0),
                },
                ..default()
            },
            BackgroundColor(tokens::BORDER),
        ));
        dock_button::spawn(b, &fonts, &asset_server, DockKind::Help, "?");
    });

    // ── Dock panel (container, body placeholder for now) ───────────
    let panel = commands
        .spawn((
            DockPanel,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(130.0),
                left: Val::Px(70.0),
                width: Val::Px(400.0),
                max_height: Val::Px(420.0),
                flex_direction: FlexDirection::Column,
                border_radius: card::radius_panel(),
                overflow: Overflow::clip(),
                display: Display::None, // toggled by `ActiveDock`
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
        ))
        .id();
    commands.entity(*root).add_child(panel);

    commands.entity(panel).with_children(|p| {
        // Header: section title (left) + esc close button (right).
        p.spawn((
            Node {
                padding: UiRect::axes(Val::Px(10.0), Val::Px(7.0)),
                justify_content: JustifyContent::SpaceBetween,
                align_items: AlignItems::Center,
                border: UiRect {
                    bottom: Val::Px(1.0),
                    ..default()
                },
                ..default()
            },
            BorderColor::all(tokens::BORDER),
        ))
        .with_children(|hdr| {
            // Left: section title with leading dot accent.
            hdr.spawn(Node {
                column_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                ..default()
            })
            .with_children(|left| {
                // Dot 6×6 accent.
                left.spawn((
                    Node {
                        width: Val::Px(6.0),
                        height: Val::Px(6.0),
                        border_radius: BorderRadius::all(Val::Px(50.0)),
                        ..default()
                    },
                    BackgroundColor(tokens::ACCENT),
                ));
                left.spawn((
                    DockPanelTitle,
                    Text::new("logs"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
            // Right: [esc] close hud-btn — clickable, closes the dock.
            hdr.spawn((DockCloseButton, Button, hud_btn::bundle()))
                .with_children(|btn| {
                    kbd::spawn(btn, &fonts, "esc", true);
                    btn.spawn((
                        Text::new("close"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
        });
        // Body — one wrapper per tab, only the active one displayed.
        // Logs body fully implemented in Phase 5; watch/robots are
        // placeholders pending Phases 6/7.
        p.spawn((
            DockPaneBody(DockKind::Logs),
            Node {
                flex_direction: FlexDirection::Column,
                flex_grow: 1.0,
                // Override the `min-height: auto` default so the body
                // can be smaller than its content — required for the
                // inner LogsScrollArea to actually scroll.
                min_height: Val::Px(0.0),
                ..default()
            },
        ))
        .with_children(|logs_body| {
            logs_panel::populate(logs_body, &fonts, &log_state);
        });
        p.spawn((
            DockPaneBody(DockKind::Watch),
            Node {
                flex_direction: FlexDirection::Column,
                flex_grow: 1.0,
                min_height: Val::Px(0.0),
                display: Display::None,
                ..default()
            },
        ))
        .with_children(|w| {
            watch_panel::populate(w, &fonts, &kv_state);
        });
        p.spawn((
            DockPaneBody(DockKind::Robots),
            Node {
                flex_direction: FlexDirection::Column,
                flex_grow: 1.0,
                min_height: Val::Px(0.0),
                display: Display::None,
                ..default()
            },
        ))
        .with_children(|r| {
            // Inner wrapper marked `RobotsBodyRoot` — Phase 10c's
            // refresh system despawns this entity's children and
            // rebuilds them when the live robot list changes.
            r.spawn((
                robots_panel::RobotsBodyRoot,
                Node {
                    flex_direction: FlexDirection::Column,
                    flex_grow: 1.0,
                    min_height: Val::Px(0.0),
                    ..default()
                },
            ))
            .with_children(|inner| {
                robots_panel::populate(inner, &fonts, &robots_state, mock.our_side);
            });
        });
    });

    commands.insert_resource(ActiveDock::default());
}

// ─── Click + keyboard handling ──────────────────────────────────────

pub fn dock_button_input(
    mut q_btn: Query<(&Interaction, &DockKind), (Changed<Interaction>, With<DockButton>)>,
    q_close: Query<&Interaction, (Changed<Interaction>, With<DockCloseButton>)>,
    mut active: ResMut<ActiveDock>,
) {
    for (interaction, kind) in &mut q_btn {
        if *interaction == Interaction::Pressed {
            active.0 = match active.0 {
                Some(prev) if prev == *kind => None,
                _ => Some(*kind),
            };
        }
    }
    for i in &q_close {
        if *i == Interaction::Pressed {
            active.0 = None;
        }
    }
}

/// Set the window cursor to `Pointer` when hovering any UI button (any
/// entity with `Interaction::Hovered`), `Default` otherwise. Lightweight
/// alternative to bevy_feathers' picking-based cursor system.
pub fn update_cursor_icon(
    mut commands: Commands,
    q_hover: Query<&Interaction>,
    q_window: Query<Entity, With<PrimaryWindow>>,
) {
    let any_hovered = q_hover
        .iter()
        .any(|i| matches!(i, Interaction::Hovered | Interaction::Pressed));
    let icon: bevy::window::CursorIcon = if any_hovered {
        bevy::window::SystemCursorIcon::Pointer.into()
    } else {
        bevy::window::SystemCursorIcon::Default.into()
    };
    for e in &q_window {
        commands.entity(e).insert(icon.clone());
    }
}

/// Disable the 3D camera while the cursor is anywhere over the HUD
/// (whole left ~480px column under the top strip). This is wider than
/// just the dock panel but it covers the dock bar + dock panel + part
/// of the header, which is fine — there's nothing 3D-relevant under
/// those zones anyway.
pub fn camera_mute_when_over_dock(
    windows: Query<&Window, With<PrimaryWindow>>,
    active: Res<ActiveDock>,
    mut q_cam: Query<&mut PanOrbitCamera>,
) {
    let Ok(win) = windows.single() else { return };
    let Some(cur) = win.cursor_position() else {
        for mut c in &mut q_cam {
            c.enabled = true;
        }
        return;
    };
    // Hardcoded HUD-zone rect. Dock bar lives at top:130, left:12,
    // width≈54, height≈210 (4 buttons × 44 + gaps + padding). When a
    // dock is active, the panel adds left:70..470, top:130..550.
    let dock_open = active.0.is_some();
    let over_bar = cur.x >= 6.0 && cur.x <= 64.0 && cur.y >= 124.0 && cur.y <= 350.0;
    let over_panel = dock_open
        && cur.x >= 64.0
        && cur.x <= 478.0
        && cur.y >= 124.0
        && cur.y <= 558.0;
    let over = over_bar || over_panel;
    for mut c in &mut q_cam {
        c.enabled = !over;
    }
}

pub fn dock_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    focus: Res<super::LogsFilterFocus>,
    mut active: ResMut<ActiveDock>,
) {
    if focus.0 {
        return;
    }
    let toggle = |a: &mut ActiveDock, k: DockKind| {
        a.0 = match a.0 {
            Some(prev) if prev == k => None,
            _ => Some(k),
        };
    };
    if keys.just_pressed(KeyCode::Digit1) {
        toggle(&mut active, DockKind::Logs);
    }
    if keys.just_pressed(KeyCode::Digit2) {
        toggle(&mut active, DockKind::Watch);
    }
    if keys.just_pressed(KeyCode::Digit3) {
        toggle(&mut active, DockKind::Robots);
    }
    if keys.just_pressed(KeyCode::Escape) {
        active.0 = None;
    }
}

// ─── Dock visibility + button on/off ───────────────────────────────

pub fn update_dock_visuals(
    active: Res<ActiveDock>,
    mut q_panel: Query<&mut Node, (With<DockPanel>, Without<DockPaneBody>)>,
    mut q_title: Query<&mut Text, With<DockPanelTitle>>,
    mut q_btn: Query<(&DockKind, &mut BackgroundColor), With<DockButton>>,
    mut q_body: Query<(&DockPaneBody, &mut Node), Without<DockPanel>>,
) {
    if let Ok(mut n) = q_panel.single_mut() {
        n.display = if active.0.is_some() {
            Display::Flex
        } else {
            Display::None
        };
    }
    if let Ok(mut t) = q_title.single_mut() {
        let label = match active.0 {
            Some(DockKind::Logs) => "logs",
            Some(DockKind::Watch) => "watch",
            Some(DockKind::Robots) => "robots",
            Some(DockKind::Help) => "help",
            None => "",
        };
        if t.0 != label {
            t.0 = label.into();
        }
    }
    for (kind, mut bg) in &mut q_btn {
        let is_on = active.0 == Some(*kind);
        bg.0 = if is_on {
            Color::srgba(0.4275, 0.7490, 0.2275, 0.10)
        } else {
            Color::NONE
        };
    }
    for (pane, mut node) in &mut q_body {
        node.display = if Some(pane.0) == active.0 {
            Display::Flex
        } else {
            Display::None
        };
    }
}
