//! `WatchPanel` — populates the dock panel body when active dock = Watch.
//! Reference: `WatchPanel` in `hud_03.jsx:127-153`.
//!
//! Layout: scrollable column, KV entries grouped by source. Each
//! group has a small header (dot + uppercase source label + count),
//! then one row per entry: key text_dim left, value accent_hot right.

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use bevy::text::FontWeight;

use super::tokens::{self, HudFonts};
use super::HudKvState;

/// Source order matches the LogsPanel filter (galipeur / pami / sim
/// — same canonical names produced by `log_capture::parse_target`).
const SRC_ORDER: &[&str] = &["galipeur", "pami", "sim"];

#[derive(Component)]
pub struct WatchScrollArea;

pub fn populate(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    kv_state: &HudKvState,
) {
    parent
        .spawn((
            WatchScrollArea,
            Node {
                flex_direction: FlexDirection::Column,
                padding: UiRect::axes(Val::Px(6.0), Val::Px(8.0)),
                row_gap: Val::Px(12.0),
                overflow: Overflow::scroll_y(),
                flex_grow: 1.0,
                min_height: Val::Px(0.0),
                ..default()
            },
        ))
        .with_children(|body| {
            for src in SRC_ORDER {
                let group: Vec<&super::HudKv> = kv_state
                    .entries
                    .iter()
                    .filter(|e| e.src == *src)
                    .collect();
                if group.is_empty() {
                    continue;
                }
                spawn_group(body, fonts, src, &group);
            }
        });
}

fn spawn_group(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    src: &str,
    entries: &[&super::HudKv],
) {
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(2.0),
            ..default()
        })
        .with_children(|grp| {
            // Header line: dot + label + count.
            grp.spawn(Node {
                column_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                padding: UiRect::new(Val::Px(4.0), Val::Px(0.0), Val::Px(0.0), Val::Px(4.0)),
                ..default()
            })
            .with_children(|hdr| {
                hdr.spawn((
                    Node {
                        width: Val::Px(5.0),
                        height: Val::Px(5.0),
                        border_radius: BorderRadius::all(Val::Px(50.0)),
                        ..default()
                    },
                    BackgroundColor(tokens::ACCENT),
                ));
                hdr.spawn((
                    Text::new(src.to_uppercase()),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 10.0,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(tokens::TEXT),
                ));
                hdr.spawn((
                    Text::new(format!("· {}", entries.len())),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ));
            });
            // KV rows.
            for e in entries {
                grp.spawn((
                    Node {
                        padding: UiRect::axes(Val::Px(8.0), Val::Px(3.0)),
                        column_gap: Val::Px(8.0),
                        justify_content: JustifyContent::SpaceBetween,
                        align_items: AlignItems::Center,
                        border_radius: BorderRadius::all(Val::Px(3.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.015)),
                ))
                .with_children(|row| {
                    // Key
                    row.spawn((
                        Text::new(e.key),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 11.0,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                    // Value (accent_hot, semibold, tabular)
                    row.spawn((
                        Text::new(e.value),
                        TextFont {
                            font: fonts.mono_medium.clone(),
                            font_size: 11.0,
                            weight: FontWeight::SEMIBOLD,
                            ..default()
                        },
                        TextColor(tokens::ACCENT_HOT),
                    ));
                });
            }
        });
}

/// Mouse-wheel scroll for the watch panel. Gated on `ActiveDock` so
/// scrolling never silently moves the (hidden) other panes.
pub fn handle_scroll_wheel(
    mut wheel: MessageReader<MouseWheel>,
    active: Res<super::dock::ActiveDock>,
    mut q: Query<&mut ScrollPosition, With<WatchScrollArea>>,
) {
    if active.0 != Some(super::widgets::dock_button::DockKind::Watch) {
        wheel.clear();
        return;
    }
    let mut delta_y = 0.0_f32;
    for ev in wheel.read() {
        let step = match ev.unit {
            MouseScrollUnit::Line => ev.y * 30.0,
            MouseScrollUnit::Pixel => ev.y,
        };
        delta_y -= step;
    }
    if delta_y.abs() < f32::EPSILON {
        return;
    }
    for mut sp in &mut q {
        let mut v = sp.0;
        v.y = (v.y + delta_y).max(0.0);
        sp.0 = v;
    }
}
