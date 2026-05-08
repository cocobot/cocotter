//! `LogTail` — 3 most recent log entries floating above the footer.
//! Reference: `LogTail` in `hud_03.jsx:196-218`.
//!
//! Layout: absolute, `bottom: 40, left: 12, right: 12`, gap 1, no
//! pointer events. Older rows are dimmer; newest row is brightest. The
//! whole strip is hidden when the dock has the `Logs` pane open
//! (otherwise we'd be showing the same logs twice).
//!
//! Phase 8 scope: render-only — Phase 10 will hook live logs in. For
//! now we just sample the tail of `HudLogState.entries`.

use bevy::prelude::*;
use bevy::text::{FontWeight, LineBreak, TextLayout, TextSpan};

use super::tokens::{self, HudFonts};
use super::widgets::dock_button::DockKind;
use super::{HudLogState, HudRoot, LogLevel};

/// Index 0 = oldest (top), 2 = newest (bottom). Older rows are dimmer.
#[derive(Component, Clone, Copy)]
pub struct LogTailRow(pub usize);

#[derive(Component)]
pub struct LogTailRoot;

#[derive(Component, Clone, Copy)]
pub enum LogTailField {
    Time,
    LevelGlyph,
    Source,
    Channel,
    Message,
}

const N_ROWS: usize = 3;

pub fn setup_log_tail(
    mut commands: Commands,
    fonts: Res<HudFonts>,
    root: Single<Entity, With<HudRoot>>,
) {
    let strip = commands
        .spawn((
            LogTailRoot,
            Node {
                position_type: PositionType::Absolute,
                // Higher than the mockup's 40px to clear a 2-line
                // footer at narrow viewport widths.
                bottom: Val::Px(56.0),
                left: Val::Px(12.0),
                right: Val::Px(12.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                ..default()
            },
        ))
        .id();
    commands.entity(*root).add_child(strip);

    commands.entity(strip).with_children(|p| {
        for i in 0..N_ROWS {
            spawn_row(p, &fonts, i);
        }
    });
}

fn spawn_row(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, idx: usize) {
    // Mockup: bg alpha = 0.45 + idx*0.20 (newest brightest), but
    // calibrated to our opaque panel — keep the gradient roughly the
    // same shape but with a softer ramp.
    let bg_alpha = 0.30 + idx as f32 * 0.18; // 0.30 / 0.48 / 0.66
    let row = LogTailRow(idx);
    parent
        .spawn((
            row,
            Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(8.0),
                padding: UiRect::axes(Val::Px(8.0), Val::Px(2.0)),
                align_items: AlignItems::Start,
                border_radius: BorderRadius::all(Val::Px(4.0)),
                max_width: Val::Px(760.0),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0314, 0.0431, 0.0353, bg_alpha)),
        ))
        .with_children(|r| {
            // T+X.Xs (fixed 56) — also tag with `LogTailRow(idx)` so
            // the update query `(&LogTailRow, &LogTailField, ...)`
            // matches on the same entity.
            r.spawn((
                row,
                LogTailField::Time,
                Text::new(""),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
                Node {
                    width: Val::Px(56.0),
                    flex_shrink: 0.0,
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
            ));
            // level glyph (14)
            r.spawn((
                row,
                LogTailField::LevelGlyph,
                Text::new(""),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 11.5,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(tokens::TEXT),
                Node {
                    width: Val::Px(14.0),
                    flex_shrink: 0.0,
                    margin: UiRect::right(Val::Px(8.0)),
                    justify_content: JustifyContent::Center,
                    ..default()
                },
            ));
            // src (64)
            r.spawn((
                row,
                LogTailField::Source,
                Text::new(""),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_DIM),
                Node {
                    width: Val::Px(64.0),
                    flex_shrink: 0.0,
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
            ));
            // ch (56)
            r.spawn((
                row,
                LogTailField::Channel,
                Text::new(""),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::ACCENT),
                Node {
                    width: Val::Px(56.0),
                    flex_shrink: 0.0,
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
            ));
            // msg (flex)
            r.spawn((
                row,
                LogTailField::Message,
                Text::new(""),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 11.0,
                    ..default()
                },
                TextColor(tokens::TEXT),
                TextLayout::new_with_linebreak(LineBreak::NoWrap),
                Node {
                    flex_grow: 1.0,
                    flex_basis: Val::Px(0.0),
                    flex_shrink: 1.0,
                    overflow: Overflow::clip_x(),
                    ..default()
                },
            ));
            let _ = TextSpan::new(""); // imports stable
        });
}

/// Update tail content + visibility every frame. Hidden when
/// dock == Logs to avoid duplicate display.
#[allow(clippy::too_many_arguments)]
pub fn update_log_tail(
    log_state: Res<HudLogState>,
    active: Res<super::dock::ActiveDock>,
    mut q_root: Query<&mut Node, (With<LogTailRoot>, Without<LogTailRow>)>,
    mut q_row_node: Query<(&LogTailRow, &mut Node), (Without<LogTailRoot>, Without<LogTailField>)>,
    mut q_fields: Query<(&LogTailRow, &LogTailField, &mut Text, &mut TextColor)>,
) {
    // Whole-strip visibility.
    let hide = active.0 == Some(DockKind::Logs);
    if let Ok(mut n) = q_root.single_mut() {
        n.display = if hide {
            Display::None
        } else {
            Display::Flex
        };
    }
    if hide {
        return;
    }

    let total = log_state.entries.len();
    for (row, mut node) in &mut q_row_node {
        // Row idx: 0 = oldest, 2 = newest. Latest log goes to row N_ROWS-1.
        let entry_idx = total
            .checked_sub(N_ROWS - row.0)
            .filter(|_| total >= N_ROWS - row.0);
        node.display = if entry_idx.is_some() {
            Display::Flex
        } else {
            Display::None
        };
    }

    for (row, field, mut text, mut color) in &mut q_fields {
        let Some(entry_idx) = total
            .checked_sub(N_ROWS - row.0)
            .filter(|_| total >= N_ROWS - row.0)
        else {
            continue;
        };
        let Some(entry) = log_state.entries.get(entry_idx) else {
            continue;
        };
        // Older rows fade slightly (mockup `opacity: 0.55 + i*0.22`).
        let row_alpha = 0.55 + row.0 as f32 * 0.22;

        let (s, c) = match field {
            LogTailField::Time => {
                let sign = if entry.t < 0.0 { "-" } else { "+" };
                (
                    format!("T{sign}{:.1}s", entry.t.abs()),
                    tokens::TEXT_MUTE.with_alpha(row_alpha),
                )
            }
            LogTailField::LevelGlyph => (
                entry.level.glyph().to_string(),
                entry.level.color().with_alpha(row_alpha),
            ),
            LogTailField::Source => (
                entry.src.to_string(),
                tokens::TEXT_DIM.with_alpha(row_alpha),
            ),
            LogTailField::Channel => (
                entry.channel.to_string(),
                tokens::ACCENT.with_alpha(row_alpha),
            ),
            LogTailField::Message => {
                // Debug rows are dimmed extra.
                let extra = if matches!(entry.level, LogLevel::Debug) {
                    0.65
                } else {
                    1.0
                };
                (
                    entry.msg.to_string(),
                    tokens::TEXT.with_alpha(row_alpha * extra),
                )
            }
        };
        if text.0 != s {
            text.0 = s;
        }
        color.0 = c;
    }
}
