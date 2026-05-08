//! `LogsPanel` — populates the dock panel body when active dock = Logs.
//! Reference: `LogsPanel` in `hud_02.jsx:229-287`.
//!
//! Layout (top to bottom):
//!  1. Header line 1 — section title "logs", count "X/Y", spacer,
//!     debug/info/warn/error toggle buttons, vertical divider, filter
//!     input placeholder, pause/clear buttons.
//!  2. Header line 2 — "CHANNELS" label + 7 channel toggle buttons.
//!  3. Body — grid rows `t / lvl / src / ch / msg` for each `LogEntry`.
//!
//! Phase 5 scope: filter buttons (level + channel) are functional;
//! pause / clear / search input are placeholders (visual state only —
//! Phase 10 will wire them up to live data).

use bevy::ecs::message::MessageReader;
use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use bevy::text::{FontWeight, LineBreak, TextLayout};

use super::tokens::{self, HudFonts};
use super::widgets::card;
use super::{
    HudLogState, LogEntry, LogLevel, LogsFilterFocus, LOG_LEVELS, LOG_SOURCES,
};

#[derive(Component)]
pub struct LogLevelToggle(pub LogLevel);

/// Phase 10d-rework: sources (sim/galipeur/pami), not channels.
#[derive(Component)]
pub struct LogSourceToggle(pub usize);

/// Marker on a log row entity. Holds level + source index + entry
/// index so the filter system can hide/show without re-walking
/// children.
#[derive(Component, Clone, Copy)]
pub struct LogRow {
    pub level: LogLevel,
    pub source_idx: usize,
    pub entry_idx: usize,
}

#[derive(Component)]
pub struct LogsCountText;

/// Marker on the scrollable log body node — used by the wheel-scroll
/// system to find the right `ScrollPosition` to update.
#[derive(Component)]
pub struct LogsScrollArea;

/// Marker on the filter-input wrapper node so click-to-focus can target it.
#[derive(Component)]
pub struct LogsFilterInput;

/// Marker on the inner Text of the filter input — for value updates.
#[derive(Component)]
pub struct LogsFilterInputText;

/// Build the LogsPanel content. Called from `dock::setup_dock` with
/// the dock-body entity as the parent.
pub fn populate(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, log_state: &HudLogState) {
    // ── Header row 1 ───────────────────────────────────────────────
    parent
        .spawn((
            Node {
                padding: UiRect::axes(Val::Px(12.0), Val::Px(10.0)),
                column_gap: Val::Px(10.0),
                row_gap: Val::Px(8.0),
                align_items: AlignItems::Center,
                flex_wrap: FlexWrap::Wrap,
                border: UiRect {
                    bottom: Val::Px(1.0),
                    ..default()
                },
                ..default()
            },
            BorderColor::all(tokens::BORDER),
        ))
        .with_children(|hdr| {
            // Section title with leading dot accent.
            hdr.spawn(Node {
                column_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                ..default()
            })
            .with_children(|t| {
                t.spawn((
                    Node {
                        width: Val::Px(6.0),
                        height: Val::Px(6.0),
                        border_radius: BorderRadius::all(Val::Px(50.0)),
                        ..default()
                    },
                    BackgroundColor(tokens::ACCENT),
                ));
                t.spawn((
                    Text::new("logs"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
            // Count "X/Y".
            hdr.spawn((
                LogsCountText,
                Text::new(format!(
                    "{}/{}",
                    log_state.entries.len(),
                    log_state.entries.len()
                )),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
            ));
            // Spacer.
            hdr.spawn(Node {
                flex_grow: 1.0,
                ..default()
            });
            // 4 level toggle buttons.
            for level in LOG_LEVELS {
                spawn_level_button(hdr, fonts, level, log_state.levels[level.idx()]);
            }
            // Right-aligned group: divider + filter + pause + clear,
            // wrapped together so when the row overflows they all wrap
            // to a new line and stay grouped on the right.
            hdr.spawn(Node {
                column_gap: Val::Px(8.0),
                align_items: AlignItems::Center,
                margin: UiRect::left(Val::Auto),
                ..default()
            })
            .with_children(|right| {
                // Divider.
                right.spawn((
                    Node {
                        width: Val::Px(1.0),
                        height: Val::Px(16.0),
                        margin: UiRect::axes(Val::Px(2.0), Val::Px(0.0)),
                        ..default()
                    },
                    BackgroundColor(tokens::BORDER),
                ));
                // Search input — focusable, displays current filter_query.
                right
                    .spawn((
                        LogsFilterInput,
                        Button,
                        Node {
                            width: Val::Px(110.0),
                            padding: UiRect::axes(Val::Px(7.0), Val::Px(3.0)),
                            border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_BTN)),
                            align_items: AlignItems::Center,
                            ..default()
                        },
                        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.30)),
                        Outline::new(Val::Px(1.0), Val::Px(0.0), tokens::BORDER),
                    ))
                    .with_children(|inp| {
                        inp.spawn((
                            LogsFilterInputText,
                            Text::new("filter\u{2026}"),
                            TextFont {
                                font: fonts.mono.clone(),
                                font_size: 11.0,
                                ..default()
                            },
                            TextColor(tokens::TEXT_MUTE),
                        ));
                    });
                // Pause + clear (visual placeholders).
                spawn_basic_btn(right, fonts, "|| pause");
                spawn_basic_btn(right, fonts, "clear");
            });
        });

    // ── Header row 2 ───────────────────────────────────────────────
    parent
        .spawn((
            Node {
                padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                column_gap: Val::Px(6.0),
                row_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                flex_wrap: FlexWrap::Wrap,
                border: UiRect {
                    bottom: Val::Px(1.0),
                    ..default()
                },
                ..default()
            },
            BorderColor::all(tokens::BORDER),
        ))
        .with_children(|hdr2| {
            hdr2.spawn((
                Text::new("SOURCES"),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 9.5,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
                Node {
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
            ));
            for (idx, src) in LOG_SOURCES.iter().enumerate() {
                spawn_source_button(hdr2, fonts, idx, src, log_state.sources[idx]);
            }
        });

    // ── Body — log rows (scrollable) ───────────────────────────────
    parent
        .spawn((
            LogsScrollArea,
            Node {
                flex_direction: FlexDirection::Column,
                padding: UiRect::axes(Val::Px(0.0), Val::Px(8.0)),
                row_gap: Val::Px(2.0),
                overflow: Overflow::scroll_y(),
                flex_grow: 1.0,
                // `min_height: 0` lets the scroll area shrink below its
                // content height — without this it grows to fit and
                // there's nothing to scroll.
                min_height: Val::Px(0.0),
                ..default()
            },
        ))
        .with_children(|body| {
            for (idx, entry) in log_state.entries.iter().enumerate() {
                spawn_log_row(body, fonts, idx, entry);
            }
        });
}

fn spawn_level_button(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    level: LogLevel,
    on: bool,
) {
    let (text_color, border_color, bg) = level_btn_visuals(level, on);
    parent
        .spawn((
            LogLevelToggle(level),
            Button,
            Node {
                padding: UiRect::axes(Val::Px(9.0), Val::Px(4.0)),
                border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_BTN)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(bg),
            Outline::new(Val::Px(1.0), Val::Px(0.0), border_color),
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(level.label()),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(text_color),
            ));
        });
}

fn spawn_source_button(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    idx: usize,
    label: &str,
    on: bool,
) {
    let (text_color, border_color, bg) = source_btn_visuals(on);
    let prefix = if on { "\u{25CF} " } else { "\u{25CB} " };
    parent
        .spawn((
            LogSourceToggle(idx),
            Button,
            Node {
                padding: UiRect::axes(Val::Px(9.0), Val::Px(5.0)),
                border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_BTN)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(bg),
            Outline::new(Val::Px(1.0), Val::Px(0.0), border_color),
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(format!("{prefix}{label}")),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.0,
                    ..default()
                },
                TextColor(text_color),
            ));
        });
}

/// Visual presets for level buttons. `on` = filter enabled (clearly
/// colored bg + bordered + colored text), `off` = dimmed with subtle
/// text + border. Background bumped to 0.20 so the dim debug color
/// still reads clearly when on.
fn level_btn_visuals(level: LogLevel, on: bool) -> (Color, Color, Color) {
    if on {
        let c = level.color();
        (c, c.with_alpha(0.70), c.with_alpha(0.20))
    } else {
        (tokens::TEXT_MUTE, tokens::BORDER, Color::NONE)
    }
}

fn source_btn_visuals(on: bool) -> (Color, Color, Color) {
    if on {
        (
            tokens::ACCENT_HOT,
            tokens::BORDER_HI,
            tokens::ACCENT.with_alpha(0.07),
        )
    } else {
        (tokens::TEXT_MUTE, tokens::BORDER, Color::NONE)
    }
}

fn spawn_basic_btn(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, label: &str) {
    parent
        .spawn(super::widgets::hud_btn::bundle())
        .with_children(|p| {
            p.spawn((
                Text::new(label),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_DIM),
            ));
        });
}

fn spawn_log_row(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    entry_idx: usize,
    entry: &LogEntry,
) {
    let source_idx = LOG_SOURCES
        .iter()
        .position(|s| *s == entry.src.as_str())
        .unwrap_or(0);
    let level_color = entry.level.color();
    let row_alpha = if matches!(entry.level, LogLevel::Debug) {
        0.65
    } else {
        1.0
    };
    let border_left_color = match entry.level {
        LogLevel::Warn => tokens::YELLOW,
        LogLevel::Error => tokens::RED,
        _ => Color::NONE,
    };
    let row_bg = if matches!(entry.level, LogLevel::Error) {
        Color::srgba(0.9412, 0.4000, 0.3922, 0.05)
    } else {
        Color::NONE
    };
    // Flexbox row, NOT grid. Bevy 0.18 grid auto-flows extra rows when
    // a `flex(1.0)` track contains a multi-line text — that pushes
    // siblings onto fresh implicit rows and produces the "msg above,
    // t/lvl/src/ch below" visual we get with a wrapped message.
    parent
        .spawn((
            LogRow {
                level: entry.level,
                source_idx,
                entry_idx,
            },
            Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(8.0),
                padding: UiRect::axes(Val::Px(8.0), Val::Px(3.0)),
                align_items: AlignItems::Start,
                border: UiRect::left(Val::Px(2.0)),
                ..default()
            },
            BackgroundColor(row_bg),
            BorderColor::all(border_left_color),
        ))
        .with_children(|r| {
            // t — `T+12.3s` (fixed 56px)
            let sign = if entry.t < 0.0 { "-" } else { "+" };
            let abs_t = entry.t.abs();
            r.spawn((
                Text::new(format!("T{sign}{abs_t:.1}s")),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE.with_alpha(row_alpha)),
                Node {
                    width: Val::Px(56.0),
                    flex_shrink: 0.0,
                    ..default()
                },
            ));
            // lvl glyph (14px, centered)
            r.spawn((
                Text::new(entry.level.glyph()),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 11.5,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(level_color.with_alpha(row_alpha)),
                Node {
                    width: Val::Px(14.0),
                    flex_shrink: 0.0,
                    justify_content: JustifyContent::Center,
                    ..default()
                },
            ));
            // src (64px)
            r.spawn((
                Text::new(entry.src.clone()),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_DIM.with_alpha(row_alpha)),
                Node {
                    width: Val::Px(64.0),
                    flex_shrink: 0.0,
                    ..default()
                },
            ));
            // channel (56px)
            r.spawn((
                Text::new(entry.channel.clone()),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::ACCENT.with_alpha(row_alpha)),
                Node {
                    width: Val::Px(56.0),
                    flex_shrink: 0.0,
                    ..default()
                },
            ));
            // msg — flex_grow=1, wraps on word boundary, tight line
            // height so multi-line rows stay compact.
            r.spawn((
                Text::new(entry.msg.clone()),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 11.0,
                    ..default()
                },
                TextColor(tokens::TEXT.with_alpha(row_alpha)),
                // `WordOrCharacter`: prefer word boundaries, fall back
                // to char break when a single word (e.g.
                // "current_target=depose_zone_4") overflows the cell.
                // `WordBoundary` alone leaves long unbreakable tokens
                // overflowing past the right edge of the dock panel.
                TextLayout::new_with_linebreak(LineBreak::WordOrCharacter),
                bevy::text::LineHeight::RelativeToFont(1.05),
                Node {
                    flex_grow: 1.0,
                    flex_basis: Val::Px(0.0),
                    flex_shrink: 1.0,
                    ..default()
                },
            ));
        });
    let _ = card::outline; // silence unused import warning
}

// ─── Click handlers ────────────────────────────────────────────────

pub fn handle_filter_clicks(
    mut log_state: ResMut<HudLogState>,
    mut focus: ResMut<LogsFilterFocus>,
    mouse: Res<ButtonInput<MouseButton>>,
    q_levels: Query<(&Interaction, &LogLevelToggle), Changed<Interaction>>,
    q_sources: Query<(&Interaction, &LogSourceToggle), Changed<Interaction>>,
    q_input: Query<&Interaction, With<LogsFilterInput>>,
) {
    // Level / source toggle clicks.
    for (i, lv) in &q_levels {
        if *i == Interaction::Pressed {
            log_state.levels[lv.0.idx()] = !log_state.levels[lv.0.idx()];
        }
    }
    for (i, src) in &q_sources {
        if *i == Interaction::Pressed {
            log_state.sources[src.0] = !log_state.sources[src.0];
        }
    }
    // Focus management — every left-click sets focus = (click was on the
    // filter input). Click outside → unfocus, click on input → focus.
    if mouse.just_pressed(MouseButton::Left) {
        let over_input = q_input.iter().any(|i| *i == Interaction::Pressed);
        focus.0 = over_input;
    }
}

/// Reads keyboard input and edits `filter_query` while the input is
/// focused. Backspace removes last char; Escape unfocuses.
pub fn handle_filter_keyboard(
    mut focus: ResMut<LogsFilterFocus>,
    mut log_state: ResMut<HudLogState>,
    mut events: bevy::ecs::message::MessageReader<bevy::input::keyboard::KeyboardInput>,
) {
    if !focus.0 {
        events.clear();
        return;
    }
    use bevy::input::keyboard::Key;
    use bevy::input::ButtonState;
    for ev in events.read() {
        if ev.state != ButtonState::Pressed {
            continue;
        }
        match &ev.logical_key {
            Key::Backspace => {
                log_state.filter_query.pop();
            }
            Key::Escape | Key::Enter => {
                focus.0 = false;
            }
            _ => {
                if let Some(t) = &ev.text {
                    // Filter out control chars (eg. backspace yields "\x08").
                    for c in t.chars() {
                        if !c.is_control() {
                            log_state.filter_query.push(c);
                        }
                    }
                }
            }
        }
    }
}

pub fn refresh_filter_visuals(
    time: Res<Time>,
    log_state: Res<HudLogState>,
    focus: Res<LogsFilterFocus>,
    mut q_rows: Query<
        (&LogRow, &mut Node),
        (Without<LogLevelToggle>, Without<LogSourceToggle>),
    >,
    mut q_count: Query<&mut Text, With<LogsCountText>>,
    mut q_level_btn: Query<
        (&LogLevelToggle, &mut Outline, &Children, &mut BackgroundColor),
        Without<LogSourceToggle>,
    >,
    mut q_source_btn: Query<
        (&LogSourceToggle, &mut Outline, &Children, &mut BackgroundColor),
        Without<LogLevelToggle>,
    >,
    mut q_text: Query<
        (&mut Text, &mut TextColor),
        (
            Without<LogsCountText>,
            Without<LogRow>,
            Without<LogsFilterInputText>,
        ),
    >,
    mut q_input_text: Query<
        (&mut Text, &mut TextColor),
        (
            With<LogsFilterInputText>,
            Without<LogsCountText>,
            Without<LogRow>,
        ),
    >,
    mut q_input: Query<
        &mut Outline,
        (
            With<LogsFilterInput>,
            Without<LogLevelToggle>,
            Without<LogSourceToggle>,
        ),
    >,
    log_entries: Res<HudLogState>,
) {
    // Always run — the row set can change through three paths (logs
    // appended, filter toggled, search edited) and gating on
    // `is_changed` skipped re-applying filters to rows just respawned
    // by `refresh_logs_ui` in the same frame.

    let q_lower = log_state.filter_query.to_lowercase();
    let has_query = !q_lower.is_empty();

    // Row visibility — passes if level + source are both enabled, and
    // (no query, or msg/src contains it).
    let mut visible = 0u32;
    for (row, mut node) in &mut q_rows {
        let mut on = log_state.levels[row.level.idx()] && log_state.sources[row.source_idx];
        if on && has_query {
            let entry = log_entries.entries.get(row.entry_idx);
            on = entry
                .map(|e| {
                    e.msg.to_lowercase().contains(&q_lower)
                        || e.src.to_lowercase().contains(&q_lower)
                })
                .unwrap_or(false);
        }
        node.display = if on { Display::Flex } else { Display::None };
        if on {
            visible += 1;
        }
    }

    // Count text.
    if let Ok(mut t) = q_count.single_mut() {
        let s = format!("{}/{}", visible, log_state.entries.len());
        if t.0 != s {
            t.0 = s;
        }
    }

    // Level button visuals.
    for (toggle, mut outline, children, mut bg) in &mut q_level_btn {
        let on = log_state.levels[toggle.0.idx()];
        let (text_color, border_color, bg_color) = level_btn_visuals(toggle.0, on);
        outline.color = border_color;
        bg.0 = bg_color;
        if let Some(child) = children.first() {
            if let Ok((_, mut tc)) = q_text.get_mut(*child) {
                tc.0 = text_color;
            }
        }
    }

    // Source button visuals.
    for (toggle, mut outline, children, mut bg) in &mut q_source_btn {
        let on = log_state.sources[toggle.0];
        let (text_color, border_color, bg_color) = source_btn_visuals(on);
        outline.color = border_color;
        bg.0 = bg_color;
        if let Some(child) = children.first() {
            if let Ok((mut t, mut tc)) = q_text.get_mut(*child) {
                tc.0 = text_color;
                let prefix = if on { "\u{25CF} " } else { "\u{25CB} " };
                let label = LOG_SOURCES[toggle.0];
                let s = format!("{prefix}{label}");
                if t.0 != s {
                    t.0 = s;
                }
            }
        }
    }

    // Filter input text + focus chrome.
    //
    //   ┌──────────────────────┬──────────────────────────┐
    //   │ state                │ displayed                │
    //   ├──────────────────────┼──────────────────────────┤
    //   │ empty + not focused  │ "filter…" (TEXT_MUTE)    │
    //   │ empty + focused      │ "<caret>" (TEXT)         │
    //   │ typing               │ "<query><caret>" (TEXT)  │
    //   │ has query, blurred   │ "<query>" (TEXT)         │
    //   └──────────────────────┴──────────────────────────┘
    //
    // The input is 110px wide; in our mono font that's roughly 14
    // characters at 11px. When the query is longer we drop chars off
    // the left so the caret stays visible (mimics a single-line text
    // input with horizontal scroll).
    const VISIBLE_CHARS: usize = 14;
    if let Ok((mut t, mut tc)) = q_input_text.single_mut() {
        let q = log_state.filter_query.as_str();
        let is_placeholder = q.is_empty() && !focus.0;
        // Blink caret 0.55s on / 0.45s off.
        let caret_on = focus.0 && (time.elapsed_secs() % 1.0) < 0.55;
        let caret_glyph = if caret_on { "|" } else { " " };

        let display = if is_placeholder {
            "filter\u{2026}".to_string()
        } else {
            // Truncate query from the left if too long, leave room for
            // the caret.
            let max_q = VISIBLE_CHARS.saturating_sub(1);
            let visible: String = if q.chars().count() > max_q {
                let skip = q.chars().count() - max_q;
                q.chars().skip(skip).collect()
            } else {
                q.to_string()
            };
            if focus.0 {
                format!("{visible}{caret_glyph}")
            } else {
                visible
            }
        };
        if t.0 != display {
            t.0 = display;
        }
        tc.0 = if is_placeholder {
            tokens::TEXT_MUTE
        } else {
            tokens::TEXT
        };
    }
    if let Ok(mut o) = q_input.single_mut() {
        o.color = if focus.0 { tokens::ACCENT } else { tokens::BORDER };
    }
}

/// Frames remaining where the logs panel should be force-stuck to the
/// bottom. Bumped to ~5 frames every time we want auto-scroll (new
/// log arrived, or dock just (re)opened). While the counter is > 0:
///  - clamp_logs_scroll_position keeps writing `sp.y = 99_999`
///  - regular ComputedNode-clamping is skipped
/// The 5-frame window covers Bevy's PostUpdate layout latency: when
/// the dock opens, layout has to reflow and `ComputedNode.scroll_position`
/// reads as 0 for a couple frames, which would otherwise reset the
/// bump back to the top.
#[derive(Resource, Default)]
pub struct LogsStickyFrames(pub u8);

const STICKY_FRAMES_ON_GROW: u8 = 5;
const STICKY_FRAMES_ON_OPEN: u8 = 8;

pub fn refresh_logs_ui(
    mut commands: Commands,
    log_state: Res<HudLogState>,
    fonts: Option<Res<HudFonts>>,
    q: Query<Entity, With<LogsScrollArea>>,
    mut sticky: ResMut<LogsStickyFrames>,
    mut prev_len: Local<usize>,
) {
    if !log_state.is_changed() {
        return;
    }
    let Some(fonts) = fonts else { return };
    let Ok(area) = q.single() else { return };
    commands
        .entity(area)
        .despawn_related::<Children>()
        .with_children(|p| {
            for (idx, entry) in log_state.entries.iter().enumerate() {
                spawn_log_row(p, &fonts, idx, entry);
            }
        });

    // Stick to bottom on grow only — filter toggles change state but
    // don't add entries, so they shouldn't yank the user's scroll.
    if log_state.entries.len() > *prev_len {
        sticky.0 = STICKY_FRAMES_ON_GROW.max(sticky.0);
    }
    *prev_len = log_state.entries.len();
}

/// Bump sticky frames every time the dock just transitioned to Logs.
pub fn auto_scroll_on_dock_open(
    active: Res<super::dock::ActiveDock>,
    mut sticky: ResMut<LogsStickyFrames>,
    mut prev_active: Local<Option<super::widgets::dock_button::DockKind>>,
) {
    let now = active.0;
    let just_opened_logs =
        now == Some(super::widgets::dock_button::DockKind::Logs) && *prev_active != now;
    *prev_active = now;
    if just_opened_logs {
        sticky.0 = STICKY_FRAMES_ON_OPEN.max(sticky.0);
    }
}

/// Single source of truth for the logs scroll position:
///  - while `LogsStickyFrames > 0`, force `sp.y = 99_999` (layout
///    will clamp visually) and decrement the counter
///  - otherwise mirror `ComputedNode.scroll_position` back into
///    `ScrollPosition` so wheel input clamps correctly
pub fn clamp_logs_scroll_position(
    mut sticky: ResMut<LogsStickyFrames>,
    mut q: Query<(&ComputedNode, &mut ScrollPosition), With<LogsScrollArea>>,
) {
    let stick = sticky.0 > 0;
    if stick {
        sticky.0 -= 1;
        if let Ok((_, mut sp)) = q.single_mut() {
            sp.0.y = 99_999.0;
        }
        return;
    }
    for (cn, mut sp) in &mut q {
        let clamped = cn.scroll_position;
        if (sp.0.y - clamped.y).abs() > 0.5 {
            sp.0.y = clamped.y;
        }
    }
}

/// Mouse-wheel handler for the logs scroll area. Scrolls the body
/// node's `ScrollPosition` regardless of cursor position — there's
/// only one scrollable area in the HUD right now, so a hover check
/// would be over-engineering.
pub fn handle_scroll_wheel(
    mut wheel: MessageReader<MouseWheel>,
    active: Res<super::dock::ActiveDock>,
    mut q: Query<&mut ScrollPosition, With<LogsScrollArea>>,
) {
    if active.0 != Some(super::widgets::dock_button::DockKind::Logs) {
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
    // TODO(phase-10): stick-to-bottom — when live logs arrive, if the
    // user is currently at `sp.y == max_possible_offset`, auto-bump
    // `sp.y` to follow the new content. Else keep the current scroll
    // (user is reading older logs, don't yank them).
}
