//! Top strip (header) — three cards laid out horizontally, anchored
//! `top:12, left:12, right:12` with `gap:10`. Reference:
//! `HudTopStrip` in `hud_03.jsx:221-256` (live variant) and the
//! underlying `MatchStateBadge`, `ScoreBlock`, `TimerBlock` from
//! `hud_02.jsx:122-195`.
//!
//! Note: `.hud { font-family: mono }` in the mockup makes JetBrains
//! Mono the default for *every* HUD text. Inter is only used outside
//! `.hud`, which we don't reproduce in-engine.

use bevy::prelude::*;
use bevy::text::{FontWeight, TextSpan};

use super::tokens::{self, HudFonts};
use super::widgets::{card, kbd, pill};
use super::{HudMatchState, HudMockData, HudRoot, Side};

// ─── Component markers ──────────────────────────────────────────────────

#[derive(Component)]
pub struct HeaderRoot;

/// One enum drives every text-field update via a tiny query.
#[derive(Component, Clone, Copy)]
pub enum HeaderText {
    StateTitle,
    StateHint,
    ScoreLeft,
    ScoreRight,
    TimerHeader,
    TimerCountdown,
    TimerValue,
    IpcLat,
    IpcRobots,
    /// Mono numeric span "144" inside the stats row.
    FrameFps,
    /// Mono numeric span "1.2" inside the stats row.
    AsservMs,
}

/// One enum tags every node whose Node/BG/Border/BoxShadow we mutate
/// from the per-frame update systems. Using a single tag instead of
/// per-marker structs collapses N conflicting `Query<&mut T, With<X>>`
/// queries into ONE query — Bevy 0.18's disjointness checker doesn't
/// have to reason across queries any more.
#[derive(Component, Clone, Copy, PartialEq, Eq)]
pub enum HeaderNode {
    StateBadgeDot,
    ScoreLeftCell,
    ScoreRightCell,
    ScoreLeftUsChip,
    ScoreRightUsChip,
    TimerProgressFill,
    /// Score+timer card — receives the `hud-glow` animation in RUNNING.
    ScoreTimerCard,
}

// ─── Setup ──────────────────────────────────────────────────────────────

pub fn setup_header(
    mut commands: Commands,
    fonts: Res<HudFonts>,
    root: Single<Entity, With<HudRoot>>,
) {
    let strip = commands
        .spawn((
            HeaderRoot,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(12.0),
                right: Val::Px(12.0),
                column_gap: Val::Px(10.0),
                align_items: AlignItems::Stretch,
                ..default()
            },
        ))
        .id();
    commands.entity(*root).add_child(strip);

    commands.entity(strip).with_children(|strip| {
        spawn_match_state_card(strip, &fonts);
        spawn_score_timer_card(strip, &fonts);
        spawn_ipc_card(strip, &fonts);
    });
}

fn spawn_match_state_card(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn((
            Node {
                // Stable width so the card doesn't resize when state hint
                // text changes length (PREGAME hint is much longer than
                // RUNNING/ENDED). Sized for the longest hint.
                min_width: Val::Px(200.0),
                border_radius: card::radius_panel(),
                padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                column_gap: Val::Px(8.0),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
        ))
        .with_children(|card| {
            // Dot 10×10 — full-radius (circle), with halo BoxShadow.
            // Color + shadow color updated each frame from match state.
            card.spawn((
                HeaderNode::StateBadgeDot,
                Node {
                    width: Val::Px(10.0),
                    height: Val::Px(10.0),
                    border_radius: BorderRadius::all(Val::Px(50.0)),
                    ..default()
                },
                BackgroundColor(tokens::ACCENT),
                BoxShadow::new(
                    tokens::ACCENT.with_alpha(0.12),
                    Val::Px(0.0),
                    Val::Px(0.0),
                    Val::Px(0.0),
                    Val::Px(5.0),
                ),
            ));
            // Two-line text column. All mono per `.hud` rule.
            card.spawn(Node {
                flex_direction: FlexDirection::Column,
                ..default()
            })
            .with_children(|col| {
                col.spawn((
                    HeaderText::StateTitle,
                    Text::new("PREGAME"),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 13.0,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(tokens::BLUE),
                ));
                col.spawn((
                    HeaderText::StateHint,
                    Text::new("press T to insert starter"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
        });
}

fn spawn_score_timer_card(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn((
            HeaderNode::ScoreTimerCard,
            Node {
                border_radius: card::radius_panel(),
                padding: UiRect::axes(Val::Px(18.0), Val::Px(6.0)),
                column_gap: Val::Px(18.0),
                align_items: AlignItems::Center,
                flex_grow: 0.0,
                flex_shrink: 0.0,
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
            // Glow animated each frame in `update_header_nodes` when
            // state==RUNNING. Alpha 0 elsewhere = invisible.
            BoxShadow::new(
                tokens::ACCENT.with_alpha(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
            ),
        ))
        .with_children(|card| {
            spawn_score_block(card, fonts);
            // Vertical divider 1px × 50px.
            card.spawn((
                Node {
                    width: Val::Px(1.0),
                    height: Val::Px(50.0),
                    ..default()
                },
                BackgroundColor(tokens::BORDER),
            ));
            spawn_timer_block(card, fonts);
        });
}

fn spawn_score_block(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn(Node {
            align_items: AlignItems::Baseline,
            column_gap: Val::Px(6.0),
            ..default()
        })
        .with_children(|row| {
            spawn_score_cell(row, fonts, Side::Left);
            row.spawn((
                Text::new("—"),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 42.0 * 0.45,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
                Node {
                    align_self: AlignSelf::Center,
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(0.0)),
                    ..default()
                },
            ));
            spawn_score_cell(row, fonts, Side::Right);
        });
}

fn spawn_score_cell(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, side: Side) {
    let color = match side {
        Side::Left => tokens::SIDE_LEFT,
        Side::Right => tokens::SIDE_RIGHT,
    };
    let label = match side {
        Side::Left => "LEFT",
        Side::Right => "RIGHT",
    };
    let cell_kind = match side {
        Side::Left => HeaderNode::ScoreLeftCell,
        Side::Right => HeaderNode::ScoreRightCell,
    };
    let chip_kind = match side {
        Side::Left => HeaderNode::ScoreLeftUsChip,
        Side::Right => HeaderNode::ScoreRightUsChip,
    };
    parent
        .spawn((
            cell_kind,
            Node {
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_PANEL)),
                padding: UiRect::axes(Val::Px(6.0), Val::Px(3.0)),
                flex_direction: FlexDirection::Column,
                align_items: match side {
                    Side::Left => AlignItems::FlexEnd,
                    Side::Right => AlignItems::FlexStart,
                },
                row_gap: Val::Px(2.0),
                ..default()
            },
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
            BoxShadow::new(
                color.with_alpha(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(6.0),
            ),
        ))
        .with_children(|c| {
            c.spawn(Node {
                column_gap: Val::Px(5.0),
                align_items: AlignItems::Center,
                justify_content: match side {
                    Side::Left => JustifyContent::FlexEnd,
                    Side::Right => JustifyContent::FlexStart,
                },
                ..default()
            })
            .with_children(|hdr| {
                hdr.spawn((
                    chip_kind,
                    Node {
                        padding: UiRect::axes(Val::Px(4.0), Val::Px(0.0)),
                        align_items: AlignItems::Center,
                        display: Display::None,
                        border_radius: BorderRadius::all(Val::Px(2.0)),
                        ..default()
                    },
                    BackgroundColor(color),
                ))
                .with_children(|c2| {
                    c2.spawn((
                        Text::new("US"),
                        TextFont {
                            font: fonts.mono_bold.clone(),
                            font_size: 8.0,
                            weight: FontWeight::BOLD,
                            ..default()
                        },
                        TextColor(Color::srgb(0.0470, 0.0627, 0.0627)),
                    ));
                });
                hdr.spawn((
                    Text::new(label),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 9.0,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(color),
                ));
            });
            c.spawn((
                match side {
                    Side::Left => HeaderText::ScoreLeft,
                    Side::Right => HeaderText::ScoreRight,
                },
                Text::new("00"),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 42.0,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(color),
                bevy::text::LineHeight::RelativeToFont(1.0),
            ));
        });
}

fn spawn_timer_block(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(4.0),
            min_width: Val::Px(130.0),
            ..default()
        })
        .with_children(|col| {
            col.spawn(Node {
                justify_content: JustifyContent::SpaceBetween,
                ..default()
            })
            .with_children(|hdr| {
                hdr.spawn((
                    HeaderText::TimerHeader,
                    Text::new("MATCH · PREGAME"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 9.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
                hdr.spawn((
                    HeaderText::TimerCountdown,
                    Text::new("T-100.0s"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 9.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ));
            });
            col.spawn((
                HeaderText::TimerValue,
                Text::new("01:40"),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 36.0,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(tokens::TEXT),
                bevy::text::LineHeight::RelativeToFont(1.0),
            ));
            col.spawn((
                Node {
                    width: Val::Percent(100.0),
                    height: Val::Px(3.0),
                    border_radius: BorderRadius::all(Val::Px(2.0)),
                    ..default()
                },
                BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.07)),
            ))
            .with_children(|bar| {
                bar.spawn((
                    HeaderNode::TimerProgressFill,
                    Node {
                        width: Val::Percent(0.0),
                        height: Val::Percent(100.0),
                        ..default()
                    },
                    BackgroundColor(tokens::ACCENT),
                ));
            });
        });
}

fn spawn_ipc_card(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn((
            Node {
                border_radius: card::radius_panel(),
                padding: UiRect::axes(Val::Px(12.0), Val::Px(6.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(4.0),
                justify_content: JustifyContent::Center,
                flex_grow: 1.0,
                flex_basis: Val::Px(0.0),
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
        ))
        .with_children(|card| {
            card.spawn(Node {
                column_gap: Val::Px(6.0),
                justify_content: JustifyContent::FlexEnd,
                ..default()
            })
            .with_children(|row| {
                pill::spawn(row, fonts, "● ipc up", pill::PillVariant::On);
                spawn_marked_pill(
                    row,
                    fonts,
                    "—",
                    pill::PillVariant::Default,
                    HeaderText::IpcLat,
                );
                spawn_marked_pill(
                    row,
                    fonts,
                    "—",
                    pill::PillVariant::Default,
                    HeaderText::IpcRobots,
                );
            });
            // Stats row — 5 sibling spans, gap 8.
            card.spawn(Node {
                column_gap: Val::Px(8.0),
                justify_content: JustifyContent::FlexEnd,
                align_items: AlignItems::Center,
                ..default()
            })
            .with_children(|row| {
                // Group 1: "frame [144] fps"
                row.spawn((
                    Text::new("frame "),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.5,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ))
                .with_children(|t| {
                    t.spawn((
                        HeaderText::FrameFps,
                        TextSpan::new("---"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT),
                    ));
                    t.spawn((
                        TextSpan::new(" fps"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
                // Mute "·"
                row.spawn((
                    Text::new("·"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.5,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ));
                // Group 3: "asserv [1.2] ms"
                row.spawn((
                    Text::new("asserv "),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.5,
                        ..default()
                    },
                    TextColor(tokens::TEXT_DIM),
                ))
                .with_children(|t| {
                    t.spawn((
                        HeaderText::AsservMs,
                        TextSpan::new("---"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT),
                    ));
                    t.spawn((
                        TextSpan::new(" ms"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
                row.spawn((
                    Text::new("·"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.5,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ));
                // Group 5: [H] hide
                row.spawn(Node {
                    column_gap: Val::Px(4.0),
                    align_items: AlignItems::Center,
                    ..default()
                })
                .with_children(|g| {
                    kbd::spawn(g, fonts, "H", true);
                    g.spawn((
                        Text::new("hide"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
            });
        });
}

fn spawn_marked_pill(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    label: &str,
    variant: pill::PillVariant,
    kind: HeaderText,
) {
    let (text_color, border_color, bg) = pill::colors(variant);
    parent
        .spawn((
            Node {
                border: UiRect::all(Val::Px(1.0)),
                border_radius: BorderRadius::MAX,
                padding: UiRect::axes(Val::Px(7.0), Val::Px(2.0)),
                align_items: AlignItems::Center,
                column_gap: Val::Px(5.0),
                ..default()
            },
            BackgroundColor(bg),
            BorderColor::all(border_color),
        ))
        .with_children(|p| {
            p.spawn((
                kind,
                Text::new(label.to_uppercase()),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(text_color),
            ));
        });
}

// ─── Update systems ────────────────────────────────────────────────────

/// Drives every text/textspan field by dispatching on `HeaderText`.
/// Two queries (Text vs TextSpan) are inherently disjoint — a given
/// entity has Text *or* TextSpan, never both — so the `Without<>`
/// filters keep Bevy's checker satisfied.
pub fn update_header_text(
    data: Res<HudMockData>,
    mut q_text: Query<(&HeaderText, &mut Text, &mut TextColor), Without<TextSpan>>,
    mut q_span: Query<(&HeaderText, &mut TextSpan), Without<Text>>,
) {
    let (state_label, state_hint, state_color) = match data.match_state {
        HudMatchState::Pregame => ("PREGAME", "press T to insert starter", tokens::BLUE),
        HudMatchState::Starter => ("STARTER", "press T to remove", tokens::YELLOW),
        HudMatchState::Running => ("RUNNING", "match in progress", tokens::ACCENT),
        HudMatchState::Ended => ("ENDED", "press R to reset", tokens::TEXT_DIM),
    };
    let remain = (data.match_secs - data.elapsed_secs).max(0.0);
    let danger = remain <= 10.0 && data.match_state == HudMatchState::Running;
    let timer_color = match data.match_state {
        HudMatchState::Ended => tokens::TEXT_DIM,
        HudMatchState::Pregame => tokens::BLUE,
        HudMatchState::Starter => tokens::YELLOW,
        HudMatchState::Running if danger => tokens::RED,
        HudMatchState::Running => tokens::TEXT,
    };

    for (kind, mut text, mut color) in &mut q_text {
        match kind {
            HeaderText::StateTitle => {
                set_text(&mut text, state_label);
                color.0 = state_color;
            }
            HeaderText::StateHint => {
                set_text(&mut text, state_hint);
                color.0 = tokens::TEXT_DIM;
            }
            HeaderText::ScoreLeft => set_text(&mut text, &format!("{:02}", data.score_left)),
            HeaderText::ScoreRight => set_text(&mut text, &format!("{:02}", data.score_right)),
            HeaderText::TimerHeader => set_text(&mut text, &format!("MATCH · {state_label}")),
            HeaderText::TimerCountdown => set_text(&mut text, &format!("T-{:>5.1}s", remain)),
            HeaderText::TimerValue => {
                let m = (remain / 60.0).floor() as i32;
                let s = (remain - (m as f32) * 60.0).floor() as i32;
                set_text(&mut text, &format!("{:02}:{:02}", m, s));
                color.0 = timer_color;
            }
            HeaderText::IpcLat => set_text(
                &mut text,
                &format!("{:.1}MS · {}/S", data.ipc_lat_ms, data.ipc_pkts_per_s).to_uppercase(),
            ),
            HeaderText::IpcRobots => set_text(
                &mut text,
                &format!("{}/{} ROBOTS", data.robots_online, data.robots_total),
            ),
            HeaderText::FrameFps | HeaderText::AsservMs => {}
        }
    }

    for (kind, mut span) in &mut q_span {
        match kind {
            HeaderText::FrameFps => set_span(&mut span, &format!("{}", data.frame_fps)),
            HeaderText::AsservMs => set_span(&mut span, &format!("{:.1}", data.asserv_ms)),
            _ => {}
        }
    }
}

fn set_text(text: &mut Text, s: &str) {
    if text.0 != s {
        text.0 = s.to_string();
    }
}

fn set_span(span: &mut TextSpan, s: &str) {
    if span.0 != s {
        span.0 = s.to_string();
    }
}

/// One Query, one System — handles every HUD-node mutation
/// (BG / Border / BoxShadow / Node) via `HeaderNode` dispatch.
pub fn update_header_nodes(
    time: Res<Time>,
    data: Res<HudMockData>,
    mut q: Query<(
        &HeaderNode,
        &mut Node,
        &mut BackgroundColor,
        &mut BorderColor,
        Option<&mut BoxShadow>,
    )>,
) {
    let state_color = match data.match_state {
        HudMatchState::Pregame => tokens::BLUE,
        HudMatchState::Starter => tokens::YELLOW,
        HudMatchState::Running => tokens::ACCENT,
        HudMatchState::Ended => tokens::TEXT_DIM,
    };
    let pct = (data.elapsed_secs / data.match_secs).clamp(0.0, 1.0);
    let danger = (data.match_secs - data.elapsed_secs).max(0.0) <= 10.0
        && data.match_state == HudMatchState::Running;
    let running = data.match_state == HudMatchState::Running;

    // ── Blink envelope: 1.1s period, step animation matching the CSS
    //    @keyframes hud-blink (0..55% → 1.0, 60..100% → 0.25, with a
    //    short linear segment in 55..60%).
    let blink_alpha = if running {
        let phase = (time.elapsed_secs() / 1.1).rem_euclid(1.0);
        if phase < 0.55 {
            1.0
        } else if phase < 0.60 {
            // 55%..60% lerp from 1.0 → 0.25
            let k = (phase - 0.55) / 0.05;
            1.0 - k * 0.75
        } else {
            0.25
        }
    } else {
        1.0
    };

    // ── Glow envelope: 2.4s ease-in-out cycle (cosine) — alpha,
    //    blur, spread interpolate from 0 → 0.35 / 14px / 1px and back.
    let glow_t = if running {
        let phase = (time.elapsed_secs() / 2.4).rem_euclid(1.0);
        0.5 - 0.5 * (phase * std::f32::consts::TAU).cos() // 0..1 ease-in-out
    } else {
        0.0
    };
    let glow_alpha = 0.35 * glow_t;
    let glow_blur = 14.0 * glow_t;
    let glow_spread = 1.0 * glow_t;

    for (kind, mut node, mut bg, mut border, shadow) in &mut q {
        match kind {
            HeaderNode::StateBadgeDot => {
                bg.0 = state_color.with_alpha(blink_alpha);
                if let Some(mut s) = shadow {
                    if let Some(s0) = s.0.first_mut() {
                        s0.color = state_color.with_alpha(0.12 * blink_alpha);
                    }
                }
            }
            HeaderNode::ScoreTimerCard => {
                if let Some(mut s) = shadow {
                    if let Some(s0) = s.0.first_mut() {
                        s0.color = tokens::ACCENT.with_alpha(glow_alpha);
                        s0.blur_radius = Val::Px(glow_blur);
                        s0.spread_radius = Val::Px(glow_spread);
                    }
                }
            }
            HeaderNode::ScoreLeftCell => {
                apply_cell_us(
                    &mut node,
                    &mut bg,
                    &mut border,
                    shadow,
                    data.our_side == Side::Left,
                    tokens::SIDE_LEFT,
                );
            }
            HeaderNode::ScoreRightCell => {
                apply_cell_us(
                    &mut node,
                    &mut bg,
                    &mut border,
                    shadow,
                    data.our_side == Side::Right,
                    tokens::SIDE_RIGHT,
                );
            }
            HeaderNode::ScoreLeftUsChip => {
                node.display = if data.our_side == Side::Left {
                    Display::Flex
                } else {
                    Display::None
                };
            }
            HeaderNode::ScoreRightUsChip => {
                node.display = if data.our_side == Side::Right {
                    Display::Flex
                } else {
                    Display::None
                };
            }
            HeaderNode::TimerProgressFill => {
                node.width = Val::Percent(pct * 100.0);
                bg.0 = if danger { tokens::RED } else { tokens::ACCENT };
            }
        }
    }
}

fn apply_cell_us(
    node: &mut Node,
    bg: &mut BackgroundColor,
    border: &mut BorderColor,
    shadow: Option<Mut<BoxShadow>>,
    is_us: bool,
    side_color: Color,
) {
    if is_us {
        node.padding = UiRect::axes(Val::Px(10.0), Val::Px(3.0));
        bg.0 = side_color.with_alpha(0.078); // mockup `${color}14`
        *border = BorderColor::all(side_color);
        if let Some(mut s) = shadow {
            if let Some(s0) = s.0.first_mut() {
                s0.color = side_color.with_alpha(0.06);
            }
        }
    } else {
        node.padding = UiRect::axes(Val::Px(6.0), Val::Px(3.0));
        bg.0 = Color::NONE;
        *border = BorderColor::all(Color::NONE);
        if let Some(mut s) = shadow {
            if let Some(s0) = s.0.first_mut() {
                s0.color = side_color.with_alpha(0.0);
            }
        }
    }
}
