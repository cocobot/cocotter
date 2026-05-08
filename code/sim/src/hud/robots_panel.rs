//! `RobotsBySide` — populates the dock panel body when active dock =
//! Robots. Reference: `RobotsBySide` in `hud_02.jsx:357-425`.
//!
//! Layout: padding 10, gap 12, two side groups (LEFT yellow / RIGHT
//! blue). Each group has a header (label + optional US chip + count)
//! then one row per robot. Real robots come first, fake adversary last
//! with distinct chrome (red, dashed border, FAKE chip, driver hint).

use bevy::ecs::message::MessageReader;
use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use bevy::text::{FontWeight, TextSpan};

use super::tokens::{self, HudFonts};
use super::widgets::kbd;
use super::{HudMockData, HudRobot, HudRobotsState, Side};

#[derive(Component)]
pub struct RobotsScrollArea;

pub fn populate(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    robots_state: &HudRobotsState,
    our_side: Side,
) {
    parent
        .spawn((
            RobotsScrollArea,
            Node {
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(10.0)),
                row_gap: Val::Px(12.0),
                overflow: Overflow::scroll_y(),
                flex_grow: 1.0,
                min_height: Val::Px(0.0),
                ..default()
            },
        ))
        .with_children(|body| {
            for side in [Side::Left, Side::Right] {
                spawn_side_group(body, fonts, side, our_side, &robots_state.robots);
            }
        });
}

fn side_color(side: Side) -> Color {
    match side {
        Side::Left => tokens::SIDE_LEFT,
        Side::Right => tokens::SIDE_RIGHT,
    }
}

fn side_label(side: Side) -> &'static str {
    match side {
        Side::Left => "LEFT",
        Side::Right => "RIGHT",
    }
}

fn spawn_side_group(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    side: Side,
    our_side: Side,
    robots: &[HudRobot],
) {
    let color = side_color(side);
    // Real robots first, fake adversary last.
    let mut list: Vec<&HudRobot> = robots.iter().filter(|r| r.side == side).collect();
    list.sort_by_key(|r| r.fake as u8);
    let real_count = list.iter().filter(|r| !r.fake).count();
    let us = our_side == side;

    parent
        .spawn(Node {
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(4.0),
            ..default()
        })
        .with_children(|grp| {
            // Group header.
            grp.spawn(Node {
                column_gap: Val::Px(6.0),
                align_items: AlignItems::Center,
                padding: UiRect::left(Val::Px(4.0)),
                ..default()
            })
            .with_children(|hdr| {
                hdr.spawn((
                    Text::new(side_label(side)),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 10.5,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(color),
                ));
                if us {
                    hdr.spawn((
                        Node {
                            padding: UiRect::axes(Val::Px(4.0), Val::Px(0.0)),
                            border_radius: BorderRadius::all(Val::Px(2.0)),
                            ..default()
                        },
                        BackgroundColor(color),
                    ))
                    .with_children(|chip| {
                        chip.spawn((
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
                }
                hdr.spawn((
                    Text::new(format!(
                        "· {} robot{}",
                        real_count,
                        if real_count > 1 { "s" } else { "" }
                    )),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ));
            });
            // Robot rows.
            grp.spawn(Node {
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                ..default()
            })
            .with_children(|rows| {
                if list.is_empty() {
                    rows.spawn((
                        Node {
                            padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
                            ..default()
                        },
                        BackgroundColor(Color::NONE),
                    ))
                    .with_children(|p| {
                        p.spawn((
                            Text::new("no robot"),
                            TextFont {
                                font: fonts.mono.clone(),
                                font_size: 10.5,
                                ..default()
                            },
                            TextColor(tokens::TEXT_MUTE),
                        ));
                    });
                }
                for r in &list {
                    if r.fake {
                        spawn_fake_row(rows, fonts, r);
                    } else {
                        spawn_real_row(rows, fonts, r, color, us);
                    }
                }
            });
        });
}

fn spawn_real_row(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    r: &HudRobot,
    color: Color,
    us: bool,
) {
    // Build "us" tint by hand from the side color RGB at low alpha —
    // `Color::with_alpha` on a `Color::srgb(...)` produces something
    // that renders much more saturated than 0.06 in Bevy 0.18; explicit
    // `srgba` keeps the tint truly subtle.
    let bg = if us {
        let lin = color.to_srgba();
        Color::srgba(lin.red, lin.green, lin.blue, 0.04)
    } else {
        Color::srgba(1.0, 1.0, 1.0, 0.02)
    };
    let border_left_color = if us { color } else { Color::NONE };
    let dot_color = if r.online {
        tokens::ACCENT_HOT
    } else {
        tokens::TEXT_MUTE
    };
    parent
        .spawn((
            Node {
                padding: UiRect::axes(Val::Px(8.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                border_radius: BorderRadius::all(Val::Px(4.0)),
                border: UiRect::left(Val::Px(2.0)),
                ..default()
            },
            BackgroundColor(bg),
            BorderColor::all(border_left_color),
        ))
        .with_children(|row| {
            // Online dot — explicit right margin instead of relying on
            // the parent's `column_gap`, which Bevy 0.18 sometimes
            // collapses around `Text` children.
            row.spawn((
                Node {
                    width: Val::Px(6.0),
                    height: Val::Px(6.0),
                    border_radius: BorderRadius::all(Val::Px(50.0)),
                    margin: UiRect::right(Val::Px(10.0)),
                    ..default()
                },
                BackgroundColor(dot_color),
            ));
            // Kind label.
            row.spawn((
                Text::new(r.kind),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 12.0,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(color),
                Node {
                    min_width: Val::Px(70.0),
                    margin: UiRect::right(Val::Px(10.0)),
                    ..default()
                },
            ));
            // Pose right-aligned.
            spawn_pose(row, fonts, r);
        });
}

fn spawn_fake_row(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, r: &HudRobot) {
    parent
        .spawn((
            Node {
                padding: UiRect::axes(Val::Px(8.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                border_radius: BorderRadius::all(Val::Px(4.0)),
                border: UiRect::all(Val::Px(1.0)),
                margin: UiRect::top(Val::Px(4.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.9412, 0.4000, 0.3922, 0.05)),
            BorderColor::all(tokens::RED.with_alpha(0.55)),
        ))
        .with_children(|row| {
            row.spawn((
                Node {
                    width: Val::Px(6.0),
                    height: Val::Px(6.0),
                    border_radius: BorderRadius::all(Val::Px(50.0)),
                    margin: UiRect::right(Val::Px(10.0)),
                    ..default()
                },
                BackgroundColor(tokens::RED),
            ));
            // FAKE chip.
            row.spawn((
                Node {
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(1.0)),
                    border_radius: BorderRadius::all(Val::Px(2.0)),
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
                BackgroundColor(tokens::RED),
            ))
            .with_children(|chip| {
                chip.spawn((
                    Text::new("FAKE"),
                    TextFont {
                        font: fonts.mono_bold.clone(),
                        font_size: 8.0,
                        weight: FontWeight::BOLD,
                        ..default()
                    },
                    TextColor(Color::srgb(0.0470, 0.0235, 0.0314)),
                ));
            });
            // adversary label.
            row.spawn((
                Text::new("adversary"),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 12.0,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(tokens::RED),
                Node {
                    margin: UiRect::right(Val::Px(8.0)),
                    ..default()
                },
            ));
            // KBD driver — drop the "· driven by" prefix; on a 400px
            // panel it wraps awkwardly and the kbd alone is enough.
            kbd::spawn(row, fonts, r.driver.unwrap_or("ZQSD"), true);
            // pose right-aligned
            spawn_pose(row, fonts, r);
        });
}

fn spawn_pose(parent: &mut ChildSpawnerCommands, fonts: &HudFonts, r: &HudRobot) {
    // Right-aligned via `margin-left: auto`. Each label+value is a
    // SINGLE Text + TextSpan (rather than two separate Texts in a
    // nested flex) so they can't be split by the flex layout — that
    // was the bug where the leading char of the first pair vanished
    // on offline / adversary rows.
    parent
        .spawn(Node {
            margin: UiRect::left(Val::Auto),
            column_gap: Val::Px(10.0),
            ..default()
        })
        .with_children(|p| {
            // x/y in metres, θ in degrees (more readable than rad for
            // a UI). 3 decimals on x/y → mm precision on a 3 m table.
            let theta_deg = r.theta.to_degrees();
            for (label, value) in [
                ("x ", format!("{:.3}", r.x)),
                ("y ", format!("{:.3}", r.y)),
                ("\u{03B8} ", format!("{:.0}\u{00B0}", theta_deg)),
            ] {
                p.spawn((
                    Text::new(label),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 11.0,
                        ..default()
                    },
                    TextColor(tokens::TEXT_MUTE),
                ))
                .with_children(|t| {
                    t.spawn((
                        TextSpan::new(value),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 11.0,
                            ..default()
                        },
                        TextColor(tokens::TEXT),
                    ));
                });
            }
        });
}

pub fn handle_scroll_wheel(
    mut wheel: MessageReader<MouseWheel>,
    active: Res<super::dock::ActiveDock>,
    mut q: Query<&mut ScrollPosition, With<RobotsScrollArea>>,
) {
    if active.0 != Some(super::widgets::dock_button::DockKind::Robots) {
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

#[allow(dead_code)]
fn _suppress(_: HudMockData) {} // keep import path stable

// ─── Phase 10c: live binding + UI refresh ──────────────────────────

/// Read the live `World` snapshot every frame and rebuild the
/// `HudRobotsState` resource. Only rebuilds the resource if anything
/// actually changed (so `Changed<HudRobotsState>` doesn't fire every
/// frame for stationary robots — we compare the produced Vec with the
/// existing one).
pub fn bind_robots(
    world: Res<crate::controls::SharedWorld>,
    config: Res<crate::app::SimConfigRes>,
    mut state: ResMut<super::HudRobotsState>,
) {
    use crate::world::EntityKind;
    use sim_protocol::RobotKind;

    let x_max_mm = config.0.field.x_max_mm as f32;
    let mut next: Vec<super::HudRobot> = Vec::new();
    for (id, snap) in world.0.entries() {
        let kind = match snap.kind {
            EntityKind::Robot(RobotKind::Galipeur) => "Galipeur",
            EntityKind::Robot(RobotKind::Pami) => "Pami",
            EntityKind::Robot(RobotKind::Adversary) => "Adversary",
            EntityKind::Human => continue,
        };
        let fake = matches!(snap.kind, EntityKind::Robot(RobotKind::Adversary));
        // Real robots are spawned with `{kind}-{side_name}` ids by
        // `controls::launch_robot` — parse the side from the suffix
        // rather than inferring from position (which is wrong for
        // robots that have crossed the midline).
        let side = parse_side_from_id(&id).unwrap_or_else(|| {
            // Fallback: position-based. Used by the adversary (id
            // "adversary" with no suffix).
            if snap.pose.x_mm < x_max_mm / 2.0 {
                Side::Left
            } else {
                Side::Right
            }
        });
        next.push(super::HudRobot {
            id: id.clone(),
            kind,
            side,
            x: snap.pose.x_mm * 0.001,
            y: snap.pose.y_mm * 0.001,
            theta: snap.pose.theta_rad,
            online: true,
            fake,
            driver: if fake { Some("ZQSD") } else { None },
        });
    }
    next.sort_by(|a, b| (a.fake as u8).cmp(&(b.fake as u8)).then(a.id.cmp(&b.id)));

    // Bypass change detection if nothing actually changed (stationary
    // robots would otherwise mark the resource Changed every frame and
    // spam UI rebuilds).
    if !robots_equal(&state.robots, &next) {
        state.robots = next;
    }
}

fn parse_side_from_id(id: &str) -> Option<Side> {
    if id.ends_with("-left") {
        Some(Side::Left)
    } else if id.ends_with("-right") {
        Some(Side::Right)
    } else {
        None
    }
}

fn robots_equal(a: &[super::HudRobot], b: &[super::HudRobot]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b.iter()).all(|(x, y)| {
        x.id == y.id
            && x.kind == y.kind
            && x.side == y.side
            // 0.5 mm tolerance — below the displayed precision (3
            // decimals = 1 mm).
            && (x.x - y.x).abs() < 0.0005
            && (x.y - y.y).abs() < 0.0005
            // ~0.5° in radians.
            && (x.theta - y.theta).abs() < 0.009
            && x.online == y.online
            && x.fake == y.fake
    })
}

/// Marker on the robots dock-pane body so the refresh system can
/// despawn and rebuild its children when `HudRobotsState` changes.
#[derive(Component)]
pub struct RobotsBodyRoot;

/// Despawn all children of the robots dock body and re-populate from
/// the latest `HudRobotsState`. Runs only when the state has changed
/// (so stationary panels don't churn the UI tree).
pub fn refresh_robots_ui(
    mut commands: Commands,
    state: Res<super::HudRobotsState>,
    mock: Res<super::HudMockData>,
    fonts: Option<Res<super::tokens::HudFonts>>,
    q_body: Query<Entity, With<RobotsBodyRoot>>,
) {
    if !state.is_changed() {
        return;
    }
    let Some(fonts) = fonts else { return };
    let Ok(body) = q_body.single() else { return };

    commands
        .entity(body)
        .despawn_related::<Children>()
        .with_children(|p| {
            populate(p, &fonts, &state, mock.our_side);
        });
}
