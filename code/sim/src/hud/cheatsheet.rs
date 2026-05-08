//! `CheatsheetModal` — full-screen overlay listing every keyboard
//! shortcut, grouped by `ShortcutScope`. Reference: `CheatsheetModal`
//! in `hud_02.jsx:312-341`.
//!
//! Triggered by:
//!  - pressing `?` (Shift+/) on the keyboard
//!  - clicking the "[?] full cheatsheet" button in the footer
//!
//! Closes on:
//!  - Escape
//!  - clicking the backdrop
//!  - clicking the close button
//!
//! Layout: dark backdrop on top of everything, centered card 480..640
//! wide, 3-column grid of `KBD desc` rows.

use bevy::prelude::*;
use bevy::text::FontWeight;

use super::tokens::{self, HudFonts};
use super::widgets::card;
use super::widgets::hud_btn;
use super::widgets::kbd;
use super::{
    CheatsheetVisible, HudRoot, LogsFilterFocus, Shortcut, ShortcutScope, SHORTCUTS,
};

#[derive(Component)]
pub struct CheatsheetRoot;

/// Backdrop entity — clicking it closes the modal.
#[derive(Component)]
pub struct CheatsheetBackdrop;

/// Close button (top-right of the card).
#[derive(Component)]
pub struct CheatsheetCloseBtn;

pub fn setup_cheatsheet(
    mut commands: Commands,
    fonts: Res<HudFonts>,
    root: Single<Entity, With<HudRoot>>,
) {
    let modal = commands
        .spawn((
            CheatsheetRoot,
            CheatsheetBackdrop,
            Button,
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(0.0),
                left: Val::Px(0.0),
                right: Val::Px(0.0),
                bottom: Val::Px(0.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                display: Display::None,
                ..default()
            },
            // Mockup uses 0.55 with a backdrop-blur — without blur,
            // a much darker scrim is needed to make the modal pop.
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.85)),
            ZIndex(50),
        ))
        .id();
    commands.entity(*root).add_child(modal);

    commands.entity(modal).with_children(|p| {
        // Card.
        p.spawn((
            // Block clicks from reaching backdrop.
            Button,
            Node {
                padding: UiRect::all(Val::Px(20.0)),
                min_width: Val::Px(480.0),
                max_width: Val::Px(640.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(14.0),
                border_radius: card::radius_panel(),
                ..default()
            },
            BackgroundColor(tokens::PANEL_BG),
            card::outline(),
        ))
        .with_children(|card| {
            // Header.
            card.spawn(Node {
                justify_content: JustifyContent::SpaceBetween,
                align_items: AlignItems::Center,
                ..default()
            })
            .with_children(|hdr| {
                // section-title "keyboard shortcuts" (dot + label)
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
                        Text::new("keyboard shortcuts"),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 11.0,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
                // Close button.
                hdr.spawn((CheatsheetCloseBtn, Button, hud_btn::bundle()))
                    .with_children(|btn| {
                        btn.spawn((
                            Text::new("\u{2715} esc"),
                            TextFont {
                                font: fonts.mono.clone(),
                                font_size: 10.5,
                                ..default()
                            },
                            TextColor(tokens::TEXT_DIM),
                        ));
                    });
            });

            // Body — 3-column grid (HUD / Match / View).
            card.spawn(Node {
                display: Display::Grid,
                grid_template_columns: vec![
                    GridTrack::flex(1.0),
                    GridTrack::flex(1.0),
                    GridTrack::flex(1.0),
                ],
                column_gap: Val::Px(16.0),
                ..default()
            })
            .with_children(|grid| {
                for (scope, label) in [
                    (ShortcutScope::Hud, "HUD"),
                    (ShortcutScope::Match, "Match"),
                    (ShortcutScope::View, "View / Camera"),
                ] {
                    spawn_group(grid, &fonts, scope, label);
                }
            });

        });
    });
}

fn spawn_group(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    scope: ShortcutScope,
    label: &str,
) {
    let items: Vec<&Shortcut> = SHORTCUTS.iter().filter(|s| s.scope == scope).collect();
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(4.0),
            ..default()
        })
        .with_children(|col| {
            // Group label.
            col.spawn((
                Text::new(label.to_uppercase()),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 9.5,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
                Node {
                    margin: UiRect::bottom(Val::Px(4.0)),
                    ..default()
                },
            ));
            // Items.
            for s in items {
                col.spawn(Node {
                    column_gap: Val::Px(8.0),
                    align_items: AlignItems::Center,
                    padding: UiRect::axes(Val::Px(0.0), Val::Px(3.0)),
                    ..default()
                })
                .with_children(|row| {
                    // KBD with min-width 32 (mockup spec).
                    row.spawn(Node {
                        min_width: Val::Px(32.0),
                        ..default()
                    })
                    .with_children(|wrap| {
                        kbd::spawn(wrap, fonts, s.key, false);
                    });
                    row.spawn((
                        Text::new(s.desc),
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

// ─── Open/close handlers ────────────────────────────────────────────

/// Toggle on `?` regardless of keyboard layout (US QWERTY = Shift+/,
/// FR AZERTY = Shift+,, etc). We read the *logical* key from the
/// keyboard event stream rather than `KeyCode` so the layout doesn't
/// matter.
pub fn handle_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    focus: Res<LogsFilterFocus>,
    mut events: bevy::ecs::message::MessageReader<bevy::input::keyboard::KeyboardInput>,
    mut visible: ResMut<CheatsheetVisible>,
) {
    if focus.0 {
        events.clear();
        return;
    }
    use bevy::input::ButtonState;
    for ev in events.read() {
        if ev.state != ButtonState::Pressed {
            continue;
        }
        if let Some(t) = &ev.text {
            if t.contains('?') {
                visible.0 = !visible.0;
            }
        }
    }
    if keys.just_pressed(KeyCode::Escape) && visible.0 {
        visible.0 = false;
    }
}

/// Click the backdrop or close button to dismiss.
pub fn handle_clicks(
    q_backdrop: Query<&Interaction, (Changed<Interaction>, With<CheatsheetBackdrop>)>,
    q_close: Query<&Interaction, (Changed<Interaction>, With<CheatsheetCloseBtn>)>,
    mut visible: ResMut<CheatsheetVisible>,
) {
    for i in &q_backdrop {
        if *i == Interaction::Pressed {
            visible.0 = false;
        }
    }
    for i in &q_close {
        if *i == Interaction::Pressed {
            visible.0 = false;
        }
    }
}

/// Apply visibility from the resource. Run only when changed.
pub fn refresh_visibility(
    visible: Res<CheatsheetVisible>,
    mut q: Query<&mut Node, With<CheatsheetRoot>>,
) {
    if !visible.is_changed() {
        return;
    }
    if let Ok(mut n) = q.single_mut() {
        n.display = if visible.0 {
            Display::Flex
        } else {
            Display::None
        };
    }
}
