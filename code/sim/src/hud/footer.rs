//! Bottom strip — three `kbd` shortcut groups (HUD / MATCH / VIEW), a
//! "? full cheatsheet" button, and a live IPC stats line. Reference:
//! `HudBottomStrip` in `hud_03.jsx:156-193`.
//!
//! Layout: full-width sticky bar, `bottom: 0`, padding `6px 10px`,
//! `border-top: 1px BORDER`, `gap: 16`.

use bevy::prelude::*;
use bevy::text::{FontWeight, TextSpan};

use super::tokens::{self, HudFonts};
use super::widgets::{hud_btn, kbd};
use super::{CheatsheetVisible, HudMockData, HudRoot};

#[derive(Component)]
pub struct FooterRoot;

/// Marker on the "[?] full cheatsheet" button so its clicks can open
/// the cheatsheet modal.
#[derive(Component)]
pub struct OpenCheatsheetBtn;

/// Marker for the per-frame text updates inside the footer.
#[derive(Component, Clone, Copy)]
pub enum FooterText {
    /// Mono span "412/s · 1.2ms · 0 drops" inside the IPC status block.
    IpcStats,
}

// Footer is a compact summary — only the most-used real bindings.
// The full list is in `SHORTCUTS` (cheatsheet modal).
const HUD_GROUP: &[(&str, &str)] = &[
    ("?", "help"),
    ("1", "logs"),
    ("2", "watch"),
    ("3", "robots"),
];
const MATCH_GROUP: &[(&str, &str)] = &[
    ("T", "starter"),
    ("N", "spawn"),
    ("K", "kill"),
    ("R", "reset"),
];
const VIEW_GROUP: &[(&str, &str)] = &[
    ("LMB", "orbit"),
    ("RMB", "pan"),
    // ASCII "scroll" instead of \u{23F5} (mono fonts render it as a
    // tofu square).
    ("scroll", "zoom"),
];

pub fn setup_footer(
    mut commands: Commands,
    fonts: Res<HudFonts>,
    root: Single<Entity, With<HudRoot>>,
) {
    let strip = commands
        .spawn((
            FooterRoot,
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(0.0),
                left: Val::Px(0.0),
                right: Val::Px(0.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(6.0)),
                column_gap: Val::Px(16.0),
                align_items: AlignItems::Center,
                // Force a single line — wrapping pushes the footer up
                // and overlaps the log tail. Trailing items get
                // clipped at narrow widths instead.
                flex_wrap: FlexWrap::NoWrap,
                overflow: Overflow::clip_x(),
                border: UiRect {
                    top: Val::Px(1.0),
                    ..default()
                },
                ..default()
            },
            BackgroundColor(tokens::FOOTER_BG),
            BorderColor::all(tokens::BORDER),
        ))
        .id();
    commands.entity(*root).add_child(strip);

    commands.entity(strip).with_children(|p| {
        spawn_group(p, &fonts, "HUD", HUD_GROUP);
        spawn_group(p, &fonts, "MATCH", MATCH_GROUP);
        spawn_group(p, &fonts, "VIEW", VIEW_GROUP);
        // flex:1 spacer
        p.spawn(Node {
            flex_grow: 1.0,
            ..default()
        });
        spawn_full_cheatsheet_btn(p, &fonts);
        spawn_divider(p);
        spawn_ipc_status(p, &fonts);
    });
}

fn spawn_group(
    parent: &mut ChildSpawnerCommands,
    fonts: &HudFonts,
    label: &str,
    items: &[(&str, &str)],
) {
    parent
        .spawn(Node {
            column_gap: Val::Px(8.0),
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|g| {
            // Group label: 9px, weight 700, color text_mute.
            g.spawn((
                Text::new(label),
                TextFont {
                    font: fonts.mono_bold.clone(),
                    font_size: 9.0,
                    weight: FontWeight::BOLD,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
            ));
            // Items: [kbd] desc — gap 4 inside each item, item-to-item
            // spacing is the parent's column_gap (8px).
            for (k, d) in items {
                g.spawn(Node {
                    column_gap: Val::Px(4.0),
                    align_items: AlignItems::Center,
                    ..default()
                })
                .with_children(|item| {
                    kbd::spawn(item, fonts, k, true /* dim */);
                    item.spawn((
                        Text::new(*d),
                        TextFont {
                            font: fonts.mono.clone(),
                            font_size: 10.5,
                            ..default()
                        },
                        TextColor(tokens::TEXT_DIM),
                    ));
                });
            }
        });
}

fn spawn_full_cheatsheet_btn(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn((OpenCheatsheetBtn, Button, hud_btn::bundle()))
        .with_children(|btn| {
            kbd::spawn(btn, fonts, "?", false /* not dim */);
            btn.spawn((
                Text::new("full cheatsheet"),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_DIM),
            ));
        });
}

fn spawn_divider(parent: &mut ChildSpawnerCommands) {
    parent.spawn((
        Node {
            width: Val::Px(1.0),
            height: Val::Px(16.0),
            ..default()
        },
        BackgroundColor(tokens::BORDER),
    ));
}

fn spawn_ipc_status(parent: &mut ChildSpawnerCommands, fonts: &HudFonts) {
    parent
        .spawn(Node {
            column_gap: Val::Px(6.0),
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|row| {
            // Dot 6×6 accent_hot.
            row.spawn((
                Node {
                    width: Val::Px(6.0),
                    height: Val::Px(6.0),
                    border_radius: BorderRadius::all(Val::Px(50.0)),
                    ..default()
                },
                BackgroundColor(tokens::ACCENT_HOT),
            ));
            // "ipc" mute + mono stats span. Use Text + TextSpan to keep
            // them inline with consistent baseline.
            row.spawn((
                Text::new("ipc "),
                TextFont {
                    font: fonts.mono.clone(),
                    font_size: 10.5,
                    ..default()
                },
                TextColor(tokens::TEXT_MUTE),
            ))
            .with_children(|t| {
                t.spawn((
                    FooterText::IpcStats,
                    TextSpan::new("--/s · --ms · - drops"),
                    TextFont {
                        font: fonts.mono.clone(),
                        font_size: 10.5,
                        ..default()
                    },
                    TextColor(tokens::TEXT),
                ));
            });
        });
}

// ─── Update system ───────────────────────────────────────────────────

/// Click on the "? full cheatsheet" button → toggle modal visibility.
pub fn handle_cheatsheet_btn(
    q: Query<&Interaction, (Changed<Interaction>, With<OpenCheatsheetBtn>)>,
    mut visible: ResMut<CheatsheetVisible>,
) {
    for i in &q {
        if *i == Interaction::Pressed {
            visible.0 = !visible.0;
        }
    }
}

pub fn update_footer_text(data: Res<HudMockData>, mut q: Query<(&FooterText, &mut TextSpan)>) {
    for (kind, mut span) in &mut q {
        match kind {
            FooterText::IpcStats => {
                let s = format!(
                    "{}/s · {:.1}ms · {} drops",
                    data.ipc_pkts_per_s, data.ipc_lat_ms, data.ipc_drops
                );
                if span.0 != s {
                    span.0 = s;
                }
            }
        }
    }
}
