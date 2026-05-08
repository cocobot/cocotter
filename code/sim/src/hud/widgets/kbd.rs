//! `hud-kbd` — keyboard-key chip used inline (footer hints, [esc] close,
//! dock kbd badges, "[H] hide" in stats). Reference: `.hud-kbd` in
//! `hud_02.jsx:52-63`.
//!
//! Specs from the mockup:
//!  - inline-flex, min-width 18px, height 18px, padding 0 5px
//!  - border-radius 3px, 1px border (bottom = 2px for the bevel feel)
//!  - background rgba(255,255,255,0.06)
//!  - font: mono, 10.5px, weight 600
//!  - color: text (default) / text_dim (dim variant)

use bevy::prelude::*;
use bevy::text::FontWeight;

use super::super::tokens;

pub fn spawn(
    parent: &mut ChildSpawnerCommands,
    fonts: &super::super::tokens::HudFonts,
    label: &str,
    dim: bool,
) {
    let text_color = if dim { tokens::TEXT_DIM } else { tokens::TEXT };
    parent
        .spawn((
            Node {
                min_width: Val::Px(18.0),
                height: Val::Px(18.0),
                padding: UiRect::axes(Val::Px(5.0), Val::Px(0.0)),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                border: UiRect {
                    top: Val::Px(1.0),
                    right: Val::Px(1.0),
                    bottom: Val::Px(2.0),
                    left: Val::Px(1.0),
                },
                border_radius: BorderRadius::all(Val::Px(tokens::RADIUS_KBD)),
                ..default()
            },
            BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.015)),
            BorderColor::all(tokens::BORDER),
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(label),
                TextFont {
                    font: fonts.mono_medium.clone(),
                    font_size: 10.5,
                    weight: FontWeight::SEMIBOLD,
                    ..default()
                },
                TextColor(text_color),
            ));
        });
}
