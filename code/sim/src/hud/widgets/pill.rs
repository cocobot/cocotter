//! `hud-pill` — small uppercase chip used for status indicators (ipc up,
//! latency, robot count). Reference: `.hud-pill` in `hud_02.jsx:42-51`.
//!
//! Variants:
//!  - `default` : muted text, faint border, 3% white background
//!  - `on`      : accent_hot text, brighter border, 10% accent background

use bevy::prelude::*;

use super::super::tokens;

#[derive(Clone, Copy)]
pub enum PillVariant {
    Default,
    On,
    #[allow(dead_code)]
    Warn,
    #[allow(dead_code)]
    Red,
}

pub fn colors(variant: PillVariant) -> (Color, Color, Color) {
    // Mid-point between mockup-literal (10%) and visible-on-opaque
    // (40%). Iterating with the user.
    match variant {
        PillVariant::Default => (
            tokens::TEXT_DIM,
            tokens::BORDER,
            Color::srgba(1.0, 1.0, 1.0, 0.01),
        ),
        PillVariant::On => (
            tokens::ACCENT_HOT,
            tokens::BORDER_HI,
            Color::srgba(0.4275, 0.7490, 0.2275, 0.035),
        ),
        PillVariant::Warn => (
            tokens::YELLOW,
            Color::srgba(0.9137, 0.7255, 0.2863, 0.45),
            Color::srgba(0.9137, 0.7255, 0.2863, 0.06),
        ),
        PillVariant::Red => (
            tokens::RED,
            Color::srgba(0.9412, 0.4000, 0.3922, 0.45),
            Color::srgba(0.9412, 0.4000, 0.3922, 0.06),
        ),
    }
}

pub fn spawn(
    parent: &mut ChildSpawnerCommands,
    fonts: &super::super::tokens::HudFonts,
    label: &str,
    variant: PillVariant,
) {
    let (text_color, border_color, bg) = colors(variant);
    parent
        .spawn((
            Node {
                border: UiRect::all(Val::Px(1.0)),
                // BorderRadius::MAX → capsule shape, equivalent to CSS
                // `border-radius: 999px`. Avoids sub-pixel weirdness we
                // hit with literal `999px` in Bevy 0.18.
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
