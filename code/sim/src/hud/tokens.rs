//! Design tokens — single source of truth for HUD colors, fonts, spacings,
//! border-radii. Mirrors `HUD_TOKENS` from the React mockup
//! (`hud_02.jsx`), with the alpha bumped from 0.82 → 0.92 on `panel` to
//! compensate for the absence of `backdrop-filter: blur` in Bevy UI 0.18.
//!
//! Many tokens are unused in earlier phases — they get consumed
//! progressively as cards/widgets land in phases 1+.

#![allow(dead_code)]

use bevy::prelude::*;

// ─── Panels / surfaces ──────────────────────────────────────────────────
// Mockup CSS uses 0.82 with `backdrop-filter: blur(8px)` so scene
// chroma bleeds through and pills tinted at 10% read as visibly green.
// Bevy 0.18 has no blur — keeping the same alpha (0.88, slightly above
// mockup) lets pill tints register without making the panel itself
// noticeably translucent.
pub const PANEL_BG: Color = Color::srgba(0.055, 0.0706, 0.0627, 0.88);
pub const PANEL_HI_BG: Color = Color::srgba(0.0784, 0.102, 0.0863, 0.92);
pub const FOOTER_BG: Color = Color::srgba(0.0314, 0.0431, 0.0353, 0.88);

// ─── Borders ────────────────────────────────────────────────────────────
pub const BORDER: Color = Color::srgba(0.5490, 0.7843, 0.5882, 0.22);
pub const BORDER_HI: Color = Color::srgba(0.5490, 0.7843, 0.5882, 0.45);

// ─── Text ───────────────────────────────────────────────────────────────
pub const TEXT: Color = Color::srgb(0.8745, 0.9098, 0.8745); // #dfe8df
pub const TEXT_DIM: Color = Color::srgb(0.5412, 0.6039, 0.5490); // #8a9a8c
pub const TEXT_MUTE: Color = Color::srgb(0.3569, 0.4157, 0.3647); // #5b6a5d

// ─── Accent (Rob'Otter green) ───────────────────────────────────────────
pub const ACCENT: Color = Color::srgb(0.4275, 0.7490, 0.2275); // #6dbf3a
pub const ACCENT_HOT: Color = Color::srgb(0.5529, 0.8784, 0.3216); // #8de052
#[allow(dead_code)]
pub const ACCENT_DIM: Color = Color::srgb(0.2392, 0.4314, 0.1333); // #3d6e22

// ─── Status colors ──────────────────────────────────────────────────────
pub const RED: Color = Color::srgb(0.9412, 0.4000, 0.3922); // #f06664
pub const YELLOW: Color = Color::srgb(0.9137, 0.7255, 0.2863); // #e9b94a
pub const BLUE: Color = Color::srgb(0.4980, 0.7137, 0.8784); // #7fb6e0

// ─── Side colors (table convention: left=yellow, right=blue) ────────────
pub const SIDE_LEFT: Color = Color::srgb(0.9608, 0.7843, 0.2902); // #f5c84a
pub const SIDE_RIGHT: Color = Color::srgb(0.3725, 0.6902, 0.9098); // #5fb0e8

// ─── Radii (px) ─────────────────────────────────────────────────────────
pub const RADIUS_PANEL: f32 = 6.0;
pub const RADIUS_PILL: f32 = 999.0;
pub const RADIUS_KBD: f32 = 3.0;
pub const RADIUS_BTN: f32 = 4.0;

// ─── Font asset paths ───────────────────────────────────────────────────
const FONT_INTER_REG: &str = "fonts/Inter-Regular.ttf";
const FONT_INTER_MED: &str = "fonts/Inter-Medium.ttf";
#[allow(dead_code)]
const FONT_INTER_SB: &str = "fonts/Inter-SemiBold.ttf";
const FONT_INTER_BOLD: &str = "fonts/Inter-Bold.ttf";
const FONT_MONO_REG: &str = "fonts/JetBrainsMono-Regular.ttf";
const FONT_MONO_MED: &str = "fonts/JetBrainsMono-Medium.ttf";
const FONT_MONO_BOLD: &str = "fonts/JetBrainsMono-Bold.ttf";

/// Cached font handles. Loaded once at startup and shared by every HUD
/// system that spawns text. Avoids re-resolving the asset path on every
/// `Text` spawn and ensures a single `Handle<Font>` per weight.
#[derive(Resource, Clone)]
pub struct HudFonts {
    pub sans: Handle<Font>,
    pub sans_medium: Handle<Font>,
    pub sans_semibold: Handle<Font>,
    pub sans_bold: Handle<Font>,
    pub mono: Handle<Font>,
    pub mono_medium: Handle<Font>,
    pub mono_bold: Handle<Font>,
}

/// Bevy UI 0.18 has no `letter-spacing` equivalent. We approximate the
/// CSS `letter-spacing: <em>em` of the mockup by inserting unicode
/// spaces between characters. The result isn't pixel-identical (the
/// space widths are font-defined, not parametric) but visually it
/// reads almost the same and the bold weight stops looking cramped.
///
/// Levels mirror the CSS values used in `hud_02.jsx`/`hud_03.jsx`:
///  - `Tight`  ≈ 0.04em (pills)
///  - `Med`    ≈ 0.12em (state title)
///  - `Wide`   ≈ 0.18em (uppercase tracked labels)
///  - `Wider`  ≈ 0.20em (footer group labels)
pub fn track(s: &str, level: Tracking) -> String {
    if s.is_empty() {
        return String::new();
    }
    let sep = match level {
        // U+200A HAIR SPACE — narrow.
        Tracking::Tight => "\u{200A}",
        // U+2009 THIN SPACE.
        Tracking::Med => "\u{2009}",
        // U+2009 + U+200A — slightly wider than thin alone.
        Tracking::Wide => "\u{2009}",
        // U+2005 FOUR-PER-EM SPACE.
        Tracking::Wider => "\u{2005}",
    };
    let mut out = String::with_capacity(s.len() * 2);
    for (i, c) in s.chars().enumerate() {
        if i > 0 {
            // Don't track around regular spaces — would look weird.
            if c.is_whitespace() || s.chars().nth(i.saturating_sub(1)) == Some(' ') {
                // skip insertion this round
            } else {
                out.push_str(sep);
            }
        }
        out.push(c);
    }
    out
}

#[derive(Clone, Copy)]
pub enum Tracking {
    Tight,
    Med,
    Wide,
    Wider,
}

impl HudFonts {
    pub fn load(asset_server: &AssetServer) -> Self {
        Self {
            sans: asset_server.load(FONT_INTER_REG),
            sans_medium: asset_server.load(FONT_INTER_MED),
            sans_semibold: asset_server.load(FONT_INTER_SB),
            sans_bold: asset_server.load(FONT_INTER_BOLD),
            mono: asset_server.load(FONT_MONO_REG),
            mono_medium: asset_server.load(FONT_MONO_MED),
            mono_bold: asset_server.load(FONT_MONO_BOLD),
        }
    }
}
