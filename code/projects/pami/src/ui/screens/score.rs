use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::MonoTextStyleBuilder, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};
use embedded_vintage_fonts::FONT_24X32;

use crate::ui::UIScreen;

pub struct Score {
    current_score: usize,
}

impl Score {
    pub fn new() -> Self {
        Self {
            current_score: 321,
        }
    }
}

impl UIScreen for Score {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        //reset the display
        Rectangle::new(offset, size)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(display)
            .unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_24X32)
            .text_color(BinaryColor::On)
            .build();

        let text_style_center = TextStyleBuilder::new()
            .baseline(Baseline::Middle)
            .alignment(Alignment::Center)
            .build();

        Text::with_text_style(
            &format!("{}", self.current_score), 
            Point::new(offset.x + size.width as i32 / 2, offset.y + size.height as i32 / 2), 
            text_style,
            text_style_center
        ).draw(display).unwrap();
    }

    fn get_priority(&self) -> usize {
        0
    }

    fn get_screen_name(&self) -> &'static str {
        "Score"
    }
}