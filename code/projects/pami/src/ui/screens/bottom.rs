use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};

use crate::ui::UIScreen;

pub struct Bottom {
    current_screen_name: &'static str,
    remaning_time: usize,
}

impl Bottom {
    pub fn new() -> Self {
        Self {
            current_screen_name: "?",
            remaning_time: 100,
        }
    }

    pub fn set_screen_name(&mut self, screen_name: &'static str) {
        self.current_screen_name = screen_name;
    }
}

impl UIScreen for Bottom {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        //reset the display
        Rectangle::new(offset, size)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(display)
            .unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_5X7)
            .text_color(BinaryColor::On)
            .build();

        let text_style_left = TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Left)
            .build();

        let text_style_right = TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Right)
            .build();

        Text::with_text_style(
            &format!("{}", self.current_screen_name), 
            offset, 
            text_style,
            text_style_left
        ).draw(display).unwrap();

        Text::with_text_style(
            &format!("{} s", self.remaning_time),
            Point::new(offset.x + size.width as i32 - 1, offset.y), 
            text_style,
            text_style_right
        ).draw(display).unwrap();
    }

    fn get_screen_name(&self) -> &'static str {
        "Bottom"
    }
}