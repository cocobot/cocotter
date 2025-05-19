use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::{iso_8859_15::FONT_6X10, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{Line, PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};

use crate::ui::UIScreen;

pub struct Start {
    last_blink: bool,
}

impl Start {
    pub fn new() -> Self {
        Self {
            last_blink: true,
        }
    }
}

impl UIScreen for Start {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        //reset the display
        Rectangle::new(offset, size)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(display)
            .unwrap();

        const PAMI_WIDTH: u32 = 20;
        const PAMI_HEIGHT: u32 = 28;
        const PAMI_SEMI_HEIGHT: u32 = 20;
        const PAMI_GAP: i32 = 5;

        Line::new(
            offset + Point { x: 2, y: 3 },
            offset + Point { x: size.width as i32 - 3, y: 3 },
        ).into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display)
            .unwrap();

        Line::new(
            offset + Point { x: size.width as i32 - 3, y: 3 },
            offset + Point { x: size.width as i32 - 3, y: size.height as i32 - 2 },
        ).into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display)
            .unwrap();

        for i in 0..4 {
            let mut stroke_size = 1;

            if i == 2 {
                if self.last_blink {
                    stroke_size = 3;
                }
                self.last_blink = !self.last_blink;
            }

            Rectangle::new(offset + Point { x: 7 + i * (PAMI_GAP + PAMI_WIDTH as i32), y: 10 }, Size::new(PAMI_WIDTH, PAMI_HEIGHT))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, stroke_size))
                .draw(display)
                .unwrap();

            Rectangle::new(offset + Point { x: 7 + i * (PAMI_GAP + PAMI_WIDTH as i32), y: 10 }, Size::new(PAMI_WIDTH, PAMI_SEMI_HEIGHT))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, stroke_size))
                .draw(display)
                .unwrap();
        } 

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        let text_style_center = TextStyleBuilder::new()
            .baseline(Baseline::Bottom)
            .alignment(Alignment::Center)
            .build();

        Text::with_text_style(
            &format!("Violet"), 
            Point::new(offset.x + size.width as i32 / 2, offset.y + size.height as i32 - 1), 
            text_style,
            text_style_center
        ).draw(display).unwrap();
    }

    fn get_priority(&self) -> usize {
        0
    }

    fn get_screen_name(&self) -> &'static str {
        "Start"
    }
}