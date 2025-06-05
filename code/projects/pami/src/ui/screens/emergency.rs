use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::MonoTextStyleBuilder, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};
use embedded_vintage_fonts::FONT_24X32;

use crate::{events::Event, ui::UIScreen};

pub struct Emergency {
    triggered: bool,
}

impl Emergency {
    pub fn new() -> Self {
        Self {
            triggered: false,
        }
    }
}

impl UIScreen for Emergency {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
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

        if self.triggered {
            Text::with_text_style(
                &format!("ARU"), 
                Point::new(offset.x + size.width as i32 / 2, offset.y + size.height as i32 / 2), 
                text_style,
                text_style_center
            ).draw(display).unwrap();
        }
    }

    fn handle_event(&mut self, event: &Event) {
        match event {
            Event::EmergencyTriggered { } => {
                self.triggered = true;
            }
            _ => {}
        }
    }

    fn get_priority(&self) -> isize {
        if self.triggered {
           return 5;
        }

        return -1;
    }

    fn get_screen_name(&self) -> &'static str {
        "ARU"
    }
}