use std::time::Instant;

use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};

use crate::{config::GAME_TIME_SECONDS, events::Event, ui::UIScreen};

pub struct Bottom {
    current_screen_name: &'static str,
    start_time: Option<Instant>,
}

impl Bottom {
    pub fn new() -> Self {
        Self {
            current_screen_name: "?",
            start_time: None,
        }
    }

    pub fn set_screen_name(&mut self, screen_name: &'static str) {
        self.current_screen_name = screen_name;
    }
}

impl UIScreen for Bottom {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        let ellsaped_time = self.start_time
            .map(|start| start.elapsed().as_secs())
            .unwrap_or(0);

        let remaning_time = if ellsaped_time < GAME_TIME_SECONDS {
            GAME_TIME_SECONDS - ellsaped_time
        } else {
            0
        };

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
            &format!("{} s", remaning_time),
            Point::new(offset.x + size.width as i32 - 1, offset.y), 
            text_style,
            text_style_right
        ).draw(display).unwrap();
    }

    fn get_screen_name(&self) -> &'static str {
        "Bottom"
    }

     fn handle_event(&mut self, event: &Event) {
        match event {
            Event::GameStarted { timestamp } => {
                self.start_time = Some(*timestamp);
            }
            _ => {}
        }
    }
}