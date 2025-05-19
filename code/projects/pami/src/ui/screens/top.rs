use board_pami_2023::DisplayType;
use embedded_graphics::{mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};

use crate::ui::UIScreen;

pub struct Top {
    id: usize,
    color: String,
    battery_mv: Option<f32>,
    battery_percent: Option<f32>,
}

impl Top {
    pub fn new(id: usize, color: String) -> Self {
        Self {
            id,
            color,
            battery_mv: None,
            battery_percent: None,
        }
    }

    pub fn set_battery(&mut self, voltage_mv: f32, percentage: f32) {
        self.battery_mv = Some(voltage_mv);
        self.battery_percent = Some(percentage);
    }
}

impl UIScreen for Top {
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
            &format!("{} ({})", self.color, self.id), 
            offset, 
            text_style,
            text_style_left
        ).draw(display).unwrap();

        if self.battery_mv.is_some() && self.battery_percent.is_some() {
            let battery_text = format!("{:.2} V ({:.0} %)", self.battery_mv.unwrap() / 1000.0, self.battery_percent.unwrap());
            Text::with_text_style(
                &battery_text, 
                Point::new(offset.x + size.width as i32 - 1, offset.y), 
                text_style,
                text_style_right
            ).draw(display).unwrap();
        }
    }

    fn get_screen_name(&self) -> &'static str {
        "Top"
    }
}