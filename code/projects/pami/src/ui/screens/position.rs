use board_pami_2023::DisplayType;
use cocotter::position::robot_coordinate::RobotCoordinate;
use embedded_graphics::{mono_font::{iso_8859_14::FONT_6X10, MonoTextStyleBuilder}, pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}, text::{Alignment, Baseline, Text, TextStyleBuilder}};

use crate::{events::Event, ui::UIScreen};

pub struct Position {
    coordinate: RobotCoordinate<2>,
}

impl Position {
    pub fn new() -> Self {
        Self {
            coordinate: RobotCoordinate::from(0.0, 0.0, 0.0),
        }
    }
}

impl UIScreen for Position {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        //reset the display
        Rectangle::new(offset, size)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(display)
            .unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

         let text_style_left = TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Left)
            .build();

        Text::with_text_style(
            &format!("x: {:.1} mm | y: {:.1} mm", self.coordinate.get_x_mm(), self.coordinate.get_y_mm()), 
            offset, 
            text_style,
            text_style_left
        ).draw(display).unwrap();

        Text::with_text_style(
            &format!("angle: {:.1} °", self.coordinate.get_a_deg()), 
            offset + Point::new(0, 20), 
            text_style,
            text_style_left
        ).draw(display).unwrap();        
    }

    fn get_priority(&self) -> isize {
        100
    }

    fn get_screen_name(&self) -> &'static str {
        "Position"
    }

    fn handle_event(&mut self, event: &Event) {
        match event {
            Event::Position { coords } => {
                self.coordinate = coords.clone();
            }
            _ => {}
        }
    }
}