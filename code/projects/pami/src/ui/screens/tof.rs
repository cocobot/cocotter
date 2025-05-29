use std::time::Instant;

use board_pami_2023::DisplayType;
use cocotter::trajectory::TrajectoryEvent;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{Line, PrimitiveStyle, Rectangle}};

use crate::{events::Event, ui::UIScreen};

pub struct Tof {
    triggered: bool,
    back_distance: [i16; 8],
}

impl Tof {
    pub fn new() -> Self {
        Self {
            triggered: false,
            back_distance: [i16::MAX; 8],
        }
    }
}

impl UIScreen for Tof {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size) {
        const PAMI_WIDTH: i32 = 20;
        const PAMI_HEIGHT: i32 = 28;
        const PAMI_SEMI_HEIGHT: i32 = 20;

        let pami_origin : Point = offset + Point::new((size.width as i32 - PAMI_HEIGHT) / 2, (size.height as i32  - PAMI_WIDTH) / 2);
        let area_height = size.height as i32 / 8;

        let pami_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let area_style = PrimitiveStyle::with_stroke(BinaryColor::On, 2);

        Rectangle::new(pami_origin, Size::new(PAMI_HEIGHT as u32, PAMI_WIDTH as u32))
            .into_styled(pami_style)
            .draw(display).unwrap();
        Rectangle::new(pami_origin, Size::new((PAMI_HEIGHT - PAMI_SEMI_HEIGHT) as u32, PAMI_WIDTH as u32))
            .into_styled(pami_style)
            .draw(display).unwrap();

        for i in 0i32..8i32 {
            let x_min = (offset.x + 1) as f32;
            let x_max = (pami_origin.x - 1) as f32;
            let d_max = 500.0 as f32;
            let d = self.back_distance[i as usize] as f32;

            let x = x_min + ((x_max - x_min) * (d_max - d)) / d_max;
            if (x >= x_min) && (x <= x_max) {
                let x = x.floor() as i32;
                Line::new(
                    Point::new(x,  offset.y + area_height * i),
                    Point::new(x,  offset.y + area_height * (i + 1)),
                )
                    .into_styled(area_style)
                    .draw(display).unwrap();
            }
        }
    }

    fn handle_event(&mut self, event: &Event) {
        match event {
            Event::BackDistance { distance } => {
                self.back_distance = *distance;
            }
            Event::TrajectoryStatus { opponent_detected } => {
                self.triggered = *opponent_detected;

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
        "Tof"
    }
}