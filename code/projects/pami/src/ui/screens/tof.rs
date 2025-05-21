use std::time::Instant;

use board_pami_2023::DisplayType;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{Line, PrimitiveStyle, Rectangle}};

use crate::{events::Event, ui::UIScreen};

pub struct Tof {
    last_trigger: Option<Instant>,
    back_distance: [i16; 8],
}

impl Tof {
    pub fn new() -> Self {
        Self {
            last_trigger: None,
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

                let mut alert = false;
                for i in 0..self.back_distance.len() {
                    if self.back_distance[i] < 100 {
                        alert = true;
                        break;
                    }
                }

                if alert {
                    if self.last_trigger.is_none() {
                        self.last_trigger = Some(Instant::now());
                    } else {
                        let elapsed = self.last_trigger.unwrap().elapsed();
                        if elapsed.as_secs() > 10 {
                            log::info!("Alert triggered!");
                            self.last_trigger = Some(Instant::now());
                        }
                    }
                }
                else {
                    self.last_trigger = None;
                }
            }
            _ => {}
        }
    }

    fn get_priority(&self) -> isize {
        if let Some(last_trigger) = self.last_trigger {
            if last_trigger.elapsed().as_secs() < 7 {
                return 1;
            }
        }

        return -1;
    }

    fn get_screen_name(&self) -> &'static str {
        "Tof"
    }
}