use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, Instant};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::{PrimitiveStyle, Rectangle},
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};

pub trait UiTarget: DrawTarget {
    fn flush_display(&mut self) {}
}

#[cfg(target_os = "espidf")]    
use board_pami::DpadState;

#[cfg(target_os = "espidf")]    
impl UiTarget for board_pami::Display {
    fn flush_display(&mut self) {
        self.flush().unwrap()
    }
}


pub enum UiEvent {
    Battery { percent: u8 },
    DistanceSensors { first: u16, back: u16, front: u16 },
    Dpad(DpadState),
}


const DISPLAY_WIDTH: u32 = 128;
const DISPLAY_HEIGHT: u32 = 64;
const YELLOW_HEIGHT: u32 = 16;
const TOP_BAR_SIZE: u32 = 9;

/// Minimum duration between two screen refresh
const MIN_REFRESH_DELAY: Duration = Duration::from_millis(250);


pub struct Ui<T: UiTarget<Color = BinaryColor> + Send + 'static> {
    target: T,
    //XXX Temporary; should store a list of screens
    screens: Vec<Box<dyn UiScreen<T>>>,
    current_screen_index: usize,
    display_is_dirty: bool,
    last_refresh: Instant,
    battery_percent: Option<u8>,
}

impl<T: UiTarget<Color = BinaryColor> + Send + 'static> Ui<T> where T::Error: std::fmt::Debug {
    /// Create and run UI in a dedicated thread, return event queue to send messages
    pub fn run(target: T) -> Sender<UiEvent> {
        let (sender, receiver): (Sender<UiEvent>, Receiver<UiEvent>) = mpsc::channel();
        //TODO Tweak stack size?
        std::thread::Builder::new().stack_size(16 * 8192).name("UI".to_string()).spawn(move || {
            let mut instance = Self {
                target,
                screens: vec![
                    Box::new(DebugScreen::default()),
                ],
                current_screen_index: 0,
                display_is_dirty: false,
                last_refresh: Instant::now(),
                battery_percent: None,
            };

            loop {
                match receiver.recv() {
                    Ok(event) => {
                        instance.handle_event(event);
                    }
                    Err(_) => {},  // Ignore
                }
            }
        }).unwrap();

        sender
    }

    fn handle_event(&mut self, event: UiEvent) {
        self.display_is_dirty |= self.on_event(&event);
        for (i, screen) in self.screens.iter_mut().enumerate() {
            if screen.on_event(&event) {
                if i == self.current_screen_index {
                    self.display_is_dirty = true;
                }
            }
        }

        if self.display_is_dirty && self.last_refresh.elapsed() > MIN_REFRESH_DELAY {
            self.draw_decorations().unwrap();
            if let Some(screen) = self.screens.get(self.current_screen_index) {
                screen.draw(&mut self.target).unwrap();
            }
            self.target.flush_display();
        }
    }

    fn on_event(&mut self, event: &UiEvent) -> bool {
        match event {
            UiEvent::Battery { percent } => {
                self.battery_percent = Some(*percent);
                true
            }
            _ => false
        }
    }

    /// Display screen decorations (top bar), clear middle part
    fn draw_decorations(&mut self) -> Result<(), T::Error> {
        Rectangle::new(Point::new(0, 0), Size::new(DISPLAY_WIDTH, TOP_BAR_SIZE))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut self.target)?;

        Rectangle::new(Point::new(0, TOP_BAR_SIZE as i32), Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT - TOP_BAR_SIZE))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut self.target)?;

        let character_style = MonoTextStyleBuilder::new()
            .font(&ascii::FONT_5X7)
            .text_color(BinaryColor::Off)
            .build();

        let text_style_left = TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Left)
            .build();

        let text_style_right = TextStyleBuilder::new()
            .baseline(Baseline::Top)
            .alignment(Alignment::Right)
            .build();

        if let Some(screen) = self.screens.get(self.current_screen_index) {
            Text::with_text_style(
                screen.name(),
                Point::new(2, 1),
                character_style,
                text_style_left,
            ).draw(&mut self.target)?;
        }

        if let Some(percent) = self.battery_percent {
            Text::with_text_style(
                &format!("{percent:.0}%"),
                Point::new(DISPLAY_WIDTH as i32 - 2, 1),
                character_style,
                text_style_right
            ).draw(&mut self.target)?;
        }

        Ok(())
    }
}


trait UiScreen<T: DrawTarget> {
    /// Screen name, displayed at the top
    fn name(&self) -> &'static str;
    /// Draw the screen
    fn draw(&self, target: &mut T) -> Result<(), T::Error>;
    /// Called on new UI event, return true if something changed
    fn on_event(&mut self, event: &UiEvent) -> bool;
}

//TODO Debug screen, for tests
#[derive(Default)]
struct DebugScreen {
    text: String
}

impl<T: DrawTarget<Color = BinaryColor>> UiScreen<T> for DebugScreen {
    fn name(&self) -> &'static str {
        "Debug"
    }

    fn draw(&self, target: &mut T) -> Result<(), T::Error> {
        let character_style = MonoTextStyleBuilder::new()
            .font(&ascii::FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        let text_style = TextStyleBuilder::new()
            .baseline(Baseline::Middle)
            .alignment(Alignment::Center)
            .build();

        Text::with_text_style(
            &self.text,
            Point::new(DISPLAY_WIDTH as i32 / 2, DISPLAY_HEIGHT as i32 / 2),
            character_style,
            text_style,
        ).draw(target)?;
        Ok(())
    }

    fn on_event(&mut self, event: &UiEvent) -> bool {
        match event {
            UiEvent::Dpad(dpad) => {
                self.text.clear();
                self.text.push(dpad.as_char());
                true
            }
            _ => false,
        }
    }
}
