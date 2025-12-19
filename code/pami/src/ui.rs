use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, Instant};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    prelude::*,
    text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder},
};
use board_common::Team;
use board_pami::DpadState;
use crate::events::{UiEvent, UiPamiMode, UiTrigger};


const DISPLAY_WIDTH: u32 = 128;
const DISPLAY_HEIGHT: u32 = 64;
#[allow(dead_code)]
const YELLOW_HEIGHT: u32 = 16;
const TOP_BAR_SIZE: u32 = YELLOW_HEIGHT;

/// Minimum duration between two screen refresh
const MIN_REFRESH_DELAY: Duration = Duration::from_millis(250);


/// Result of a screen event
enum ScreenEventResult {
    /// Nothing happened
    None,
    /// Display has been updated and is now dirty
    Updated,
    /// Exit current screen
    Exit,
    /// Trigger event to be propagated to the user
    Trigger(UiTrigger),
}


pub struct Ui<T: DrawTarget<Color = BinaryColor>> {
    name: &'static str,
    target: T,
    /// Screen stack, must never be empty
    ///
    /// For now, use a stack. The first item is the main screen.
    /// The last item is the active screen.
    /// It can be extended later to have multiple "persistent" screens.
    screens: Vec<Box<dyn Screen<T>>>,
    display_is_dirty: bool,
    next_refresh: Instant,
    event_receiver: Receiver<UiEvent>,
    trigger_sender: Sender<UiTrigger>,
}

trait Screen<T: DrawTarget<Color = BinaryColor>> {
    /// Draw the initial state of the screen
    fn draw_init(&self, target: &mut T) -> Result<(), T::Error>;
    /// Called on new UI event when screen is active
    fn on_event_active(&mut self, event: &UiEvent, target: &mut T) -> Result<ScreenEventResult, T::Error>;
    /// Called on new UI event when screen is not active
    fn on_event_inactive(&mut self, _event: &UiEvent) {}
}

impl<T: DrawTarget<Color = BinaryColor> + Send + 'static> Ui<T> where T::Error: std::fmt::Debug {
    /// Create and run UI in a dedicated thread, return queues to send events and receive triggers
    pub fn run_thread<F: Fn(&mut T) + Send + 'static>(name: &'static str, target: T, flush: F) -> (Sender<UiEvent>, Receiver<UiTrigger>) {
        let (event_sender, event_receiver) = mpsc::channel::<UiEvent>();
        let (trigger_sender, trigger_receiver) = mpsc::channel::<UiTrigger>();
        //TODO Tweak stack size?
        std::thread::Builder::new().stack_size(16 * 8192).name("UI".to_string()).spawn(move || {
            let mut instance = Self::new(name, target, event_receiver, trigger_sender);
            loop {
                match instance.step() {
                    None => log::error!("UI event receiver closed"),
                    Some(true) => flush(&mut instance.target),
                    Some(false) => {},
                }
            }
        }).unwrap();

        (event_sender, trigger_receiver)
    }
}

impl<T: DrawTarget<Color = BinaryColor>> Ui<T> where T::Error: std::fmt::Debug {
    /// Create a new instance
    ///
    /// This method is intended to be used by simulators, to all `step()` manually.
    pub fn new(name: &'static str, target: T, event_receiver: Receiver<UiEvent>, trigger_sender: Sender<UiTrigger>) -> Self {
        let mut instance = Self {
            name,
            target,
            screens: Vec::new(),
            display_is_dirty: false,
            next_refresh: Instant::now(),
            event_receiver,
            trigger_sender,
        };
        instance.draw_init().unwrap();
        instance.push_screen(Box::new(MainScreen::default())).unwrap();
        instance
    }

    pub fn target(&mut self) -> &mut T {
        &mut self.target
    }

    /// Run a single UI step
    ///
    /// Return `true` if display needs a flush.
    /// Return `None` if event receiver is disconnected.
    pub fn step(&mut self) -> Option<bool> {
        let event = if let Some(delay) = self.step_delay() {
            match self.event_receiver.recv_timeout(delay) {
                Ok(event) => Some(event),
                Err(mpsc::RecvTimeoutError::Timeout) => None,
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    return None;
                }
            }
        } else {
            self.event_receiver.recv().ok()
        };
        Some(self.update(event))
    }

    /// Run a single UI step, don't block
    ///
    /// Return `true` if display needs a flush.
    pub fn try_step(&mut self) -> bool {
        self.update(self.event_receiver.try_recv().ok())
    }

    /// Update UI if needed, return `true` if display needs a flush
    fn update(&mut self, event: Option<UiEvent>) -> bool {
        // Process event, if any
        if let Some(event) = event {
            self.process_event(&event).unwrap();
        }

        // Refresh display if needed
        let mut refreshed = false;
        if self.display_is_dirty {
            let now = Instant::now();
            if now >= self.next_refresh {
                //TODO Is there something to do?
                self.next_refresh = now + MIN_REFRESH_DELAY;
                self.display_is_dirty = false;
                refreshed = true;
            }
        }
        refreshed
    }

    /// Return duration before next planned event, if any.
    /// Can be used for animation or other for time-based events.
    pub fn step_delay(&mut self) -> Option<Duration> {
        if self.display_is_dirty {
            Some(self.next_refresh - Instant::now())
        } else {
            None
        }
    }

    /// Process an event, propagate to screen, update `display_is_dirty`
    fn process_event(&mut self, event: &UiEvent) -> Result<(), T::Error> {
        self.display_is_dirty |= self.on_event(event)?;
        for screen in self.screens.iter_mut().skip(1) {
            screen.on_event_inactive(event);
        }
        if let Some(screen) = self.screens.last_mut() {
            let result = screen.on_event_active(event, &mut self.target)?;
            match result {
                ScreenEventResult::None => {},
                ScreenEventResult::Updated => {
                    self.display_is_dirty = true;
                }
                ScreenEventResult::Exit => {
                    self.pop_screen()?;
                },
                ScreenEventResult::Trigger(trigger) => {
                    let _ = self.trigger_sender.send(trigger);
                }
            }
        }
        Ok(())
    }


    const NAME_CHARACTER_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_7X14_BOLD)
        .text_color(BinaryColor::Off)
        .build();
    const NAME_TEXT_STYLE: TextStyle = TextStyleBuilder::new()
        .baseline(Baseline::Top)
        .alignment(Alignment::Left)
        .build();
    const BATTERY_CHARACTER_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_8X13_BOLD)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();
    const BATTERY_TEXT_STYLE: TextStyle = TextStyleBuilder::new()
        .baseline(Baseline::Top)
        .alignment(Alignment::Right)
        .build();
    const STOP_CHARACTER_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_6X13_BOLD)
        .text_color(BinaryColor::On)
        .build();
    const STOP_TEXT_STYLE: TextStyle = TextStyleBuilder::new()
        .baseline(Baseline::Top)
        .alignment(Alignment::Left)
        .build();


    /// Process event for global UI elements, as a screen would
    fn on_event(&mut self, event: &UiEvent) -> Result<bool, T::Error> {
        match event {
            UiEvent::Battery { percent } => {
                self.draw_battery_usage(*percent)?;
                Ok(true)
            }
            UiEvent::EmergencyStop(enabled) => {
                self.draw_emergency_stop(*enabled)?;
                Ok(true)
            }
            UiEvent::KeypassNotif(key) => {
                self.push_screen(Box::new(PasskeyScreen::new(*key)))?;
                Ok(true)
            }
            _ => Ok(false)
        }
    }

    /// Push a new screen and switch to it
    fn push_screen(&mut self, screen: Box<dyn Screen<T>>) -> Result<(), T::Error> {
        self.clear_screen_area()?;
        screen.draw_init(&mut self.target)?;
        self.screens.push(screen);
        self.display_is_dirty = true;
        Ok(())
    }

    /// Pop a screen, switch to the new top element
    fn pop_screen(&mut self) -> Result<(), T::Error> {
        if self.screens.len() > 1 {
            self.screens.pop();
            self.clear_screen_area()?;
            self.screens.last().unwrap().draw_init(&mut self.target)?;
            self.display_is_dirty = true;
        }
        Ok(())
    }

    /// Draw the initial state of the top bar
    fn draw_init(&mut self) -> Result<(), T::Error> {
        Rectangle::new(Point::new(0, 0), Size::new(DISPLAY_WIDTH, TOP_BAR_SIZE))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut self.target)?;
        Text::with_text_style(
            self.name,
            Point::new(2, 1),
            Self::NAME_CHARACTER_STYLE,
            Self::NAME_TEXT_STYLE,
        ).draw(&mut self.target)?;
        Ok(())
    }

    /// Clear the screen surface, under the top bar
    fn clear_screen_area(&mut self) -> Result<(), T::Error> {
        Rectangle::new(Point::new(0, TOP_BAR_SIZE as i32), Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT - TOP_BAR_SIZE))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut self.target)
    }

    /// Draw battery usage
    fn draw_battery_usage(&mut self, percent: u8) -> Result<(), T::Error> {
        Text::with_text_style(
            &format!("{percent:3}%"),
            Point::new(DISPLAY_WIDTH as i32 - 2, 2),
            Self::BATTERY_CHARACTER_STYLE,
            Self::BATTERY_TEXT_STYLE,
        ).draw(&mut self.target)?;
        Ok(())
    }

    /// Draw emergency stop status
    fn draw_emergency_stop(&mut self, enabled: bool) -> Result<(), T::Error> {
        const LEFT_POS: i32 = DISPLAY_WIDTH as i32 - 61;
        Rectangle::new(Point::new(LEFT_POS, 1), Size::new(28, 14))
            .into_styled(PrimitiveStyle::with_fill(if enabled { BinaryColor::Off } else { BinaryColor::On }))
            .draw(&mut self.target)?;
        if enabled {
            Text::with_text_style(
                "STOP",
                Point::new(LEFT_POS + 2, 2),
                Self::STOP_CHARACTER_STYLE,
                Self::STOP_TEXT_STYLE,
            ).draw(&mut self.target)?;
        }
        Ok(())
    }
}


struct MainScreen {
    team: Team,
    mode: UiPamiMode,
    focused: Option<u8>,
}

impl Default for MainScreen {
    fn default() -> Self {
        Self {
            team: Team::None,
            mode: UiPamiMode::Match,
            // Start unfocused, it smoother for the eye
            focused: None,
        }
    }
}

impl MainScreen {
    const COLS: u8 = 2;
    const ROWS: u8 = 3;
    const CELLS: u8 = 3;

    const BOX_WIDTH: u32 = DISPLAY_WIDTH / Self::COLS as u32;
    const BOX_HEIGHT: u32 = (DISPLAY_HEIGHT - TOP_BAR_SIZE) / Self::ROWS as u32;
    const BOX_SIZE: Size = Size::new(Self::BOX_WIDTH, Self::BOX_HEIGHT);

    const BOX_BG_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();
    const BOX_FOCUSED_BG_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();
    const BOX_CHARACTER_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_9X15_BOLD)
        .text_color(BinaryColor::On)
        .build();
    const BOX_CHARACTER_FOCUSED_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_9X15_BOLD)
        .text_color(BinaryColor::Off)
        .build();
    const BOX_TEXT_STYLE: TextStyle = TextStyleBuilder::new()
        .baseline(Baseline::Bottom)
        .alignment(Alignment::Center)
        .build();

    fn mode_label(&self) -> &'static str {
        match self.mode {
            UiPamiMode::Match => "MATCH",
            UiPamiMode::QuickStart => "RAPIDE",
            UiPamiMode::Debug => "DEBUG",
        }
    }

    fn press_cell_result(&mut self, index: u8) -> ScreenEventResult {
        match index {
            0 => {
                let team = match self.team {
                    Team::None => Team::Left,
                    Team::Left => Team::Right,
                    Team::Right => Team::Left,
                };
                ScreenEventResult::Trigger(UiTrigger::ChangeTeam(team))
            },
            1 => {
                let mode = match self.mode {
                    UiPamiMode::Match => UiPamiMode::QuickStart,
                    UiPamiMode::QuickStart => UiPamiMode::Debug,
                    UiPamiMode::Debug => UiPamiMode::Match,
                };
                ScreenEventResult::Trigger(UiTrigger::ChangeMode(mode))
            },
            2 => {
                //TODO Require a long press
                ScreenEventResult::Trigger(UiTrigger::Reboot)
            }
            _ => {
                // Should not happen
                ScreenEventResult::None
            }
        }
    }

    /// Redraw all text boxes
    ///
    /// Drawing all boxes could be avoided on refresh.
    fn draw_boxes<T: DrawTarget<Color = BinaryColor>>(&self, target: &mut T) -> Result<(), T::Error> {
        const { assert!(Self::CELLS <= Self::COLS * Self::ROWS); };

        let box_labels: [&'static str; Self::CELLS as usize] = [
            self.team.name_upper(),
            self.mode_label(),
            "REBOOT",
        ];
        for (i, label) in box_labels.iter().enumerate() {
            let focused = Some(i as u8) == self.focused;
            if focused {
                Rectangle::new(Self::box_origin(i as u8), Self::BOX_SIZE)
                    .into_styled(Self::BOX_FOCUSED_BG_STYLE)
                    .draw(target)?;
            }
            Text::with_text_style(
                label,
                Self::box_text_center(i as u8),
                if focused { Self::BOX_CHARACTER_FOCUSED_STYLE } else { Self::BOX_CHARACTER_STYLE },
                Self::BOX_TEXT_STYLE,
            ).draw(target)?;
        }

        Ok(())
    }

    const fn box_text_center(index: u8) -> Point {
        let x = index % Self::COLS;
        let y = index / Self::COLS;
        let x0 = x as u32 * Self::BOX_WIDTH + Self::BOX_WIDTH / 2;
        let y0 = TOP_BAR_SIZE + y as u32 * Self::BOX_HEIGHT + Self::BOX_HEIGHT - 1;
        Point::new(x0 as i32, y0 as i32)
    }

    const fn box_origin(index: u8) -> Point {
        let x = index % Self::COLS;
        let y = index / Self::COLS;
        let x0 = x as u32 * Self::BOX_WIDTH;
        let y0 = TOP_BAR_SIZE + y as u32 * Self::BOX_HEIGHT;
        Point::new(x0 as i32, y0 as i32)
    }
}

impl<T: DrawTarget<Color = BinaryColor>> Screen<T> for MainScreen {
    fn draw_init(&self, target: &mut T) -> Result<(), T::Error> {
        self.draw_boxes(target)
    }

    fn on_event_active(&mut self, event: &UiEvent, target: &mut T) -> Result<ScreenEventResult, T::Error> {
        let mut new_focused = self.focused;
        let updated = match event {
            UiEvent::Dpad(dpad) => {
                if let Some(focused) = self.focused {
                    // Note: empty cells are selectable; it could be improved but it's complicated
                    let current_x = focused % Self::COLS;
                    let current_y = focused / Self::COLS;
                    match dpad {
                        DpadState::None => {
                            return Ok(ScreenEventResult::None);
                        },
                        DpadState::Middle => {
                            return Ok(self.press_cell_result(focused));
                        }
                        DpadState::Up => {
                            let max_y = (Self::CELLS - current_x).div_ceil(Self::COLS).min(Self::ROWS);
                            let new_y = if current_y == 0 { max_y - 1 } else { current_y - 1 };
                            new_focused = Some(current_x + new_y * Self::COLS);
                        }
                        DpadState::Down => {
                            let max_y = (Self::CELLS - current_x).div_ceil(Self::COLS).min(Self::ROWS);
                            let new_y = if current_y >= max_y - 1 { 0 } else { current_y + 1 };
                            new_focused = Some(current_x + new_y * Self::COLS);
                        }
                        DpadState::Left => {
                            let max_x = (Self::CELLS - (current_y * Self::COLS)).min(Self::COLS);
                            let new_x = if current_x == 0 { max_x - 1 } else { current_x - 1 };
                            new_focused = Some(new_x + current_y * Self::COLS);
                        }
                        DpadState::Right => {
                            let max_x = (Self::CELLS - (current_y * Self::COLS)).min(Self::COLS);
                            let new_x = if current_x >= max_x - 1 { 0 } else { current_x + 1 };
                            new_focused = Some(new_x + current_y * Self::COLS);
                        }
                    }
                } else if *dpad != DpadState::None {
                    new_focused = Some(0);
                }
                true
            }
            UiEvent::ChangeTeam(team) => {
                self.team = *team;
                true
            }
            UiEvent::ChangeMode(mode) => {
                self.mode = *mode;
                true
            }
            _ => false,
        };
        if updated {
            // Clear the rectangle of the previously focused box
            if self.focused != new_focused {
                if let Some(focused) = self.focused {
                    Rectangle::new(Self::box_origin(focused), Self::BOX_SIZE)
                        .into_styled(Self::BOX_BG_STYLE)
                        .draw(target)?;
                }
                self.focused = new_focused;
            }
            self.draw_boxes(target)?;
            Ok(ScreenEventResult::Updated)
        } else {
            Ok(ScreenEventResult::None)
        }
    }

    fn on_event_inactive(&mut self, event: &UiEvent) {
        match event {
            UiEvent::ChangeTeam(team) => {
                self.team = *team;
            }
            UiEvent::ChangeMode(mode) => {
                self.mode = *mode;
            }
            _ => {},
        }
    }
}


/// Display a Bluetooth passkey
#[derive(Default)]
struct PasskeyScreen {
    key: u32,
}

impl PasskeyScreen {
    const CHARACTER_STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&ascii::FONT_10X20)
        .text_color(BinaryColor::On)
        .build();
    const TEXT_STYLE: TextStyle = TextStyleBuilder::new()
        .baseline(Baseline::Middle)
        .alignment(Alignment::Center)
        .build();

    fn new(key: u32) -> Self {
        Self { key }
    }
}

impl<T: DrawTarget<Color = BinaryColor>> Screen<T> for PasskeyScreen {
    fn draw_init(&self, target: &mut T) -> Result<(), T::Error> {
        Text::with_text_style(
            &format!("{:06}", self.key),
            Point::new(DISPLAY_WIDTH as i32 / 2, DISPLAY_HEIGHT as i32 / 2),
            Self::CHARACTER_STYLE,
            Self::TEXT_STYLE,
        ).draw(target)?;
        Ok(())
    }

    fn on_event_active(&mut self, event: &UiEvent, _target: &mut T) -> Result<ScreenEventResult, T::Error> {
        match event {
            UiEvent::Dpad(dpad) if *dpad != DpadState::None => {
                Ok(ScreenEventResult::Exit)
            }
            _ => Ok(ScreenEventResult::None)
        }
    }
}
