use std::time::{Duration, Instant};
use asserv::differential::{conf::*, Asserv, rome::AsservDiffRome};
use asserv::rome::AsservRome;
use board_common::{Color, Periodicity};
use board_pami::{BatteryLevel, BatteryReader, PamiBoard, PamiButtons, PamiLeds, PamiPwmController};
use embedded_hal::digital::StatefulOutputPin;
use flume::{Receiver, Sender};
use vlx::VlxSensor;
use crate::pami_asserv::{ASSERV_PERIOD, PamiAsservHardware};
use crate::events::*;


/// Everything needed for PAMI routines
///
/// Update state from multiple peripherals.
/// States are updated when calling `idle()` or `step_idle()`.
pub struct PamiRoutines<B: PamiBoard> {
    pub asserv: Asserv<PamiAsservHardware<B>>,

    // Current state of various updated values
    pub emergency_stop: bool,
    pub battery_level: BatteryLevel,

    // Peripherals
    pub pami_leds: PamiLeds<B::Led>,
    pub pami_buttons: B::Buttons,
    pub battery_reader: B::BatteryReader,
    pub pwm_controller: PamiPwmController<B::I2c>,
    pub vlx: B::Vlx,
    starting_cord_read: Box<dyn FnMut() -> bool>,

    // ROME sender/receiver
    pub rome_tx: Sender<Box<[u8]>>,
    pub rome_rx: Receiver<Box<[u8]>>,
    // UI sender/receiver
    pub ui_events: Sender<UiEvent>,
    pub ui_triggers: Receiver<UiTrigger>,

    // Periodicity states
    asserv_periodicity: Periodicity,
    asserv_tm_periodicity: Periodicity,
    battery_level_periodicity: Periodicity,
}

impl<B: PamiBoard> PamiRoutines<B> {
    const EMERGENCY_STOP_COLOR: Color = Color::RED;

    /// Initialize with default state values and peripherals from board
    ///
    /// Peripherals must be available on the board.
    /// The asserv must be configured manually, using `asserv.set_conf()`.
    pub fn new(
        board: &mut B,
        (rome_tx, rome_rx): (Sender<Box<[u8]>>, Receiver<Box<[u8]>>),
        (ui_events, ui_triggers): (Sender<UiEvent>, Receiver<UiTrigger>),
    ) -> Self {
        let mut vlx = board.vlx_sensor().unwrap();
        log::info!("Initialize VLX");
        if let Err(err) = vlx.init() {
            log::error!("VLX initalization failed: {err:?}");
        }

        let mut pwm_controller = board.pwm_controller().unwrap();
        pwm_controller.init().unwrap();

        Self {
            asserv: Asserv::new(PamiAsservHardware::new(board), ASSERV_PERIOD),
            emergency_stop: false,
            battery_level: Default::default(),

            pami_leds: board.leds().unwrap(),
            pami_buttons: board.buttons().unwrap(),
            battery_reader: board.battery_reader().unwrap(),
            pwm_controller,
            vlx,
            starting_cord_read: board.starting_cord().unwrap(),

            rome_tx,
            rome_rx,
            ui_events,
            ui_triggers,

            asserv_periodicity: Periodicity::new(ASSERV_PERIOD),
            asserv_tm_periodicity: Periodicity::new(Duration::from_millis(100)),
            battery_level_periodicity: Periodicity::new(Duration::from_millis(2000)),
        }
    }

    /// Intialize states and peripherals
    pub fn init(&mut self) {
        self.pwm_controller.set_battery_rgb(&Color::new(0.0, 0.0, 0.02));
        self.pwm_controller.set_ground_rgb(&Color::new(0.02, 0.02, 0.02));
    }

    /// Run the match setup procedure
    pub fn match_setup(&mut self) -> MatchConf {
        let mut ground_led_color = BlinkingColor::new(Duration::from_millis(1000));

        log::info!("Unplug starting cord...");
        self.ui_events.send(UiEvent::ShowMessage("UNPLUG\nCORD")).unwrap();

        ground_led_color.set_colors(Color::MAGENTA, Color::BLACK);
        while (self.starting_cord_read)() {
            let now = self.step_idle();
            if let Some(color) = ground_led_color.update(&now) {
                self.set_ground_led_color(color);
            }
        }

        // Initialize match conf from buttons state
        let mut buttons = self.pami_buttons.read_state();
        let mut match_conf = MatchConf {
            team: match buttons.switch(0) {
                false => Team::Left,
                true => Team::Right,
            },
            start_delay: match buttons.switch(1) {
                false => 90,
                true => 3,
            },
            role: match (buttons.switch(2), buttons.switch(3)) {
                (_, true) => PamiRole::None,
                (false, false) => PamiRole::Granary,
                (true, false) => PamiRole::Land,
            },
        };

        const fn role_color(role: PamiRole) -> Color {
            match role {
                PamiRole::None => Color::BLACK,
                PamiRole::Granary => Color::GREEN,
                PamiRole::Land => Color::MAGENTA,
            }
        }

        log::info!("Configure PAMI for match");
        self.ui_events.send(UiEvent::ShowMatchConf(match_conf.clone())).unwrap();

        let mut buttons_periodicity = Periodicity::new(Duration::from_millis(200));
        let mut vlx_period = Periodicity::new(Duration::from_millis(2000));
        //TODO Use the other color for role
        ground_led_color.set_colors(role_color(match_conf.role), match_conf.team.color());

        while !(self.starting_cord_read)() {
            let now = self.step_idle();
            if let Some(color) = ground_led_color.update(&now) {
                self.set_ground_led_color(color);
            }

            // Dpad update
            if buttons_periodicity.update(&now) {
                // Don't handle switch changes after init, only dpad for UI
                let new_buttons = self.pami_buttons.read_state();
                if new_buttons.dpad != buttons.dpad {
                    buttons = new_buttons;
                    self.ui_events.send(UiEvent::Dpad(new_buttons.dpad)).unwrap();
                }
            }

            // UI events
            while let Ok(trigger) = self.ui_triggers.try_recv() {
                match trigger {
                    UiTrigger::ChangeTeam(value) => {
                        match_conf.team = value;
                        self.ui_events.send(UiEvent::ChangeTeam(value)).unwrap();
                        ground_led_color.set_colors(role_color(match_conf.role), match_conf.team.color());
                    }
                    UiTrigger::ChangeStartDelay(value) => {
                        match_conf.start_delay = value;
                        self.ui_events.send(UiEvent::ChangeStartDelay(value)).unwrap();
                    }
                    UiTrigger::ChangeRole(value) => {
                        match_conf.role = value;
                        self.ui_events.send(UiEvent::ChangeRole(value)).unwrap();
                        ground_led_color.set_colors(role_color(match_conf.role), match_conf.team.color());
                    }
                }
            }

            // VLX capture
            if vlx_period.update(&now) {
                match self.vlx.get_distance() {
                    Ok(distance) => log::info!("VLX sensor distance: {distance:?}"),
                    Err(err) => log::error!("Failed to get VLX sensor distance: {err:?}"),
                }
            }
        }

        log::info!("configured: team={} delay={} role={:?}", match_conf.team.name(), match_conf.start_delay, match_conf.role);
        match_conf
    }

    /// Wait for match start
    pub fn wait_match_start(&mut self, match_conf: &MatchConf) {
        let mut ground_led_color = BlinkingColor::new(Duration::from_millis(1000));

        log::info!("Waiting match start...");
        self.ui_events.send(UiEvent::ShowMessage("WAITING\nSTART")).unwrap();

        ground_led_color.set_colors(Color::MAGENTA, match_conf.team.color());
        while (self.starting_cord_read)() {
            let now = self.step_idle();
            if let Some(color) = ground_led_color.update(&now) {
                self.set_ground_led_color(color);
            }
        }

        log::info!("Match starts!");
        self.set_ground_led_color(&Color::BLACK);
    }


    /// Execute one round of idle updates
    ///
    /// This method must be called regularly.
    pub fn idle(&mut self, now: &Instant) {
        let _ = self.pami_leds.esp.toggle();

        // Emergency stop: check whenever possible
        let new_emergency_stop = self.asserv.hardware_mut().emergency_stop_active();
        if new_emergency_stop != self.emergency_stop {
            log::info!("Emergency stop changed: {new_emergency_stop}");
            self.ui_events.send(UiEvent::EmergencyStop(new_emergency_stop)).unwrap();
            self.pwm_controller.set_ground_rgb(&Self::EMERGENCY_STOP_COLOR);
            self.emergency_stop = new_emergency_stop;
        }

        // Process ROME input messages
        for data in self.rome_rx.try_iter() {
            let _ = self.pami_leds.com.toggle();
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !self.asserv.on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }

        // Update asserv, send asserv telemetry
        if self.asserv_periodicity.update(now) {
            self.asserv.update(self.asserv_periodicity.period());
        }
        if self.asserv_tm_periodicity.update(now) {
            if let Err(err) = self.rome_tx.send(self.asserv.asserv_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Err(err) = self.rome_tx.send(self.asserv.asserv_diff_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
        }

        // Battery level, send update to ROME and UI
        if self.battery_level_periodicity.update(now) {
            let level = self.battery_reader.read_vbatt();

            let message = rome::Message::BatteryLevel { mv: level.mv, percent: level.percent };
            if let Err(err) = self.rome_tx.send(message.encode()) {
                log::error!("BLE send error: {:?}", err);
            }

            if level.percent != self.battery_level.percent {
                log::info!("Battery: {} mV, {}%", level.mv, level.percent);
                self.ui_events.send(UiEvent::Battery { percent: level.percent }).unwrap();
                self.pwm_controller.set_battery_rgb(&Self::battery_led_color(level.percent));
            }

            self.battery_level = level;
        }
    }

    /// Wait for the next asserv step, the run `idle()` and return the associated instant
    pub fn step_idle(&mut self) -> Instant {
        let mut now = Instant::now();
        let next_instant = self.asserv_periodicity.next();
        if let Some(duration) = next_instant.checked_duration_since(now) {
            std::thread::sleep(duration);
            now = *next_instant;
        }
        self.idle(&now);
        now
    }

    /// Update ground led color, override with red if emergency stop is active
    fn set_ground_led_color(&mut self, color: &Color) {
        if self.emergency_stop {
            self.pwm_controller.set_ground_rgb(&Self::EMERGENCY_STOP_COLOR);
        } else {
            self.pwm_controller.set_ground_rgb(color);
        }
    }

    /// Return battery led color to use
    const fn battery_led_color(percent: u8) -> Color {
        if percent < 10 {
            Color::new(1.0, 0.0, 0.0)
        } else if percent < 30 {
            Color::new(0.5, 0.2, 0.0)
        } else {
            Color::BLACK
        }
    }
}


struct BlinkingColor {
    periodicity: Periodicity,
    colors: [Color; 2],
    state: bool,
}

impl BlinkingColor {
    pub fn new(period: Duration) -> Self {
        Self {
            periodicity: Periodicity::new(period),
            colors: [Color::BLACK, Color::BLACK],
            state: false
        }
    }

    pub fn set_colors(&mut self, color1: Color, color2: Color) {
        self.colors = [color1, color2];
    }

    pub fn update(&mut self, now: &Instant) -> Option<&Color> {
        if self.periodicity.update(now) {
            self.state = !self.state;
            Some(&self.colors[if self.state { 0 } else { 1}])
        } else {
            None
        }
    }
}

