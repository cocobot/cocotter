use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use asserv::differential::{conf::*, Asserv};
use asserv::{rome::AsservRome, differential::rome::AsservDiffRome};
use board_common::{Color, Periodicity};
use board_pami::{BatteryLevel, PamiBoard, PamiButtons, PamiButtonsState};
use vlx::VlxSensor;
use pami::config::PamiConfig;
use pami::events::*;
use pami::pami_asserv::{ASSERV_PERIOD, PamiAsservHardware};
use pami::ui::Ui;

#[cfg(target_os = "espidf")]
type PamiBoardImpl = board_pami::EspPamiBoard<'static>;
#[cfg(not(target_os = "espidf"))]
use board_pami::MockPamiBoard as PamiBoardImpl;

#[cfg(target_os = "espidf")]
fn flush_display(display: &mut board_pami::esp::PamiDisplay) {
    display.flush().unwrap();
}
#[cfg(not(target_os = "espidf"))]
fn flush_display(_display: &mut board_pami::mock::PamiDisplay) {}


/// Group various PAMI state values, for convenience
struct MainState {
    step: MainStep,
    battery_percent: u8,
    buttons: Option<PamiButtonsState>,
    emergency_stop: bool,
    starting_cord: bool,
    team: Team,
    start_delay: u8,
    role: PamiRole,
    ui_events: Sender<UiEvent>,
    update_ground_led: bool,
}

impl MainState {
    fn new(ui_events: Sender<UiEvent>) -> Self {
        Self {
            step: MainStep::Free,
            battery_percent: 0,
            buttons: None,
            emergency_stop: false,
            starting_cord: false,
            team: Team::Left,
            start_delay: 0,
            role: PamiRole::Granary,
            ui_events,
            update_ground_led: true,
        }
    }

    /// Return ground led color based on current state 
    fn ground_led_color(&self) -> Color {
        //TODO How to include PAMI's color?
        if self.emergency_stop {
            Color::new(1.0, 0.0, 0.0)
        } else {
            match self.step {
                MainStep::Free => Color::off(),
                MainStep::CordPlugged => self.team.color(),
                //TODO Change color based on strategy state, timer, ...
                MainStep::Match => Color::new(0.2, 0.2, 0.2),
            }
        }
    }

    /// Return battery led color to use
    fn battery_led_color(&self) -> Color {
        if self.battery_percent < 10 {
            Color::new(1.0, 0.0, 0.0)
        } else if self.battery_percent < 30 {
            Color::new(0.5, 0.2, 0.0)
        } else {
            Color::off()
        }
    }

    fn set_step(&mut self, value: MainStep) {
        self.step = value;
        self.update_ground_led = true;
    }

    fn set_battery_level(&mut self, mv: u16, percent: u8) {
        if percent == self.battery_percent {
            return;
        }
        log::info!("Battery: {mv} mV, {percent}%");
        self.battery_percent = percent;
        self.ui_events.send(UiEvent::Battery { percent }).unwrap();
    }

    fn set_buttons(&mut self, value: PamiButtonsState) {
        // Don't detect switch changes at runtime, only at init
        if let Some(previous) = self.buttons {
            if value != previous {
                let dpad = value.dpad;
                if dpad != previous.dpad {
                    self.ui_events.send(UiEvent::Dpad(dpad)).unwrap();
                }
            }
        } else {
            self.set_team(match value.switch(0) {
                false => Team::Left,
                true => Team::Right,
            });
            self.set_start_delay(match value.switch(1) {
                false => 90,
                true => 3,
            });
            self.set_role(match (value.switch(2), value.switch(3)) {
                (_, true) => PamiRole::None,
                (false, false) => PamiRole::Granary,
                (true, false) => PamiRole::Land,
            });
        }
        self.buttons = Some(value);
    }

    fn set_emergency_stop(&mut self, value: bool) {
        if value != self.emergency_stop {
            log::info!("Emergency stop changed: {value}");
            self.emergency_stop = value;
            self.ui_events.send(UiEvent::EmergencyStop(value)).unwrap();
            self.update_ground_led = true;
        }
    }

    /// Update starting cord state, return true if it actually changed (and possibly step changed too)
    fn set_starting_cord(&mut self, value: bool) -> bool {
        if value == self.starting_cord {
            return false;
        }
        log::info!("Starting cord state: {}", value);
        self.starting_cord = value;
        match (self.step, value) {
            (MainStep::Free, false) => {
                // Initial unplugging, no change
            }
            (MainStep::Free, true) => {
                // Starting cord plugging, preparing for match
                log::warn!("Ready for match, remove starting cord to start");
                self.set_step(MainStep::CordPlugged);
            }
            (MainStep::CordPlugged, false) => {
                log::warn!("Start match, team {}, delay {}s, role {:?}", self.team.name(), self.start_delay, self.role);
                self.set_step(MainStep::Match);
            }
            (MainStep::CordPlugged, true) => {
                unreachable!();
            }
            (MainStep::Match, _) => {
                // Ignore changes during a match
            }
        }
        return true;
    }

    fn set_team(&mut self, value: Team) {
        self.team = value;
        self.ui_events.send(UiEvent::ChangeTeam(value)).unwrap();
        self.update_ground_led = true;
    }

    fn set_start_delay(&mut self, value: u8) {
        self.start_delay = value;
        self.ui_events.send(UiEvent::ChangeStartDelay(value)).unwrap();
    }

    fn set_role(&mut self, value: PamiRole) {
        self.role = value;
        self.ui_events.send(UiEvent::ChangeRole(value)).unwrap();
        self.update_ground_led = true;
    }
}


fn main() {
    let mut board = PamiBoardImpl::init();

    let mac_addr = board.bt_mac_address();
    let config = if let Some(config) = PamiConfig::by_bt_mac(&mac_addr) {
        log::info!("Board '{}'", config.name);
        config
    } else {
        panic!("Board is not configured (MAC address: {mac_addr:?})");
    };

    log::info!("Start UI thread");
    let (ui_events, ui_triggers) = Ui::run_thread(config.name, board.display().unwrap(), flush_display);

    // Use a "enabled" flag to disable during a match
    let passkey_enabled = Arc::new(AtomicBool::new(true));
    let passkey_notifier = {
        let passkey_enabled = passkey_enabled.clone();
        let ui_events = ui_events.clone();
        move |_addr, key| {
            if passkey_enabled.load(Ordering::Relaxed) {
                ui_events.send(UiEvent::KeypassNotif(key)).unwrap();
            }
        }
    };
    let (rome_tx, rome_rx) = board.rome(config.name.into(), passkey_notifier).unwrap();

    // VLX
    let mut vlx = board.vlx_sensor().unwrap();
    log::info!("Initialize VLX");
    if let Err(err) = vlx.init() {
        log::error!("VLX initalization failed: {err:?}");
    }

    // Asserv
    let mut asserv = Asserv::new(PamiAsservHardware::new(&mut board), ASSERV_PERIOD);
    asserv.set_conf(AsservConf {
        pid_dist: PidConf {
            gain_p: 10,
            gain_i: 1,
            .. Default::default()
        },
        pid_angle: PidConf {
            gain_p: 200,
            gain_i: 5,
            .. Default::default()
        },
        trajectory: TrajectoryConf {
            a_speed: 30.0,
            a_acc: 100.0,
            xy_speed: 2000.0,
            xy_acc: 1000.0,
            xy_stop_window: 20.0,
            xy_aim_angle_window: 0.05,
            xy_cruise_angle_window: 1.5,
            xy_approach_window: 50.0,
            a_stop_window: 0.03,
            xy_idle_speed: 0.01,
            a_idle_speed: 0.01,
        },
        motors: MotorsConf::from_dimensions(75.0, 30.0, 256),
    });

    let mut asserv_period = Periodicity::new(ASSERV_PERIOD);
    let mut asserv_tm_period = Periodicity::new(Duration::from_millis(100));
    let mut buttons_period = Periodicity::new(Duration::from_millis(200));
    let mut battery_period = Periodicity::new(Duration::from_millis(2000));
    let mut vlx_period = Periodicity::new(Duration::from_millis(2000));

    // PWM controlled peripherals
    let mut pwm_controller = board.pwm_controller().unwrap();
    pwm_controller.init().unwrap();
    pwm_controller.set_battery_rgb(Color::new(0.0, 0.0, 0.02));
    pwm_controller.set_ground_rgb(Color::new(0.02, 0.02, 0.02));

    // Other peripherals
    let mut pami_leds = board.leds().unwrap();
    let mut buttons = board.buttons().unwrap();
    let mut battery_level = board.battery_level().unwrap();
    let mut starting_cord_read = board.starting_cord().unwrap();

    // Initialize state and UI state
    let mut state = MainState::new(ui_events);
    // Don't use the setter here, to not trigger a state change
    state.starting_cord = (starting_cord_read)();
    state.set_buttons(buttons.read_state());
    state.set_emergency_stop(asserv.hardware_mut().emergency_stop_active());

    loop {
        let now = Instant::now();
        let _ = pami_leds.esp.toggle();

        // Emergency stop: check whenever possible
        state.set_emergency_stop(asserv.hardware_mut().emergency_stop_active());

        // Starting cord: check whenever possible
        //TODO Add anti-rebound?
        if state.set_starting_cord((starting_cord_read)()) {
            if state.step == MainStep::Match {
                // Note: it's not possible to disable BLE advertising.
                // It would require to make RomePeripheral available for non-ESP target,
                // and update the board interface.
                passkey_enabled.store(false, Ordering::Relaxed);
                //TODO Match starts! Trigger events (UI, etc.)
            }
        }

        if state.update_ground_led {
            pwm_controller.set_ground_rgb(state.ground_led_color());
            state.update_ground_led = false;
        }

        // Process ROME input messages
        for data in rome_rx.try_iter() {
            let _ = pami_leds.com.toggle();
            match rome::Message::decode(&data) {
                Err(err) => log::error!("ROME RX error: {err:?}"),
                Ok(message) => {
                    if !asserv.on_rome_message(&message) {
                        log::warn!("ROME: ignored message: {}", message.message_id());
                    }
                },
            }
        }

        if asserv_period.update(&now) {
            asserv.update(asserv_period.period());
        }

        if asserv_tm_period.update(&now) {
            if let Err(err) = rome_tx.send(asserv.asserv_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
            if let Err(err) = rome_tx.send(asserv.asserv_diff_tm_status().encode()) {
                log::error!("ROME send error: {:?}", err);
            }
        }

        // VLX capture
        if vlx_period.update(&now) {
            match vlx.get_distance() {
                Ok(distance) => log::info!("VLX sensor distance: {distance:?}"),
                Err(err) => log::error!("Failed to get VLX sensor distance: {err:?}"),
            }
        }

        // Buttons: dpad, switches
        if buttons_period.update(&now) {
            state.set_buttons(buttons.read_state());
        }

        // Battery voltage
        if battery_period.update(&now) {
            let (mv, percent) = battery_level.read_vbatt();
            state.set_battery_level(mv, percent);
            pwm_controller.set_battery_rgb(state.battery_led_color());
            if let Err(err) = rome_tx.send(rome::Message::BatteryLevel { mv, percent }.encode()) {
                log::error!("BLE send error: {:?}", err);
            }
        }

        // Process UI events
        while let Ok(trigger) = ui_triggers.try_recv() {
            match trigger {
                UiTrigger::ChangeTeam(new) => state.set_team(new),
                UiTrigger::ChangeStartDelay(new) => state.set_start_delay(new),
                UiTrigger::ChangeRole(new) => state.set_role(new),
            }
        }

        if let Some(duration) = asserv_period.next().checked_duration_since(now) {
            std::thread::sleep(duration);
        }
    }
}

