use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use asserv::differential::conf::*;
use asserv::differential::MovementDirection;
use asserv::maths::XYA;
use board_pami::PamiBoard;
use pami::config::PamiConfig;
use pami::events::*;
use pami::routines::{PamiRoutines, MatchStep};
use pami::ui::Ui;

#[cfg(target_os = "espidf")]
type PamiBoardImpl = board_pami::EspPamiBoard;
#[cfg(not(target_os = "espidf"))]
use board_pami::MockPamiBoard as PamiBoardImpl;

#[cfg(target_os = "espidf")]
fn flush_display(display: &mut board_pami::esp::PamiDisplay) {
    display.flush().unwrap();
}
#[cfg(not(target_os = "espidf"))]
fn flush_display(_display: &mut board_pami::mock::PamiDisplay) {}

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
    let rome_server = board.rome(config.name.into(), passkey_notifier).unwrap();

    let mut routines = PamiRoutines::new(&mut board, rome_server, (ui_events, ui_triggers));
    routines.asserv.set_conf(AsservConf {
        pid_dist: PidConf {
            gain_p: 10,
            gain_i: 1,
            max_i: 1000,
            .. Default::default()
        },
        pid_angle: PidConf {
            gain_p: 500,
            gain_i: 10,
            max_i: 1000,
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

    let match_conf = routines.match_setup();

    // Note: it's not possible to disable BLE advertising.
    // It would require to make RomePeripheral available for non-ESP target,
    // and update the board interface.
    passkey_enabled.store(false, Ordering::Relaxed);
    //TODO Match starts! Trigger events (UI, etc.)

    let flipped_x = match_conf.team == Team::Left;
    macro_rules! xflip {
        ($e:expr) => { if flipped_x { $e.xflip() } else { $e } }
    }

    let start_position = match match_conf.role {
        PamiRole::None => XYA::new(0.0, 0.0 ,0.0),
        PamiRole::Ninja => XYA::new(800.0,1950.0,0.0),
        PamiRole::Paninja(n) => XYA::new(1500.0-50.0*n as f32,1950.0, -core::f32::consts::FRAC_PI_2)
    };
    routines.asserv.reset_position(xflip!(start_position));

    routines.wait_match_start(&match_conf);

    routines.ui_events.send(UiEvent::ShowMessage("MATCH")).unwrap();

    // Wait for PAMI to be allowed to move
    loop {
        let now = routines.step_idle();
        match routines.match_step(now) {
            // Should never happen (previous step)
            MatchStep::WaitingMatchStart => {},
            // Still waiting...
            MatchStep::WaitingPamiStart(_) => {},
            // Finally, we can move!
            MatchStep::PamiActive(_) => break,
            // Should never happen at this stage
            MatchStep::MatchEnded => break,
        }
    }

    //TODO It's your time PAMI, move!

    if match_conf.role == PamiRole::None {
        routines.asserv.set_movement_direction(MovementDirection::Backward);
        routines.got_to_xy_with_detection(0.0, 500.0);
    }

    loop {
        let now = routines.step_idle();


        match routines.match_step(now) {
            // Should never happen (previous step)
            MatchStep::WaitingMatchStart => {},
            MatchStep::WaitingPamiStart(_) => {},
            MatchStep::PamiActive(_) => {},
            // Match ended, stop now!
            MatchStep::MatchEnded => break,
        }
    }

    // Match or strategy routine ended
    routines.asserv.hardware_mut().force_stop(true);
    //TODO Start the fan, update some LED, ...

    // Wait indefinitely
    loop {
        let _ = routines.step_idle();
    }
}

