#![feature(const_float_methods)]

mod events;
mod pwm;
mod ui;
mod config;
mod asserv;
mod sensors;
mod game;

use std::{thread, time::Duration};

use asserv::Asserv;
use board_pami_2023::{adc::{PamiAdc, PamiAdcChannel}, Pami2023};
use events::{Event, EventSystem};
use game::{Game, GameConfiguration};
use pwm::{PWMEvent, PWM};
use sensors::Sensors;
use ui::UI;

fn analog_reading(adc: &mut PamiAdc, event: &EventSystem) {

    let vbatt_mv : u16 = adc.read(PamiAdcChannel::VBat);//.await;    

    //log::info!("VBat: {} mV", vbatt_mv);
    //log::info!("I: {} {}", adc.read(PamiAdcChannel::IMotLeft), adc.read(PamiAdcChannel::IMotRight));

    event.send_event(Event::Vbatt { voltage_mv: vbatt_mv as f32 });
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut board = Pami2023::new();
    let event = EventSystem::new();

    PWM::new(board.pwm_extended.take().unwrap(), &event);

    let config: &'static config::PAMIConfig = config::PAMIConfig::get_config().unwrap();
    log::info!("PAMI id={} color={}", config.id, config.color);

    match config.color {
        "Rose" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([0.75, 0.0, 0.5]) });
        },
        "Blue" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([0.0, 0.0, 1.0]) });
        },
        "Red" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([1.0, 0.0, 0.0]) });
        },
        "Yellow" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([1.0, 0.8, 0.0]) });
        },
        _ => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([0.1, 0.1, 0.1]) });
        }
    }

    UI::new(board.display.take().unwrap(), config.id, config.color.to_string(), &event);
    let asserv = Asserv::new(
        board.emergency_stop.take().unwrap(),
        board.led_error.take().unwrap(),
        board.encoder_left.take().unwrap(),
        board.encoder_right.take().unwrap(),
        board.left_pwm.take().unwrap(),
        board.right_pwm.take().unwrap(),
        &event
    );
    Sensors::new(board.tof.take().unwrap(), board.line_sensor.take().unwrap(), &event);    
    
    let mut adc = board.adc.take().unwrap(); 
    let mut led_heartbeat = board.led_heartbeat.take().unwrap();


    let mut selector = board.buttons.take().unwrap();
    let conf_3_button = board.conf_3_button.take().unwrap();
    let selector_value = selector.get_input().unwrap();

    let config = GameConfiguration {
        starter: board.starter.take().unwrap(),

        x_negative_color: (selector_value & 0b1000_0000) != 0,
        test_mode: conf_3_button.is_high(),

        strategy: config.strategy,
        //strategy: game::GameStrategy::MidPit,
    };
    Game::new(config, asserv, &event);

    event.send_event(Event::Pwm { pwm_event: PWMEvent::Vaccum(0.0)});


    loop {
        led_heartbeat.toggle().ok();
        analog_reading(&mut adc, &event);

       thread::sleep(Duration::from_millis(500));
        //log::info!("VL53L5CX: {:?}", vlx.get_distance());
/*
        event.send_event(Event::Pwm { pwm_event: PWMEvent::Vaccum(1.0)});
        thread::sleep(Duration::from_millis(1000));
        event.send_event(Event::Pwm { pwm_event: PWMEvent::Vaccum(0.0)});
        */
    }
}