#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod ui;
mod pwm;
mod asserv;
mod config;
mod ble;
mod events;

extern crate alloc;

use asserv::Asserv;
use ble::CommBle;
use board_pami_2023::{Pami2023, PamiAdc, PamiAdcChannel};
use cocotter::trajectory::{order::Order, RampCfg, Trajectory, TrajectoryOrderList};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal_embassy::main;
use events::{Event, EventSystem};
use log::LevelFilter;
use pwm::PWM;
use ui::UI;


async fn analog_reading(adc: &mut PamiAdc, event: &EventSystem) {

    let vbatt_mv : u16 = adc.read(PamiAdcChannel::VBat).await;    
    event.send_event(Event::Vbatt { voltage_mv: vbatt_mv as f32 });
}

#[embassy_executor::task]
async fn game_logic(trajectory: Trajectory<CriticalSectionRawMutex, 2>) {
    //loop {
    //    let order = TrajectoryOrderList::new()
    //        .add_order(Order::GotoA { a_rad: 0_f32.to_radians() });
//
    //    trajectory.execute(order).await.ok();
//
    //    Timer::after(Duration::from_secs(5)).await;
//
//
    //    let order = TrajectoryOrderList::new()
    //        .add_order(Order::GotoA { a_rad: 45_f32.to_radians() });
//
    //    trajectory.execute(order).await.ok();
//
    //    Timer::after(Duration::from_secs(5)).await;
    //}
    //loop {
    //    let order = TrajectoryOrderList::new()
    //        .add_order(Order::GotoD { d_mm: 50.0 });
//
    //    trajectory.execute(order).await.ok();
//
    //    Timer::after(Duration::from_secs(5)).await;
//
//
    //    let order = TrajectoryOrderList::new()
    //    .add_order(Order::GotoD { d_mm: -50.0 });
//
    //    trajectory.execute(order).await.ok();
//
    //    Timer::after(Duration::from_secs(5)).await;
    //}
    loop {
        /* 
        let order = TrajectoryOrderList::new()
            .add_order(Order::GotoA { a_rad: 0_f32.to_radians() })
            .add_order(Order::GotoD { d_mm: 100.0 })
            .add_order(Order::GotoA { a_rad: 90_f32.to_radians() })
            .add_order(Order::GotoD { d_mm: 100.0 })
            .add_order(Order::GotoA { a_rad: 180_f32.to_radians() })
            .add_order(Order::GotoD { d_mm: 100.0 })
            .add_order(Order::GotoA { a_rad: -90_f32.to_radians() })
            .add_order(Order::GotoD { d_mm: 100.0 })
            ;
        

        let order = TrajectoryOrderList::new()
            .set_backwards(true) //all orders will be executed backwards

            .add_order(Order::GotoXY { x_mm: 100.0, y_mm: 0.0 })
            .add_order(Order::GotoXY { x_mm: 100.0, y_mm: 100.0 })
                .set_max_speed(RampCfg::Linear, 0.2) //this order will be executed at 20% of the max speed
            .add_order(Order::GotoXY { x_mm: 0.0, y_mm: 100.0 })
            .add_order(Order::GotoXY { x_mm: 0.0, y_mm: 0.0 })
            .add_order(Order::GotoA { a_rad: 0.0 })
            ;*/

       /*  match trajectory.execute(order).await {
            Ok(_) => log::info!("Trajectory done"),
            Err(e) => log::error!("Trajectory error: {:?}", e),
        }*/
        Timer::after(Duration::from_secs(3)).await;
    }
}


#[main]
async fn main(spawner: Spawner) {
    esp_alloc::heap_allocator!(72 * 1024);
    //esp_println::logger::init_logger_from_env();
    esp_println::logger::init_logger(LevelFilter::Debug);

    let mut board = Pami2023::new();

    let event = EventSystem::new(spawner);

    //init all subsystems
    UI::new(spawner, &event).await;
    PWM::new(board.pwm_extended.take().unwrap(), spawner, &event).await;
    let asserv = Asserv::new(      
        board.emergency_stop.take().unwrap(),

        board.left_wheel_counter.take().unwrap(), 
        board.right_wheel_counter.take().unwrap(),

        board.left_motor_pwm.take().unwrap(),
        board.right_motor_pwm.take().unwrap(),

        spawner, 
        &event,
    ).await;
    CommBle::new(board.ble_connector.unwrap(), spawner, &event).await;

    let trajectory = Trajectory::new(asserv.lock().await.get_position());
    spawner.spawn(game_logic(trajectory)).unwrap();

    //hardware access for main loop
    let mut adc = board.adc.take().unwrap();
    let mut heartbeat_led = board.led_esp.take().unwrap();

    let config = config::PAMIConfig::get_config().unwrap();
    log::info!("PAMI id={} color={}", config.id, config.color);

    match config.color {
        "Violet" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([0.5, 0.0, 0.5]) });
        },
        "Yellow" => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([1.0, 0.8, 0.0]) });
        },
        _ => {
            event.send_event(Event::Pwm { pwm_event: pwm::PWMEvent::LedBottom([0.1, 0.1, 0.1]) });
        }
    }

    //main loop
    loop {
        heartbeat_led.toggle();
        analog_reading(&mut adc, &event).await;

        Timer::after(Duration::from_millis(500)).await;
    }
}