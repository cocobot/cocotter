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
    log::info!("VBatt raw: {}", vbatt_mv);
    
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

//#[main]
//async fn main(spawner: Spawner) {
//    esp_alloc::heap_allocator!(72 * 1024);
//    //esp_println::logger::init_logger_from_env();
//    esp_println::logger::init_logger(LevelFilter::Debug);
//    let mut board = Pami2023::new();
//
//    let event = EventSystem::new(spawner);
//
//    PWM::new(board.pwm_extended.take().unwrap(), spawner).await;
// 
//    let ui = UI::new(spawner);
//   /*  let asserv = Asserv::new(
//        spawner, 
//        
//        board.left_wheel_counter.take().unwrap(), 
//        board.right_wheel_counter.take().unwrap(),
//
//        board.left_motor_pwm.take().unwrap(),
//        board.right_motor_pwm.take().unwrap(),
//    );
//    let trajectory = Trajectory::new(asserv.lock().await.get_position());*/
//    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();
//
//    /*
//    let mut vlx = board.front_tof.take().unwrap();
//    vlx.init().unwrap();
//    loop {
//        match vlx.get_distance() {
//            Ok(d) => log::info!("Distance: {:?}", d),
//            Err(e) => log::error!("VLX error: {:?}", e),
//        }
//        Timer::after(Duration::from_secs(1)).await;
//    }
//    */
// 
//    spawner.spawn(analog_reading(ui.clone(), board.adc.take().unwrap())).unwrap();
//   // spawner.spawn(game_logic(trajectory)).unwrap();
//
//    let mut accel = board.accelerometer.take().unwrap();
//    
//    spawner.spawn(test_vaccum()).unwrap();
//    //let mut vaccumm_lvl : u16;
//
//    //let mut left_motor_pwm = board.left_motor_pwm.take().unwrap();
//
//    PWM::send_event(PWMEvent::LineLedLvl(1.0));
//    accel.init();
//
//
//    let ble = CommBle::new(
//        spawner, 
//        board.ble_connector.unwrap(),
//        event,
//    );
//
//    //run color blind test
//    loop {        
//      
//        accel.update_measures();
//        //let angle = accel.get_angular_rate();
//        let angle = accel.get_angular_rate();
//        let acc = accel.get_acceleration();
//        let temp = accel.get_temperature_degc();
//        log::info!("accelerometer accel {:4} {:4} {:4} \t{:4} {:4} {:4} \t{:2.3}", angle.x, angle.y, angle.z, acc.x, acc.y, acc.z, temp);
//        if let Some(btns) = board.buttons.as_mut() {
//            let state = btns.get_input().await.ok();
//            match state{ 
//            
//            Some(i) => {
//                let mut st : [u8; 8] = [0;8];
//
//                for n in 0..7{
//                    if i & ((1<<n) as u8)==0{
//                        st[n] = 0;
//                    }else {
//                        st[n] = 1;
//                    }
//                    
//                }
//                log::info!("Buttons state: {:?}", state);
//            },
//            _=> log::info!("autre"),
//        }
//           /* match state{
//                Some(i:u8) => 
//
//            };*/
//            
//        
//        }
//        else {
//            log::info!("No buttons found");
//        }
//
//        if let Some(lines) = board.line_sensor.as_mut() {
//            let state = lines.get_input().await.ok();
//            match state{ 
//            
//                Some(i) => {
//                    let mut st : [u8; 8] = [0;8];
//    
//                    for n in 0..=7{
//                        if i & ((1<<n) as u8)==0{
//                            st[n] = 1;
//                        }else {
//                            st[n] = 0;
//                        }
//                        
//                    }
//                    log::info!("line state: {:?}, r3 {}, r2 {}, r1 {}, r0 {}, l0 {}, l1 {}, l2 {}, l3 {}", state, st[0], st[1], st[2], st[3], st[4], st[5], st[6], st[7]);
//                },
//                _=> log::info!("autre"),
//            }
//            //log::info!("lines state: {:?}", state);
//        }
//        else {
//            log::info!("No buttons found");
//        }
//
//
//        PWM::send_event(PWMEvent::LedBottom([0.5, 0.0, 0.0]));
//        Timer::after(Duration::from_millis(250)).await;
//
//        PWM::send_event(PWMEvent::LedBottom([0.5, 0.5, 0.0]));
//        Timer::after(Duration::from_millis(250)).await;
//
//        PWM::send_event(PWMEvent::LedBottom([0.5, 0.5, 0.0]));
//        Timer::after(Duration::from_millis(250)).await;
//
//        //the last one is blue. This is the easy one
//        PWM::send_event(PWMEvent::LedBottom([0.0, 0.0, 0.5]));
//        Timer::after(Duration::from_millis(250)).await;
//    }
//}
