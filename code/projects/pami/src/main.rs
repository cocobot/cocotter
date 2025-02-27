#![no_std]
#![no_main]

mod ui;
mod pwm;
mod asserv;
mod config;

use asserv::Asserv;
use board_pami_2023::{Pami2023, PamiAdc, PamiAdcChannel};
use cocotter::trajectory::{order::Order, Trajectory, TrajectoryOrderList};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::gpio::Output;
use esp_hal_embassy::main;
use pwm::{PWMEvent, PWM};
use ui::{UIEvent, UI};

extern crate alloc;
use core::mem::MaybeUninit;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn analog_reading(ui: UI, mut adc: PamiAdc) {
    loop {
        let vbatt_mv : u16 = adc.read(PamiAdcChannel::VBat).await;

        //let vbatt_mv : f32 = (f32::from(vbatt_raw))/(((1<<12) -1) as f32) * 3100.0 * (1.0 + VBATT_RH_KOHMS/VBATT_RL_KOHMS);
       
        ui.send_event(UIEvent::Vbatt { voltage_mv: vbatt_mv as f32});

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn game_logic(trajectory: Trajectory<CriticalSectionRawMutex, 2>) {

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
        */
        let order = TrajectoryOrderList::new()
            .add_order(Order::GotoXY { x_mm: 100.0, y_mm: 0.0 })
            .add_order(Order::GotoXY { x_mm: 100.0, y_mm: 100.0 })
            .add_order(Order::GotoXY { x_mm: 0.0, y_mm: 100.0 })
            .add_order(Order::GotoXY { x_mm: 0.0, y_mm: 0.0 })
            .add_order(Order::GotoA { a_rad: 0.0 })
            ;

        match trajectory.execute(order).await {
            Ok(_) => log::info!("Trajectory done"),
            Err(e) => log::error!("Trajectory error: {:?}", e),
        }
        Timer::after(Duration::from_secs(3)).await;
    }
}

#[embassy_executor::task]
async fn test_vaccum() {
loop{
    PWM::send_event(PWMEvent::Vaccum(0.0));
    Timer::after(Duration::from_secs(3)).await;
 //   PWM::send_event(PWMEvent::Vaccum(1.0));
    Timer::after(Duration::from_secs(3)).await;
}

}


#[main]
async fn main(spawner: Spawner) {
    let mut board = Pami2023::new();
    init_heap();

    esp_println::logger::init_logger_from_env();
    
    PWM::new(board.pwm_extended.take().unwrap(), spawner).await;

    let ui = UI::new(spawner);
    let asserv = Asserv::new(
        spawner, 
        
        board.emergency_stop.take().unwrap(), 

        board.left_wheel_counter.take().unwrap(), 
        board.right_wheel_counter.take().unwrap(),

        board.left_motor_pwm.take().unwrap(),
        board.right_motor_pwm.take().unwrap(),
    );
    let trajectory = Trajectory::new(asserv.lock().await.get_position());
    
    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();
    spawner.spawn(analog_reading(ui.clone(), board.adc.take().unwrap())).unwrap();
    spawner.spawn(game_logic(trajectory)).unwrap();

    let mut accel = board.accelerometer.take().unwrap();
    
    spawner.spawn(test_vaccum()).unwrap();

    PWM::send_event(PWMEvent::LineLedLvl(1.0));
    accel.init();

    let print_accell = false;
    let print_line =false;

    //run color blind test
    loop {        

        if print_accell {
            accel.update_measures();
            //let angle = accel.get_angular_rate();
            let angle = accel.get_angular_rate();
            let acc = accel.get_acceleration();
            let temp = accel.get_temperature_degc();
            log::info!("accelerometer accel {:4} {:4} {:4} \t{:4} {:4} {:4} \t{:2.3}", angle.x, angle.y, angle.z, acc.x, acc.y, acc.z, temp);
            if let Some(btns) = board.buttons.as_mut() {
                let state = btns.get_input().await.ok();
                match state{ 
                Some(i) => {
                        let mut st : [u8; 8] = [0;8];

                        for n in 0..7{
                            if i & ((1<<n) as u8)==0{
                                st[n] = 0;
                            }else {
                                st[n] = 1;
                            }
                            
                        }
                        log::info!("Buttons state: {:?}", state);
                    },
                    _=> log::info!("autre"),
                }
            }
            else {
                log::info!("No buttons found");
            }
        }

        if print_line {
            if let Some(lines) = board.line_sensor.as_mut() {
                let state = lines.get_input().await.ok();
                match state{ 
                
                    Some(i) => {
                        let mut st : [u8; 8] = [0;8];
        
                        for n in 0..=7{
                            if i & ((1<<n) as u8)==0{
                                st[n] = 1;
                            }else {
                                st[n] = 0;
                            }
                            
                        }
                        log::info!("line state: {:?}, r3 {}, r2 {}, r1 {}, r0 {}, l0 {}, l1 {}, l2 {}, l3 {}", state, st[0], st[1], st[2], st[3], st[4], st[5], st[6], st[7]);
                    },
                    _=> log::info!("autre"),
                }
                //log::info!("lines state: {:?}", state);
            }
            else {
                log::info!("No line found");
            }
        }

        PWM::send_event(PWMEvent::LedBottom([0.5, 0.0, 0.0]));
        Timer::after(Duration::from_millis(250)).await;

        PWM::send_event(PWMEvent::LedBottom([0.5, 0.5, 0.0]));
        Timer::after(Duration::from_millis(250)).await;

        PWM::send_event(PWMEvent::LedBottom([0.5, 0.5, 0.0]));
        Timer::after(Duration::from_millis(250)).await;

        //the last one is blue. This is the easy one
        PWM::send_event(PWMEvent::LedBottom([0.0, 0.0, 0.5]));
        Timer::after(Duration::from_millis(250)).await;
        
    }
}
