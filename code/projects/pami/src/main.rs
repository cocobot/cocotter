#![no_std]
#![no_main]

mod ui;
mod pwm;
mod asserv;
mod config;

use asserv::Asserv;
use board_pami_2023::{Pami2023, PamiAdc, PamiAdcChannel};
use embassy_executor::Spawner;
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
        const VBATT_RL_KOHMS : f32= 91.0;
        const VBATT_RH_KOHMS : f32= 91.0;
        let vbatt_raw : u16 = adc.read(PamiAdcChannel::VBat).await;
        log::warn!("vbatt raw {} / {}", vbatt_raw, ((vbatt_raw as f32)/(((1 << 12) -1) as f32)) * 3100.0 );
        let vbatt_mv : f32 = ((vbatt_raw as f32)/(((1 << 12) -1) as f32)) * 3100.0 * (1.0 + VBATT_RH_KOHMS/VBATT_RL_KOHMS);

        ui.send_event(UIEvent::Vbatt { voltage_mv: vbatt_mv });

        Timer::after(Duration::from_millis(500)).await;
    }
}


#[main]
async fn main(spawner: Spawner) {
    let mut board = Pami2023::new();
    init_heap();

    esp_println::logger::init_logger_from_env();
    
    PWM::new(board.pwm_extended.take().unwrap(), spawner).await;

    let ui = UI::new(spawner);
    let _asserv = Asserv::new(
        spawner, 
        
        board.left_wheel_counter.take().unwrap(), 
        board.right_wheel_counter.take().unwrap()
    );

    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();
    spawner.spawn(analog_reading(ui.clone(), board.adc.take().unwrap())).unwrap();
    

    //run color blind test
    loop {        
        
        if let Some(btns) = board.buttons.as_mut() {
            let state = btns.get_input().await.ok();
            log::info!("Buttons state: {:?}", state);
        }
        else {
            log::info!("No buttons found");
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
