#![no_std]
#![no_main]

use board_pami_2023::{board_init, PWM_EXTENDED_LED_RGB};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::gpio::Output;
use esp_hal_embassy::main;

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
        log::info!("heatbeat low !");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        log::info!("heatbeat high !");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {        
    let mut board = board_init();
    init_heap();

    esp_println::logger::init_logger_from_env();

    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();

    //configure pwm
    let mut pwm_extended = board.pwm_extended.take().unwrap();
    pwm_extended.set_prescale(100).await.unwrap(); // 60Hz for servomotor
    pwm_extended.enable().await.unwrap();
    
    //run color blind test
    loop {
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[0], 4095).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[1], 0).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[2], 0).await.unwrap();
        Timer::after(Duration::from_millis(250)).await;


        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[0], 2048).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[1], 2048).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[2], 0).await.unwrap();
        Timer::after(Duration::from_millis(250)).await;

        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[0], 0).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[1], 4095).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[2], 0).await.unwrap();
        Timer::after(Duration::from_millis(250)).await;

        //the last one is blue. This is the easy one
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[0], 0).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[1], 0).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[2], 4095).await.unwrap();
        Timer::after(Duration::from_millis(250)).await;
    }
}
