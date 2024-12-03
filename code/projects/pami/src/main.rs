#![no_std]
#![no_main]

use board_pami_2023::{board_init, PWM_EXTENDED_LED_RGB};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::gpio::Output;
use esp_hal_embassy::main;
use embedded_hal_async::i2c::I2c;

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

    log::info!("Check for I2C devices");
    for addr in 0..127 {
        let mut buf = [0u8; 1];
        let r = board.generic_i2c.read(addr, &mut buf).await;
        if r.is_ok() {
            log::info!("Found device at address 0x{:02x}", addr);
        }
    }
    
    //configure pwm
    let mut pwm_extended = board.pwm_extended.take();
    if let Some(pwm_extended) = pwm_extended.as_mut() {
        pwm_extended.set_prescale(100).await.ok(); // 60Hz for servomotor
        pwm_extended.enable().await.ok();        

        pwm_extended.set_channel_on_off(PWM_EXTENDED_LED_RGB[0], 0, 0).await.ok();
        pwm_extended.set_channel_on_off(PWM_EXTENDED_LED_RGB[1], 0, 0).await.ok();
        pwm_extended.set_channel_on_off(PWM_EXTENDED_LED_RGB[2], 0, 0).await.ok();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[0], 0).await.ok();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[1], 0).await.ok();
        pwm_extended.set_channel_on(PWM_EXTENDED_LED_RGB[2], 0).await.ok();
    }
    
    //run color blind test
    loop {
        
        
        if let Some(btns) = board.buttons.as_mut() {
            let state = btns.get_input().await.ok();
            log::info!("Buttons state: {:?}", state);
        }
        else {
            log::info!("No buttons found");
        }

        if let Some(pwm_extended) = pwm_extended.as_mut() {
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[0], 2047).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[1], 4095).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[2], 4095).await.ok();
            Timer::after(Duration::from_millis(250)).await;


            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[0], 2047).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[1], 2047).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[2], 4095).await.ok();
            Timer::after(Duration::from_millis(250)).await;

            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[0], 4095).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[1], 2047).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[2], 4095).await.ok();
            Timer::after(Duration::from_millis(250)).await;

            //the last one is blue. This is the easy one
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[0], 4095).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[1], 4095).await.ok();
            pwm_extended.set_channel_off(PWM_EXTENDED_LED_RGB[2], 2047).await.ok();
            Timer::after(Duration::from_millis(250)).await;
        }
        else {
            log::info!("No PWM device found");
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}
