#![no_std]
#![no_main]

use board_pami_2023::{board_init, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_VBAT_RGB};
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
        log::info!("heatbeat low !\n");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        log::info!("heatbeat high !\n");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}
/* 
#[embassy_executor::task]
async fn adc_mon(mut adc: PamiAdc) {
    loop {
        log::info!("heatbeat low !\n");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        log::info!("heatbeat high !\n");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}
*/

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
    
    let mut adc = board.adc.take().unwrap();

    //run color blind test
    loop {
        const VBATT_RL_KOHMS : f32= 91.0;
        const VBATT_RH_KOHMS : f32= 91.0;
        let vbatt_raw : u16 = adc.adc.read_oneshot(&mut adc.vbatt).unwrap();
        let vbatt_mv : f32 = (vbatt_raw as f32)/((1<<12 -1) as f32) * 1750.0 * (1.0 + VBATT_RH_KOHMS/VBATT_RL_KOHMS);
        log::info!("BATT : battery voltage : {}mV", vbatt_mv);

        // lifepo4 discharge example:  https://cleversolarpower.com/lifepo4-voltage-chart/
        const VBATT_LVL_WARN_MV : f32 = 3150.0; // corresponds to around 15% SOC battery SOC
        const VBATT_LVL_ERR_MV  : f32 = 3000.0; // corresponds to around 5% battery SOC

        let vbat_color :[u16; 3];
        match vbatt_mv {
            mv if mv <= VBATT_LVL_ERR_MV  => vbat_color = [ 2048, 0,    0   ], // error : stop required
            mv if mv <= VBATT_LVL_WARN_MV => vbat_color = [ 2048, 2048, 0   ], // low battery
            _                             => vbat_color = [ 0,    0,    4095], // ok
        }

        pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[0], vbat_color[0]).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[1], vbat_color[1]).await.unwrap();
        pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[2], vbat_color[2]).await.unwrap();

        
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
