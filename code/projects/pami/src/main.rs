#![no_std]
#![no_main]

use board_pami_2023::{Pami2023, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_VBAT_RGB};
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

#[embassy_executor::task]
async fn asserv(left_wheel_counter: board_pami_2023::LeftWheelEncoder, right_wheel_counter: board_pami_2023::RightWheelEncoder) {
    loop {
        log::info!("left encoder value: {} / right encoder value: {}", left_wheel_counter.get(), right_wheel_counter.get());
        Timer::after(Duration::from_millis(100)).await;
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
    /*let peripherals = esp_hal::init(esp_hal::Config::default());

    //init embassy
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    init_heap();

    esp_println::logger::init_logger_from_env();

    let pin39 = Input::new(peripherals.GPIO39, Pull::None);
    let pin40 = Input::new(peripherals.GPIO40, Pull::None);

    loop {
        log::info!("pin39: {} / pin40 {}", pin39.is_high(), pin40.is_high());
        Timer::after(Duration::from_millis(25)).await;
    }*/

    let mut board = Pami2023::new();
    init_heap();

    esp_println::logger::init_logger_from_env();

    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();
    spawner.spawn(asserv(board.left_wheel_counter.take().unwrap(), board.right_wheel_counter.take().unwrap())).unwrap();

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
    
    //let mut adc = board.adc.take().unwrap();

    //run color blind test
    loop {
        //const VBATT_RL_KOHMS : f32= 91.0;
        //const VBATT_RH_KOHMS : f32= 91.0;
        //let vbatt_raw : u16 = adc.adc.read_oneshot(&mut adc.vbatt).unwrap();
        //let vbatt_mv : f32 = (vbatt_raw as f32)/((1<<12 -1) as f32) * 1750.0 * (1.0 + VBATT_RH_KOHMS/VBATT_RL_KOHMS);
        let vbatt_mv = 5.0;
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

        
        if let Some(btns) = board.buttons.as_mut() {
            let state = btns.get_input().await.ok();
            log::info!("Buttons state: {:?}", state);
        }
        else {
            log::info!("No buttons found");
        }

        if let Some(pwm_extended) = pwm_extended.as_mut() {

            pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[0], vbat_color[0]).await.unwrap();
            pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[1], vbat_color[1]).await.unwrap();
            pwm_extended.set_channel_on(PWM_EXTENDED_VBAT_RGB[2], vbat_color[2]).await.unwrap();

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
