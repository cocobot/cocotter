#![no_std]
#![no_main]

use board_pami_2023::board_init;
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
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {    
    let mut board = board_init();
    init_heap();
    
    esp_println::logger::init_logger_from_env();
    spawner.spawn(heartbeat(board.led_esp.take().unwrap())).unwrap();

    loop {
        log::info!("Hello world!");
        Timer::after(Duration::from_millis(500)).await;
    }
}
