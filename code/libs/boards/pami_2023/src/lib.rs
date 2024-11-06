#![no_std]

use esp_hal::{gpio::{Io, Level, Output}, timer::timg::TimerGroup};


pub struct Pami2023 {
    pub led_esp: Option<Output<'static>>,
    pub led_com: Option<Output<'static>>,
}

pub fn board_init() -> Pami2023 {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    //init embassy
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    //init peripherals
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    Pami2023 { 
        led_esp: Some(Output::new(io.pins.gpio4, Level::High)),
        led_com: Some(Output::new(io.pins.gpio5, Level::High)),
    }
}