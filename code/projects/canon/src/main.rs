#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use canon_2019::board_init;
use embassy_executor::Spawner;
use embassy_time::{Timer, Duration};
use motor_controller::{MotorDriveIO, MotorState};
use {panic_halt as _};

mod hall_sensors;
mod motor_controller;
mod inputs;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board: canon_2019::Canon2019 = board_init();

    //init motor pins
    let u_drive = MotorDriveIO {drive_pin: board.u_drive.take().unwrap(), enable_pin: board.u_en.take().unwrap()};
    let v_drive = MotorDriveIO {drive_pin: board.v_drive.take().unwrap(), enable_pin: board.v_en.take().unwrap()};
    let w_drive = MotorDriveIO {drive_pin: board.w_drive.take().unwrap(), enable_pin: board.w_en.take().unwrap()};
    
    //init sensor pins
    let uhall_irq = board.uhall_irq.take().unwrap();
    let vhall_irq = board.vhall_irq.take().unwrap();
    let whall_irq = board.whall_irq.take().unwrap();

    //init input pin
    let step_irq = board.step_irq.take().unwrap();
    let dir = board.dir.take().unwrap();

    //init debug uart
    let usart = board.uart.take().unwrap();

    //feed io to motor controller
    let mut motor = MotorState::get_mutex().lock().await;
    motor.set_debug_uart(usart);
    motor.set_io(u_drive, v_drive, w_drive);
    drop(motor);

    //start coroutines
    spawner.spawn(hall_sensors::hall_sensor_decoder_task(uhall_irq, vhall_irq, whall_irq)).unwrap();
    spawner.spawn(inputs::input_decoder_task(step_irq, dir)).unwrap();
    
    //blink led
    let mut led = board.led.take().unwrap();
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(250)).await;
    }
}