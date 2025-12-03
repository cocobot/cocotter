mod movement;
mod shared_gpio;

use std::{sync::{Arc, Mutex}, thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardSabotter;
pub use board_sabotter::{ImuSpi, Motor, GpioExpander, pca9535::{GPIOBank, StandardExpanderInterface}};
//use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;
use movement::{Movement, MovementLowLevelHardware};
use sch16t::Sch16t;

use crate::shared_gpio::SharedGpio;

fn main() {
    let mut board = BoardSabotter::new();

    //configure io drivers
    let gpio_expander = SharedGpio::new(
        board.gpio_expander.take().unwrap(),
    );
    let motor_0_gpio_expander = SharedGpio::new(
        board.motor_gpio_expander[0].take().unwrap(),
    );
    let motor_1_gpio_expander = SharedGpio::new(
        board.motor_gpio_expander[1].take().unwrap(),
    );
    let motor_2_gpio_expander = SharedGpio::new(
        board.motor_gpio_expander[2].take().unwrap(),
    );

    //configure gyro
    let mut gyro = Sch16t::new(board.imu_spi.take().unwrap(), 0);
    gyro.init().unwrap();

     let mut mot_ena = board.mot_ena.take().unwrap();
    mot_ena.set_high().ok(); 
    thread::sleep(Duration::from_millis(500));
    mot_ena.set_low().ok();
    unsafe {
        ets_delay_us(10);
    }
    mot_ena.set_high().ok();

    //configure low level hardware for asserv
    let asserv_hardware = MovementLowLevelHardware::new(
        gyro,
        gpio_expander.get_pin(3),
        [
            (board.motors[0].take().unwrap(), motor_0_gpio_expander.get_pin(4)),
            (board.motors[1].take().unwrap(), motor_1_gpio_expander.get_pin(4)), 
            (board.motors[2].take().unwrap(), motor_2_gpio_expander.get_pin(4)),
        ],
    );
    let movement = Arc::new(Mutex::new(Movement::new(asserv_hardware)));
    
   

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();
    let mut motor_0_heartbeat = motor_0_gpio_expander.get_pin(2);
    let mut motor_1_heartbeat = motor_1_gpio_expander.get_pin(2);
    let mut motor_2_heartbeat = motor_2_gpio_expander.get_pin(2);

////
////    let mut motor_0 = board.motors[0].take().unwrap();
////    let mut motor_1 = board.motors[1].take().unwrap();
////    let mut motor_2 = board.motors[2].take().unwrap();
////    
////    //open brake
////    let _gpio_expander = board.gpio_expander.take().unwrap();
////
////    let mut motor_gpio_expander_0 = board.motor_gpio_expander[0].take().unwrap();
////    motor_gpio_expander_0.pin_into_output(GPIOBank::Bank0, 2).unwrap();
////    motor_gpio_expander_0.pin_into_output(GPIOBank::Bank0, 4).unwrap();
////    motor_gpio_expander_0.pin_set_low(GPIOBank::Bank0, 4).unwrap();
////
////    /*
////    gpio_expander.pin_into_output(GPIOBank::Bank0, 3).unwrap();
////
////    gpio_expander.pin_set_high(GPIOBank::Bank0, 3).unwrap();
////    gpio_expander.pin_set_low(GPIOBank::Bank0, 3).unwrap();
////    gpio_expander.pin_set_high(GPIOBank::Bank0, 3).unwrap();
////    */
////    let mut mot_ena = board.mot_ena.take().unwrap();
////    mot_ena.set_high().ok(); 
////    thread::sleep(Duration::from_millis(500));
////    mot_ena.set_low().ok();
////    unsafe {
////        ets_delay_us(10);
////    }
////    mot_ena.set_high().ok();
////
////    motor_0.dir.set_duty(0).ok();
////    motor_0.pwm.set_duty(500).ok();
////
////    motor_1.dir.set_duty(0).ok();
////    motor_1.pwm.set_duty(500).ok();
////
////    motor_2.dir.set_duty(0).ok();
////    motor_2.pwm.set_duty(500).ok();


    loop {       
        led_heartbeat.toggle().ok();
        motor_0_heartbeat.toggle();
        motor_1_heartbeat.toggle();
        motor_2_heartbeat.toggle();
        thread::sleep(Duration::from_millis(500));
       

        {
            
            let movement = movement.lock().unwrap();
            let asserv = movement.get_asserv();
            let asserv = asserv.lock().unwrap();
            let position = asserv.position().clone();
            drop(asserv);
            drop(movement);
                        
            log::info!("Position: x: {:.2} y: {:.2} theta: {:.2}", position.x, position.y, position.a); 
        }
    }
}
