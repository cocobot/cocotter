mod movement;

use std::{sync::{Arc, Mutex}, thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardSabotter;
pub use board_sabotter::ImuSpi;
use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;
use movement::{Movement, MovementLowLevelHardware};
use sch16t::Sch16t;

fn main() {
    let mut board = BoardSabotter::new();

    //configure gyro
    let mut gyro = Sch16t::new(board.imu_spi.take().unwrap(), 0);
    gyro.init().unwrap();

    //configure low level hardware for asserv
    let asserv_hardware = MovementLowLevelHardware::new(gyro);
    let movement = Arc::new(Mutex::new(Movement::new(asserv_hardware)));
    

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

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
       //
       //motor_gpio_expander_0.pin_set_high(GPIOBank::Bank0, 2).unwrap();

       //
       //thread::sleep(Duration::from_millis(500));
       //motor_gpio_expander_0.pin_set_low(GPIOBank::Bank0, 2).unwrap();
        thread::sleep(Duration::from_millis(500));
        {
            led_heartbeat.toggle().ok();
            
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
