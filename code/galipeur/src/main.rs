use std::{sync::{Arc, Mutex}, thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardSabotter;

use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;
use asserv::{conf::{AsservHardware}, Asserv};

struct GalipeurAsservHardware {
    
}

impl GalipeurAsservHardware {
    pub fn new() -> Self {
        GalipeurAsservHardware {
            
        }
    }   
}

impl AsservHardware for GalipeurAsservHardware {
    fn set_motors_break(&mut self, enable: bool) {
        log::info!("Set motors break: {}", enable);
    }

    fn set_motor_consigns(&mut self, values: [f32; 3]) {
      //  log::info!("Set motor consigns: {:?}", values);
    }

    fn get_motor_offsets(&mut self) -> [f32; 3] {
     //   log::info!("Get motor offsets");
        [0.0, 0.0, 0.0]
    }

    fn get_gyro_offset(&mut self) -> f32 {
    //    log::info!("Get gyro offset");
        0.0
    }
}

fn main() {
    let mut board = BoardSabotter::new();

    //configure asserv
    let asserv_hardware = GalipeurAsservHardware::new();
    let asserv = Arc::new(Mutex::new(Asserv::new(asserv_hardware)));
    let asserv_thread = asserv.clone();
    thread::spawn(move || {
        loop {
            {
                let mut asserv = asserv_thread.lock().unwrap();
                asserv.update();
            }
            thread::sleep(Duration::from_millis(10));
        }
    });
    

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
       //led_heartbeat.toggle().ok();
       //motor_gpio_expander_0.pin_set_high(GPIOBank::Bank0, 2).unwrap();

       //
       //thread::sleep(Duration::from_millis(500));
       //motor_gpio_expander_0.pin_set_low(GPIOBank::Bank0, 2).unwrap();
        thread::sleep(Duration::from_millis(500));
        {
            let asserv = asserv.lock().unwrap();
            let position = asserv.position().clone();
            drop(asserv);
                        
            log::info!("Position: x: {:.2} y: {:.2} theta: {:.2}", position.x, position.y, position.a); 
        }
    }
}
