use std::{thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;
#[cfg(target_os = "espidf")]
use ble::BleComm;
#[cfg(target_os = "linux")]
use board_simulator::{BoardSabotter, comm::BleComm};

mod asserv;

use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;

use crate::asserv::Asserv;


fn main() {
    let mut board = BoardSabotter::new();

    let asserv = Asserv::new();

    let (rome_tx, rome_rx) = BleComm::run(board.ble.take().unwrap(), "Galipeur".to_string());

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

    let mut motor_0 = board.motors[0].take().unwrap();
    let mut motor_1 = board.motors[1].take().unwrap();
    let mut motor_2 = board.motors[2].take().unwrap();
    
    //open brake
    let _gpio_expander = board.gpio_expander.take().unwrap();

    let mut motor_gpio_expander_0 = board.motor_gpio_expander[0].take().unwrap();
    motor_gpio_expander_0.pin_into_output(GPIOBank::Bank0, 2).unwrap();
    motor_gpio_expander_0.pin_into_output(GPIOBank::Bank0, 4).unwrap();
    motor_gpio_expander_0.pin_set_low(GPIOBank::Bank0, 4).unwrap();

    /*
    gpio_expander.pin_into_output(GPIOBank::Bank0, 3).unwrap();

    gpio_expander.pin_set_high(GPIOBank::Bank0, 3).unwrap();
    gpio_expander.pin_set_low(GPIOBank::Bank0, 3).unwrap();
    gpio_expander.pin_set_high(GPIOBank::Bank0, 3).unwrap();
    */
    let mut mot_ena = board.mot_ena.take().unwrap();
    mot_ena.set_high().ok(); 
    thread::sleep(Duration::from_millis(500));
    mot_ena.set_low().ok();
    unsafe {
        ets_delay_us(10);
    }
    mot_ena.set_high().ok();

    motor_0.dir.set_duty(0).ok();
    motor_0.pwm.set_duty(500).ok();

    motor_1.dir.set_duty(0).ok();
    motor_1.pwm.set_duty(500).ok();

    motor_2.dir.set_duty(0).ok();
    motor_2.pwm.set_duty(500).ok();


    loop {
        led_heartbeat.toggle().ok();
        motor_gpio_expander_0.pin_set_high(GPIOBank::Bank0, 2).unwrap();

        
        thread::sleep(Duration::from_millis(500));
        motor_gpio_expander_0.pin_set_low(GPIOBank::Bank0, 2).unwrap();
        thread::sleep(Duration::from_millis(500));

        //test ble
        loop {
            if let Ok(mut data) = rome_rx.recv_timeout(Duration::from_millis(100)) {
                println!("Recv {:?}", data);
                data[0] = 0xFF;
                rome_tx.send(data).unwrap();
            }
            else {
                break;
            }
        }
    }
}
