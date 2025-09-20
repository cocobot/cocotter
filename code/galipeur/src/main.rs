use std::{thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;
#[cfg(target_os = "espidf")]
use comm::Comm;
#[cfg(target_os = "linux")]
use board_simulator::{BoardSabotter, comm::Comm};

pub mod sch16t;
use sch16t::SCH16T;
use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;


fn main() {
    let mut board = BoardSabotter::new();
    let (rome_tx, rome_rx) = Comm::run(board.ble.take().unwrap(), "Galipeur".to_string());

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();

    let mut motor_0 = board.motors[0].take().unwrap();
    let mut motor_1 = board.motors[1].take().unwrap();
    let mut motor_2 = board.motors[2].take().unwrap();
    

    // Initialize SCH16T IMU sensor
    let mut imu = if let Some(spi) = board.imu_spi.take() {
        let mut sensor = SCH16T::new(spi);

        // Test basic communication first
        match sensor.test_communication() {
            Ok(true) => {
                println!("SCH16T communication OK");

                // Initialize the sensor
                if let Err(e) = sensor.init() {
                    println!("Failed to initialize SCH16T: {:?}", e);
                } else {
                    println!("SCH16T initialized");
                }

                // Dump all registers for debugging
                sensor.dump_registers().ok();

                // Run self-test
                match sensor.self_test() {
                    Ok(true) => println!("SCH16T self-test passed"),
                    Ok(false) => println!("SCH16T self-test failed"),
                    Err(e) => println!("SCH16T self-test error: {:?}", e),
                }

                Some(sensor)
            }
            Ok(false) => {
                println!("SCH16T communication FAILED - check connections");
                None
            }
            Err(e) => {
                println!("SCH16T communication error: {:?}", e);
                None
            }
        }
    } else {
        println!("No SPI interface for IMU");
        None
    };

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
        println!("enc {} {} {}",
            motor_0.encoder.get_value().unwrap(),
            motor_1.encoder.get_value().unwrap(),
            motor_2.encoder.get_value().unwrap());
        thread::sleep(Duration::from_millis(250));
    }

    loop {
        led_heartbeat.toggle().ok();
        motor_gpio_expander_0.pin_set_high(GPIOBank::Bank0, 2).unwrap();

        // Read IMU data if available
        if let Some(ref mut sensor) = imu {
            // Check if new data is ready
            if let Ok(true) = sensor.data_ready() {
                // Read gyroscope data (degrees per second)
                if let Ok((gx, gy, gz)) = sensor.read_gyro() {
                    println!("Gyro: X={:.2}°/s, Y={:.2}°/s, Z={:.2}°/s", gx, gy, gz);
                }

                // Read accelerometer data (g)
                if let Ok((ax, ay, az)) = sensor.read_accel() {
                    println!("Accel: X={:.3}g, Y={:.3}g, Z={:.3}g", ax, ay, az);
                }

                // Read temperature
                if let Ok(temp) = sensor.read_temperature() {
                    println!("Temperature: {:.1}°C", temp);
                }
            }
        }

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
