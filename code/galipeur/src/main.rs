mod movement;
mod shared_gpio;
mod can;
mod meca;
mod led;

use std::{sync::{Arc, Mutex}, thread, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardSabotter;
pub use board_sabotter::{ImuSpi, SmartLeds, Motor, GpioExpander, pca9535::{GPIOBank, StandardExpanderInterface}};
//use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;
use movement::{Movement, MovementLowLevelHardware};
use sch16t::Sch16t;
use asserv::holonomic::Asserv;
use asserv::maths::XY;
use smart_leds::RGB8;

use crate::{meca::Meca, shared_gpio::SharedGpio, led::Leds};

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

    //configure leds
    let leds = Leds::new(board.leds.take().unwrap());
    let led_sender = leds.sender();

    //configure gyro
    let mut gyro = Sch16t::new(board.imu_spi.take().unwrap(), 0);
    gyro.init().unwrap();

    let mut led_heartbeat = board.led_heartbeat.take().unwrap();
    let mut motor_0_heartbeat = motor_0_gpio_expander.get_pin(2);
    let mut motor_1_heartbeat = motor_1_gpio_expander.get_pin(2);
    let mut motor_2_heartbeat = motor_2_gpio_expander.get_pin(2);

    // Motor driver startup procedure
    // See DRV8243 §7.7.2.1 HW Variant
    //TODO heartbeat led follows expected nFAULT state
    {
        log::info!("Motor drivers init");

        motor_0_heartbeat.pin_set_high();
        motor_1_heartbeat.pin_set_high();
        motor_2_heartbeat.pin_set_high();

        let expanders = [&motor_0_gpio_expander, &motor_1_gpio_expander, &motor_2_gpio_expander];
        let mut mot_ena = board.mot_ena.take().unwrap();

        mot_ena.set_low().ok();
        thread::sleep(Duration::from_millis(10));

        mot_ena.set_high().ok();
        // Assert all expanders nFAULT low
        {
            let deadline = std::time::Instant::now() + Duration::from_secs(3);
            while !expanders.iter().all(|ex| ex.get_pin(3).pin_is_low().unwrap_or(false)) {
                if std::time::Instant::now() >= deadline {
                    log::warn!("Timeout waiting for nFAULT low on motor expanders");
                    break;
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
        thread::sleep(Duration::from_millis(10));

        mot_ena.set_low().ok();
        // A short wait (between 5µs and 10µs) is required; don't replace with a `thread::sleep()`
        unsafe { ets_delay_us(10); }
        mot_ena.set_high().ok();

        // Assert all expanders nFAULT high
        {
            let deadline = std::time::Instant::now() + Duration::from_secs(3);
            while !expanders.iter().all(|ex| ex.get_pin(3).pin_is_high().unwrap_or(false)) {
                if std::time::Instant::now() >= deadline {
                    log::warn!("Timeout waiting for nFAULT high on motor expanders");
                    break;
                }
                thread::sleep(Duration::from_millis(10));
            }
        }

        log::info!("Motor drivers initialized");
    }
    log::info!("Board initialized");

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

    let can_iface = can::CanInterface::new(board.can_bus.take().expect("CAN bus not initialized"));
    can::setup_can_log(&can_iface);
    let meca = meca::Meca::new(&can_iface);
    #[derive(Debug)]
    enum Order<'a> {
        GotoXy(f32, f32),
        GotoA(f32),
        GotoXyA(f32, f32, f32),
        RunPath(&'a [XY]),
        MecaTake,
        MecaDrop,
    }

    impl Order<'_> {
        fn apply(&self, asserv: &mut Asserv<MovementLowLevelHardware>, meca: &Meca) {
            match self {
                Order::GotoXy(x, y) => asserv.goto_xy(*x, *y),
                Order::GotoA(a) => asserv.goto_a(*a),
                Order::GotoXyA(x, y, a) => asserv.goto_xya(*x, *y, *a),
                Order::RunPath(path) => asserv.run_path(path),
                Order::MecaTake => {
                    meca.lower_arm(0, 3);
                    meca.lower_arm(0, 2);     
                    meca.lower_arm(0, 1);     
                    meca.lower_arm(0, 0);      
                    meca.release(0, 0);              
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(3));
                }
                Order::MecaDrop => {
                    meca.raise_arm(0, 3);
                    meca.raise_arm(0, 2);
                    meca.raise_arm(0, 1);
                    meca.raise_arm(0, 0);
                    meca.grab(0, 0);              
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
            }
        }
    }

    let orders = [
        Order::RunPath(&[
            XY::new(0.0, 500.0),
            XY::new(500.0, 500.0),
        ]),
        Order::GotoA(std::f32::consts::PI * 0.9),
        Order::MecaTake,
        Order::RunPath(&[
            XY::new(500.0, 0.0),
            XY::new(0.0, 0.0),
        ]),
        Order::GotoXyA(-200.0, 200.0, 0.0),
        Order::MecaDrop,
    ];
    let mut index = 0;

    {
        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        asserv.goto_xya(0., 0., 0.);
    }

    let mut color = false;
    loop {
        led_heartbeat.toggle().ok();
        motor_0_heartbeat.toggle();
        motor_1_heartbeat.toggle();
        motor_2_heartbeat.toggle();
        if color {
            led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 127, g: 127, b: 0 }}).ok();
            color = false;
        } else {
            led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 0, g: 0, b: 255 }}).ok();
            color = true;
        }
        thread::sleep(Duration::from_millis(500));

        {
            let meca_state = meca.get_state();
            log::info!("M0A0 : {} M0A1 : {} M0A2 : {} M0A3 : {}", 
                meca_state.arms[0][0].position, 
                meca_state.arms[0][1].position, 
                meca_state.arms[0][2].position, 
                meca_state.arms[0][3].position);
        }

        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        let position = asserv.cs.position();

        log::info!("Position: x: {:.2} y: {:.2} theta: {:.2}", position.x, position.y, position.a);

        if asserv.done_xy() && asserv.done_a() {
            let order = &orders[index];
            log::info!("Send new order: {:?}", order);
            order.apply(&mut asserv, &meca);
            index = (index + 1) % orders.len();
        }
    }
}
