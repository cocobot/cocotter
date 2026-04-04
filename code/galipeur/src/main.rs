mod movement;
mod shared_gpio;
mod can;
mod can_ota_relay;
mod meca;
mod led;

use std::{sync::{Arc, Mutex}, thread::{self, Thread}, time::Duration};

#[cfg(target_os = "espidf")]
use board_sabotter::BoardSabotter;

#[cfg(not(target_os = "espidf"))]
use board_simulator::BoardSabotter;
pub use board_sabotter::{ImuSpi, SmartLeds, Motor, GpioExpander, pca9535::{GPIOBank, StandardExpanderInterface}};
use ble::{BleBuilder, EspSelfOtaHandler};
//use board_sabotter::pca9535::{expander::standard::StandardExpanderInterface, GPIOBank};
use esp_idf_svc::sys::ets_delay_us;
use movement::{Movement, MovementLowLevelHardware};
use sch16t::Sch16t;
use asserv::holonomic::Asserv;
use asserv::holonomic::RobotSide;
use asserv::holonomic::TableSide;
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


    let color_selector = gpio_expander.get_pin(6);
    let starter = gpio_expander.get_pin(7);

    // Motor driver startup procedure
    // See DRV8243 §7.7.2.1 HW Variant
    //TODO heartbeat led follows expected nFAULT state
    {
        log::info!("Motor drivers init");

        motor_0_heartbeat.pin_set_high().ok();
        motor_1_heartbeat.pin_set_high().ok();
        motor_2_heartbeat.pin_set_high().ok();

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

    // Init CAN before BLE (OTA relay needs CAN)
    let can_iface = can::CanInterface::new(board.can_bus.take().expect("CAN bus not initialized"));
    can::setup_can_log(&can_iface);

    let (ble_server, _ble_client) = BleBuilder::new().run();

    // Register GATT services (must happen before host start)
    let rome_reg = ble::rome::register_gatt();
    let ota_reg = ble::ota::register_gatt(2); // target 0 = self, target 1 = picotter

    // Start NimBLE host
    ble_server.start_host();

    // Finalize services
    let rome = rome_reg.start();
    let picotter_ota = can_ota_relay::CanOtaRelayHandler::new(&can_iface);
    let _ota = ota_reg.start(vec![
        Box::new(EspSelfOtaHandler::new()),
        Box::new(picotter_ota),
    ]);
    let rome_tx = rome.sender;

    // Setup and start advertising
    ble_server.setup_advertising("Galipeur", &ble::rome::SERVICE_UUID_BYTES).unwrap();
    ble_server.start_advertising().unwrap();

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
    let meca = meca::Meca::new(&can_iface);
    #[derive(Debug)]
    enum Order<'a> {
        GotoXy(f32, f32),
        GotoA(f32),
        GotoXyA(f32, f32, f32),
        RunPath(&'a [XY]),
        MecaTake,
        MecaRaiseGrab,
        MecaRaiseDrop,
        MecaIdlePosGrab,
        MecaIdlePosDrop,
    }

    let mut robot_color = false;
    let color_from_bool = |c| if c {RGB8 { r: 127, g: 127, b: 0 }} else {RGB8 { r: 0, g: 0, b: 255 }};    
    meca.pre_init();
    //meca.calibrate_color_sensors(200, 0xC0, 1);
    log::info!("Wait for starting cord to select color");
    while starter.pin_is_high().unwrap_or(true) {
        if color_selector.pin_is_high().unwrap_or(false) {
            robot_color = true;
        } else {
            robot_color = false;
        }
        led_sender.send(led::LedMessage::GameColor { color: color_from_bool(robot_color) }).ok();
        thread::sleep(Duration::from_millis(100));

        //print all servo positions for debug
        let state = meca.get_state();
        log::info!("T0: {} M0A0 : {} M0A1 : {} M0A2 : {} M0A3 : {} M0S20 : {} M0S21 : {}", 
            state.translations[0].position,
            state.arms[0][0].position, 
            state.arms[0][1].position, 
            state.arms[0][2].position, 
            state.arms[0][3].position,
            state.stage2[0][0].position,
            state.stage2[0][1].position,
        );    
    }
    if robot_color {
        log::info!("Starting cord plugged, we are YELLOW");
    } else {
        log::info!("Starting cord plugged, we are BLUE");
    }

    meca.init();

    let mut color_off = false;
    while starter.pin_is_low().unwrap_or(true) {
        if color_off {
            led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 0, g: 0, b: 0 }}).ok();
            color_off = false;
        } else {
            led_sender.send(led::LedMessage::GameColor { color: color_from_bool(robot_color) }).ok();
            color_off = true;
        }
        //wait for starter button press
        thread::sleep(Duration::from_millis(100));
    }
    led_sender.send(led::LedMessage::GameColor { color: color_from_bool(robot_color) }).ok();
    log::info!("Starting cord unplugged, here we go!");

    let robot_side_main = if robot_color { RobotSide::Right } else { RobotSide::Left };
    let robot_side_aux  = if robot_color { RobotSide::Left } else { RobotSide::Right };
    let table_side_main = if robot_color { TableSide::Left } else { TableSide::Right };
    let table_side_aux  = if robot_color { TableSide::Right } else { TableSide::Left };

    //function to get angle to apply to Align Robot Face Along given Side of the Table
    const fn arfast(face: RobotSide, side: TableSide) -> f32 {
        match (face, side) {
            (RobotSide::Left,  TableSide::Left)  => std::f32::consts::PI *  1.0/6.0,
            (RobotSide::Left,  TableSide::Right) => std::f32::consts::PI * -5.0/6.0,
            (RobotSide::Left,  TableSide::Up)    => std::f32::consts::PI * -1.0/3.0,
            (RobotSide::Left,  TableSide::Down)  => std::f32::consts::PI *  2.0/3.0,
            (RobotSide::Right, TableSide::Left)  => std::f32::consts::PI *  5.0/6.0,
            (RobotSide::Right, TableSide::Right) => std::f32::consts::PI * -1.0/6.0,
            (RobotSide::Right, TableSide::Up)    => std::f32::consts::PI *  1.0/3.0,
            (RobotSide::Right, TableSide::Down)  => std::f32::consts::PI * -2.0/3.0,
            (RobotSide::Back,  TableSide::Left)  => std::f32::consts::PI * -1.0/2.0,
            (RobotSide::Back,  TableSide::Right) => std::f32::consts::PI *  1.0/2.0,
            (RobotSide::Back,  TableSide::Up)    => std::f32::consts::PI *  1.0,
            (RobotSide::Back,  TableSide::Down)  => std::f32::consts::PI *  0.0,
        }
    }

    macro_rules! arfast {
        ($face:ident, $side: ident) => { arfast(RobotSide::$face, TableSide::$side) }
    }


    impl Order<'_> {
        fn apply(&self, asserv: &mut Asserv<MovementLowLevelHardware>, meca: &Meca) {
            match self {
                Order::GotoXy(x, y) => asserv.goto_xy(*x, *y),
                Order::GotoA(a) => asserv.goto_a(*a),
                Order::GotoXyA(x, y, a) => asserv.goto_xya(*x, *y, *a),
                Order::RunPath(path) => asserv.run_path(path),
                Order::MecaTake => {
                    meca.lower_arm_grab(0, 3);
                    meca.lower_arm_grab(0, 2);
                    meca.lower_arm_grab(0, 1);
                    meca.lower_arm_grab(0, 0);
                    log::info!("TO_DO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
                Order::MecaRaiseGrab => {
                    meca.grab(0, 0);
                    meca.raise_arm_grab(0, 3);
                    meca.raise_arm_grab(0, 2);
                    meca.raise_arm_grab(0, 1);
                    meca.raise_arm_grab(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
                Order::MecaRaiseDrop => {
                    meca.grab(0, 0);
                    meca.raise_arm_release(0, 3);
                    meca.raise_arm_release(0, 2);
                    meca.raise_arm_release(0, 1);
                    meca.raise_arm_release(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
                Order::MecaIdlePosGrab => {
                    meca.idle_arm_grab(0, 3);
                    meca.idle_arm_grab(0, 2);
                    meca.idle_arm_grab(0, 1);
                    meca.idle_arm_grab(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
                Order::MecaIdlePosDrop => {
                    meca.idle_arm_release(0, 3);
                    meca.idle_arm_release(0, 2);
                    meca.idle_arm_release(0, 1);
                    meca.idle_arm_release(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    thread::sleep(Duration::from_secs(2));
                }
            }
        }
    }

    let orders = [
        Order::MecaRaiseDrop,
        Order::GotoXyA(230.0, 200.0, arfast!(Left, Left)),
        Order::MecaIdlePosDrop,
        Order::RunPath(&[
            XY::new(230.0, 200.0),
            XY::new(200.0, 440.0),
            XY::new(130.0, 440.0),
        ]),
        Order::MecaTake,
        Order::MecaRaiseGrab,
        Order::GotoA(arfast(RobotSide::Left, TableSide::Down)),
        Order::RunPath(&[
            XY::new(100.0, 200.0),
            XY::new(0.0, 0.0),
        ]),
        Order::MecaRaiseDrop,
        Order::GotoXyA(0.0, 100.0, arfast(RobotSide::Left, TableSide::Down)),
    ];
    let mut index = 0;

    {
        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        asserv.goto_xya(0., 0., 0.);
    }

    if robot_color {
       led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 127, g: 127, b: 0 }}).ok();
    } else {
        led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 0, g: 0, b: 255 }}).ok();
    }

    while false { //index < orders.len() {
        index = index % orders.len();

        led_heartbeat.toggle().ok();
        motor_0_heartbeat.toggle().ok();
        motor_1_heartbeat.toggle().ok();
        motor_2_heartbeat.toggle().ok();

        thread::sleep(Duration::from_millis(500));

        {
            let meca_state = meca.get_state();
            log::info!("M0A0 : {} M0A1 : {} M0A2 : {} M0A3 : {}", 
                meca_state.arms[0][0].position, 
                meca_state.arms[0][1].position, 
                meca_state.arms[0][2].position, 
                meca_state.arms[0][3].position);

            for (i_module, module) in meca_state.arms.iter().enumerate() {
                for (i_arm, arm) in module.iter().enumerate() {
                    let _ = rome_tx.send(rome::Message::MecaArmTmState {
                        module: i_module as u8,
                        arm: i_arm as u8,
                        position: arm.position,
                        color: match arm.color {
                            1 => rome::params::MecaArmTmStateColor::Yellow,
                            2 => rome::params::MecaArmTmStateColor::Blue,
                            _ => rome::params::MecaArmTmStateColor::Unknown,
                        },
                        pump: arm.pump,
                        valve: arm.valve,
                        servo_error: arm.error,
                        torque_enabled: arm.flags.torque_enabled,
                        moving: arm.flags.moving,
                        // Note: position_reached == !moving
                        pump_current: arm.pump_current,
                    }.encode());
                }
            }
            for (i_tr, translation) in meca_state.translations.iter().enumerate() {
                let _ = rome_tx.send(rome::Message::MecaArmTmTranslation {
                    module: i_tr as u8,
                    position: translation.position,
                    error: translation.error,
                }.encode());
            }
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
            index = (index + 1);
        }
        //pause between orders
        //thread::sleep(Duration::from_secs(2));

    }
    log::info!("Job done!");

    loop {
        thread::sleep(Duration::from_millis(100));
    }
}
