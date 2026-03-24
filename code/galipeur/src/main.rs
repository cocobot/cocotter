mod movement;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use asserv::holonomic::Asserv;
use asserv::maths::XY;
use board_sabotter::{SabotterBoard, SabotterMotor};
use sch16t::Sch16t;
use movement::{Movement, MovementLowLevelHardware};

#[cfg(target_os = "espidf")]
type SabotterBoardImpl = board_sabotter::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
use board_sabotter::MockSabotterBoard as SabotterBoardImpl;

type SabotterMotorImpl = SabotterMotor<<SabotterBoardImpl as SabotterBoard>::MotorEncoder, <SabotterBoardImpl as SabotterBoard>::MotorPwm>;
type GyroImpl = Sch16t<<SabotterBoardImpl as SabotterBoard>::Spi>;


fn main() {
    let mut board = SabotterBoardImpl::init();

    // Configure gyro
    let mut gyro = GyroImpl::new(board.imu_spi().unwrap(), 0);
    gyro.init().unwrap();

    let mut leds = board.leds().unwrap();

    // Configure low level hardware for asserv
    let asserv_hardware = MovementLowLevelHardware::new(
        gyro,
        board.motors().unwrap(),
    );
    let movement = Arc::new(Mutex::new(Movement::new(asserv_hardware)));

    #[derive(Debug)]
    enum Order<'a> {
        GotoXy(f32, f32),
        GotoA(f32),
        GotoXyA(f32, f32, f32),
        RunPath(&'a [XY]),
    }

    impl Order<'_> {
        fn apply(&self, asserv: &mut Asserv<MovementLowLevelHardware>) {
            match self {
                Order::GotoXy(x, y) => asserv.goto_xy(*x, *y),
                Order::GotoA(a) => asserv.goto_a(*a),
                Order::GotoXyA(x, y, a) => asserv.goto_xya(*x, *y, *a),
                Order::RunPath(path) => asserv.run_path(path),
            }
        }
    }

    let orders = [
        Order::RunPath(&[
            XY::new(0.0, 500.0),
            XY::new(500.0, 500.0),
        ]),
        Order::GotoA(std::f32::consts::PI * 0.9),
        Order::RunPath(&[
            XY::new(500.0, 0.0),
            XY::new(0.0, 0.0),
        ]),
        Order::GotoXyA(-200.0, 200.0, 0.0),
    ];
    let mut index = 0;

    {
        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        asserv.goto_xya(0., 0., 0.);
    }

    loop {
        let _ = leds.com.toggle();
        std::thread::sleep(Duration::from_millis(500));

        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        let position = asserv.cs.position().clone();

        log::info!("Position: x: {:.2} y: {:.2} theta: {:.2}", position.x, position.y, position.a);

        if asserv.done_xy() && asserv.done_a() {
            let order = &orders[index];
            log::info!("Send new order: {:?}", order);
            order.apply(&mut asserv);
            index = (index + 1) % orders.len();
        }
    }
}
