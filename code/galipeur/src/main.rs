use std::sync::{Arc, Mutex};
use std::time::Duration;
use asserv::holonomic::{Asserv, RobotSide, TableSide};
use asserv::maths::XY;
use board_sabotter::SabotterBoard;
use cancaner::esp::CanInterface;
use cancaner::Color as CanColor;
use sch16t::Sch16t;
use galipeur::meca::Meca;
use galipeur::movement::{Movement, MovementLowLevelHardware};

#[cfg(target_os = "espidf")]
type SabotterBoardImpl = board_sabotter::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
use board_sabotter::MockSabotterBoard as SabotterBoardImpl;


fn main() {
    let mut board = SabotterBoardImpl::init();

    // Configure gyro
    let mut gyro = Sch16t::new(board.imu_spi().unwrap(), 0);
    gyro.init().unwrap();

    let mut leds = board.leds().unwrap();

    let (rome_tx, rome_rx) = board.rome("Galipeur".into()).unwrap();

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
        MecaTake,
        MecaRaiseGrab,
        MecaRaiseDrop,
        MecaIdlePosGrab,
        MecaIdlePosDrop,
    }

    let mut robot_color = true;

    let robot_side_main = if robot_color { RobotSide::Right } else { RobotSide::Left };
    let robot_side_aux  = if robot_color { RobotSide::Left } else { RobotSide::Right };
    let table_side_main = if robot_color { TableSide::Left } else { TableSide::Right };
    let table_side_aux  = if robot_color { TableSide::Right } else { TableSide::Left };

    //function to get angle to apply to Align Robot Face Along given Side of the Table
    fn arfast(face: RobotSide, side: TableSide) -> f32 {
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
        fn apply(&self, asserv: &mut Asserv<MovementLowLevelHardware<SabotterBoardImpl>>, meca: &Meca) {
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
                    std::thread::sleep(Duration::from_secs(2));
                }
                Order::MecaRaiseGrab => {
                    meca.grab(0, 0);
                    meca.raise_arm_grab(0, 3);
                    meca.raise_arm_grab(0, 2);
                    meca.raise_arm_grab(0, 1);
                    meca.raise_arm_grab(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    std::thread::sleep(Duration::from_secs(2));
                }
                Order::MecaRaiseDrop => {
                    meca.grab(0, 0);
                    meca.raise_arm_release(0, 3);
                    meca.raise_arm_release(0, 2);
                    meca.raise_arm_release(0, 1);
                    meca.raise_arm_release(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    std::thread::sleep(Duration::from_secs(2));
                }
                Order::MecaIdlePosGrab => {
                    meca.idle_arm_grab(0, 3);
                    meca.idle_arm_grab(0, 2);
                    meca.idle_arm_grab(0, 1);
                    meca.idle_arm_grab(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    std::thread::sleep(Duration::from_secs(2));
                }
                Order::MecaIdlePosDrop => {
                    meca.idle_arm_release(0, 3);
                    meca.idle_arm_release(0, 2);
                    meca.idle_arm_release(0, 1);
                    meca.idle_arm_release(0, 0);
                    log::info!("TODO wait with feedback from meca");
                    std::thread::sleep(Duration::from_secs(2));
                }

            }
        }
    }

    let orders = [
        //Order::RunPath(&[
        //    XY::new(0.0, 500.0),
        //    XY::new(500.0, 500.0),
        //]),
        Order::GotoXyA(0.0, 0.0, arfast!(Back, Down)),
        Order::MecaIdlePosDrop,
        Order::MecaTake,
        //Order::RunPath(&[
        //    XY::new(500.0, 0.0),
        //    XY::new(0.0, 0.0),
        //]),
        Order::MecaRaiseGrab,
        Order::GotoXyA(50.0, 0.0, arfast(RobotSide::Back, TableSide::Down)),
        Order::MecaRaiseDrop,
    ];
    let mut index = 0;

    {
        let movement = movement.lock().unwrap();
        let asserv = movement.get_asserv();
        let mut asserv = asserv.lock().unwrap();
        asserv.goto_xya(0., 0., 0.);
    }

    /*TODO
    if robot_color {
        led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 127, g: 127, b: 0 }}).ok();
    } else {
        led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 0, g: 0, b: 255 }}).ok();
    }
    */

    let can_iface = CanInterface::new(board.can().unwrap());
    can_iface.add_log_callback("picotter");

    let meca = Meca::new(&can_iface);

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
            order.apply(&mut asserv, &meca);
            index = (index + 1) % orders.len();
        }

        let meca_state = meca.get_state();
        for (i_module, module) in meca_state.arms.iter().enumerate() {
            for (i_arm, arm) in module.iter().enumerate() {
                let _ = rome_tx.send(rome::Message::MecaArmTmState {
                    module: i_module as u8,
                    arm: i_arm as u8,
                    position: arm.position,
                    color: match arm.color {
                        CanColor::Unknown => rome::params::MecaArmTmStateColor::Unknown,
                        CanColor::Yellow => rome::params::MecaArmTmStateColor::Yellow,
                        CanColor::Blue => rome::params::MecaArmTmStateColor::Blue,
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
}
