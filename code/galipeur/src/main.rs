use std::sync::{Arc, Mutex};
use std::time::Duration;
use asserv::holonomic::{conf::*, Asserv, RobotSide, TableSide};
use asserv::maths::XY;
use board_sabotter::SabotterBoard;
use galipeur::meca::Meca;
use galipeur::movement::MovementLowLevelHardware;
use galipeur::routines::GalipeurRoutines;

#[cfg(target_os = "espidf")]
type SabotterBoardImpl = board_sabotter::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
use board_sabotter::MockSabotterBoard as SabotterBoardImpl;


fn main() {
    let mut board = SabotterBoardImpl::init();

    let mut leds = board.leds().unwrap();

    let rome_server = board.rome("Galipeur".into()).unwrap();

    let mut routines = GalipeurRoutines::new(&mut board, rome_server);
    routines.asserv.set_conf(AsservConf {
        pid_x: PidConf {
            gain_p: 50,
            gain_i: 1,
            gain_d: 0,
            max_in: 0,
            max_i: 1000,
            max_out: 0,
            out_shift: 0,
        },
        pid_y: PidConf {
            gain_p: 50,
            gain_i: 1,
            gain_d: 0,
            max_in: 0,
            max_i: 1000,
            max_out: 0,
            out_shift: 0,
        },
        pid_a: PidConf {
            gain_p: 50,
            gain_i: 1,
            gain_d: 0,
            max_in: 0,
            max_i: 1000,
            max_out: 150000,
            out_shift: 0,
        },
        trajectory: TrajectoryConf {
            a_speed: 3.14 * 200.0,
            a_acc: 3.14 * 10.0,
            xy_cruise_speed: 10.0,
            xy_cruise_acc: 0.2,
            xy_steering_speed: 4.0,
            xy_steering_acc: 0.2,
            xy_stop_speed: 3.0,
            xy_stop_acc: 0.1,
            xy_steering_window: 50.0,
            xy_stop_window: 10.0,
            a_stop_window: 0.1,
            autoset_speed: 0.0,
            autoset_wait: 0,
            autoset_duration: 0,
        },
        motors: MotorsConf {
            velocities_to_consigns: [
                0.137193775559,     -0.227742535811,    32.7587578324,
                -0.267514745628,    0.000225842067981,  32.2910980339,
                0.138273262887,     0.235015679279,     32.2670974911,
            ],
            encoders_to_position: [
                -1.24627114282,     2.4735001584,       -1.21007913871,
                2.15287736186,      -0.0169008404017,   -2.16876778164,
                -0.0103397573436,   -0.010476522571,    -0.0100097003094,
            ],
        }
    });

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
        fn apply(&self, asserv: &mut Asserv<MovementLowLevelHardware<SabotterBoardImpl>>, meca: &Meca<SabotterBoardImpl>) {
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

    /*TODO
    if robot_color {
        led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 127, g: 127, b: 0 }}).ok();
    } else {
        led_sender.send(led::LedMessage::GameColor { color: RGB8 { r: 0, g: 0, b: 255 }}).ok();
    }
    */

    loop {
        let _ = routines.step_idle();

        let position = routines.asserv.cs.position();
        log::info!("Position: x: {:.2} y: {:.2} theta: {:.2}", position.x, position.y, position.a);

        if routines.asserv.done_xy() && routines.asserv.done_a() {
            let order = &orders[index];
            log::info!("Send new order: {:?}", order);
            order.apply(&mut routines.asserv, &routines.meca);
            index = (index + 1) % orders.len();
        }
    }
}
