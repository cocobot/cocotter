use asserv::holonomic::{conf::*};
use board_sabotter::SabotterBoard;
use galipeur::routines::GalipeurRoutines;

#[cfg(target_os = "espidf")]
type SabotterBoardImpl = board_sabotter::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
use board_sabotter::MockSabotterBoard as SabotterBoardImpl;


fn main() {
    let mut board = SabotterBoardImpl::init();

    let mut routines = GalipeurRoutines::new(&mut board);
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

    //routines.ground_sensor_calibration();

    loop {
        routines.step_idle();
    }
}