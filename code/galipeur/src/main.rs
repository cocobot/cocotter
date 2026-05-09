use asserv::holonomic::{conf::*};
use board_sabotter::SabotterBoard;
use galipeur::routines::GalipeurRoutines;
use galipeur::sensors::{GroundConf, GroundLidarConf, GroundLidarPose, TopLidarConf};
use galipeur::opponent_detection::{OpponentDetectionConf, TableConfig};

#[cfg(target_os = "espidf")]
type SabotterBoardImpl = board_sabotter::EspSabotterBoard;
#[cfg(not(target_os = "espidf"))]
use board_sabotter::MockSabotterBoard as SabotterBoardImpl;


fn main() {
    let mut board = SabotterBoardImpl::init();

    let mut routines = GalipeurRoutines::new(
        &mut board,
        TopLidarConf { angle_offset: 310.6 },
        OpponentDetectionConf {
            table: TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 150.0 },
            led_angle_offset: 162.0,
            corridor_half_width_mm: 250.0,
            corridor_stop_until_mm: 600.0,
            rotation_radius_mm: 400.0,
            slow_cruise_speed: 3.0,
        },
    );
    routines.asserv.lock().unwrap().set_conf(AsservConf {
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

    routines.sensors.set_conf(
        GroundLidarConf {
            modules: [
                // Module 0: lidar 0 and lidar 1
                [
                    GroundLidarPose { x: -57.09, y: 167.44, theta: 28.550_f32.to_radians() },  // r_min=629 mm @ θ_R=136.6°
                    GroundLidarPose { x: -53.44, y: -150.17, theta: -31.300_f32.to_radians() },  // r_min=634 mm @ θ_R=224.5°
                ],
                // Module 1: lidar 2 and lidar 3
                [
                    GroundLidarPose { x: -119.21, y: -127.60, theta: 150.400_f32.to_radians() },  // r_min=623 mm @ θ_R=15.3°
                    GroundLidarPose { x: -111.49, y: 127.89, theta: -150.650_f32.to_radians() },  // r_min=631 mm @ θ_R=344.7°
                ],
                // Module 2: lidar 4 and lidar 5
                [
                    GroundLidarPose { x: 173.62, y: -33.59, theta: -91.450_f32.to_radians() },  // r_min=629 mm @ θ_R=256.6°
                    GroundLidarPose { x: 163.86, y: 27.20, theta: 90.650_f32.to_radians() },  // r_min=632 mm @ θ_R=103.4°
                ],
            ],
        },
        GroundConf {
            thresholds: [50, 50, 50],
        },
    );

    routines.init();
    //routines.ground_sensor_calibration();

    loop {
        let now = std::time::Instant::now();
        routines.idle(&now);
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}
