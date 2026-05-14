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
        TopLidarConf { angle_offset: 40.6 },
        OpponentDetectionConf {
            table: TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 250.0 },
            led_angle_offset: 72.0,
            corridor_half_width_mm: 250.0,
            corridor_stop_until_mm: 550.0,
            corridor_slow_until_mm: 800.0,
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
            a_speed: 3.14 * 75.0,
            a_acc: 3.14 * 3.0,
            xy_cruise_speed: 6.0,
            xy_cruise_acc: 0.10,
            xy_steering_speed: 1.5,
            xy_steering_acc: 0.1,
            xy_stop_speed: 1.5,
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
                // Left face: L5 (low) + L0 (high)
                [
                    GroundLidarPose { x: -33.10, y: 158.56, theta: -179.850_f32.to_radians() },
                    GroundLidarPose { x: -167.31, y: -55.24, theta: 118.650_f32.to_radians() },
                ],
                // Back face: L2 (low) + L3 (high)
                [
                    GroundLidarPose { x: 129.50, y: -117.69, theta: -119.700_f32.to_radians() },
                    GroundLidarPose { x: -130.93, y: -110.39, theta: -60.450_f32.to_radians() },
                ],
                // Right face: L1 (low) + L4 (high)
                [
                    GroundLidarPose { x: 150.26, y: -55.35, theta: 58.800_f32.to_radians() },
                    GroundLidarPose { x: 36.00, y: 172.10, theta: -1.300_f32.to_radians() },
                ],
            ],
        },
        GroundConf {
            thresholds: [42, 42, 42],
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
