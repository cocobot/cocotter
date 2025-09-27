/*mod config;
mod motors;
mod odometry;
mod pid;
mod motion_control;
mod trajectory;
mod safety;
mod line_sensors;
mod display;
mod ble_comm;
mod battery;
mod state;
mod robot;

use config::PAMIConfig;
use robot::Robot;

#[cfg(target_os = "espidf")]    
use board_pami::BoardPami;

#[cfg(target_os = "linux")]
use board_simulator::BoardPami;

fn main() {
    // Initialize logging
    esp_idf_svc::log::EspLogger::initialize_default();
    
    // Initialize board
    let board = BoardPami::new();

    // Get robot configuration based on MAC address
    let config = match PAMIConfig::get_config() {
        Some(config) => {
            log::info!("Board id {} is configured with color {:?}", config.id, config.color);
            config
        },
        None => {
            panic!("Board is not configured (mac address is {:?})", PAMIConfig::get_mac_address());
        }
    };

    // Create robot instance
    let mut robot = match Robot::new(board, config) {
        Ok(robot) => {
            log::info!("Robot created successfully");
            robot
        }
        Err(e) => {
            log::error!("Failed to create robot: {}", e);
            panic!("Robot initialization failed: {}", e);
        }
    };

    // Initialize robot systems
    if let Err(e) = robot.initialize() {
        log::error!("Failed to initialize robot: {:?}", e);
        panic!("Robot initialization failed: {:?}", e);
    }

    log::info!("PAMI Robot {} starting main loop", config.id);
    
    // Run robot main loop
    if let Err(e) = robot.run() {
        log::error!("Robot main loop error: {:?}", e);
        panic!("Robot runtime error: {:?}", e);
    }

    log::info!("PAMI Robot {} shutdown complete", config.id);
}
*/

use ble::GameControllerEvent;
#[cfg(target_os = "espidf")]
use board_pami::BoardPami;

use board_pami::MotorPwmType;
#[cfg(target_os = "linux")]
use board_simulator::BoardPami;

#[cfg(target_os = "espidf")]
use ble::BleComm;

#[cfg(target_os = "linux")]
use board_simulator::ble::BleComm;

fn set_pwm(left_pwm : &mut (MotorPwmType, MotorPwmType), right_pwm: &mut (MotorPwmType, MotorPwmType), d: f32, turn: f32, max_speed: f32) {

    let pwm_max = 1023;

    let turn_exp_coef = 75.0_f32;
    let turn_sign = -turn.signum();
    let turn_abs = turn.abs();
    let turn_curved = turn_sign * (turn_exp_coef.powf(turn_abs) - 1.0) / (turn_exp_coef - 1.0);

    let left_pwm_v = (d - turn_curved) * (pwm_max as f32) * max_speed;
    let right_pwm_v = (d + turn_curved) * (pwm_max  as f32) * max_speed;

    log::info!("d: {:.2}, turn: {:.2}, curved: {:.2} left_pwm: {:.2}, right_pwm: {:.2}", d, turn, turn_curved, left_pwm_v, right_pwm_v);

    let left_pwm_filtered = left_pwm_v.clamp(-(pwm_max as f32), pwm_max as f32) as i16;
    let right_pwm_filtered = right_pwm_v.clamp(-(pwm_max as f32), pwm_max as f32) as i16;
    
    if left_pwm_filtered >= 0 {
        let min_max = left_pwm_filtered.clamp(0, pwm_max as i16) as u32;
        left_pwm.0.set_duty(pwm_max).ok();
        left_pwm.1.set_duty(pwm_max - min_max).ok();
    }
    else {
        let min_max = (-left_pwm_filtered).clamp(0, pwm_max as i16) as u32;
        left_pwm.1.set_duty(pwm_max).ok();
        left_pwm.0.set_duty(pwm_max - min_max).ok();
    }

    if right_pwm_filtered >= 0 {
        let min_max = right_pwm_filtered.clamp(0, pwm_max as i16) as u32;
        right_pwm.0.set_duty(pwm_max).ok();
        right_pwm.1.set_duty(pwm_max - min_max).ok();
    }
    else {
        let min_max = (-right_pwm_filtered).clamp(0, pwm_max as i16) as u32;
        right_pwm.1.set_duty(pwm_max).ok();
        right_pwm.0.set_duty(pwm_max - min_max).ok();
    }             
}

fn main() {   
    let mut board = BoardPami::new();
    let enable_game_controller = true;

    let (_rome_tx, _rome_rx, game_controller_events) = BleComm::run(board.ble.take().unwrap(), "PAMI".to_string(), enable_game_controller);
    let mut left_motor = board.left_pwm.take().unwrap();
    let mut right_motor = board.right_pwm.take().unwrap();
    

    let mut d : f32 = 0.0;
    let mut turn = 0.0;
    let mut max_speed = 0.25;

    loop {
        if let Ok(event) = game_controller_events.recv() {
            match event {
                GameControllerEvent::Connected => {
                    log::info!("Game controller connected");
                }
                GameControllerEvent::ButtonUpPressed => {
                    d = 1.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonDownPressed => {
                    d = -1.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonLeftPressed => {
                    turn = -1.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonRightPressed => {
                    turn = 1.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonUpReleased | GameControllerEvent::ButtonDownReleased => {
                    d = 0.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonLeftReleased | GameControllerEvent::ButtonRightReleased => {
                    turn = 0.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonLeftJoyUpDownValue(value) => {                    
                    d = -(value as f32) / 128.0;
                    //d = -value as isize;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonLeftJoyLeftRightValue(value) => {
                    turn = (value as f32) / 128.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::Disconnected => {
                    log::info!("Game controller disconnected");
                    d = 0.0;
                    turn = 0.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonXPressed => {
                    max_speed = 0.25;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonSquarePressed => {
                    max_speed = 0.5;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonOPressed => {
                    max_speed = 0.75;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                GameControllerEvent::ButtonTrianglePressed=> {
                    max_speed = 1.0;
                    set_pwm(&mut left_motor, &mut right_motor, d, turn, max_speed);
                }
                _ => {                    
                }
            }
        }
    }
}