use std::{sync::{Arc, Mutex}, time::{Duration, Instant}};

use board_pami_2023::{encoder::Encoder, EmergencyStop, MotorPwmType};
use cocotter::{pid::{PIDConfiguration, PID}, position::{Position, PositionMutex}};

use crate::{config::{ANGLE_PID_CONFIG, ANGLE_RAMP_CONFIG, ASSERV_PERIOD_MS, DISTANCE_PID_CONFIG, DISTANCE_RAMP_CONFIG, POSITION_CONFIG}, events::{Event, EventSystem}};

pub type AsservMutexProtected = Arc<Mutex<Asserv>>;

#[derive(Debug, Clone, Copy)]
pub struct MotorSetpointOverride {
    pub after_filter: bool,
    pub left: i16,
    pub right: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct PIDSetpointOverride {
    pub distance: f32,
    pub angle: f32,
}

pub struct Asserv {
    emergency_stop: EmergencyStop,

    left_wheel_counter: Encoder<'static>,
    right_wheel_counter: Encoder<'static>,

    left_pwm: (MotorPwmType, MotorPwmType),
    right_pwm: (MotorPwmType, MotorPwmType),

    last_encoders_read: [i32; 2],
    encoders: [i32; 2],

    position: PositionMutex<2>,

    pid_distance: PID,
    pid_angle: PID,

    motor_override_setpoint: Option<MotorSetpointOverride>,
    pid_override_setpoint: Option<PIDSetpointOverride>,
    pid_distance_override_config : Option<PIDConfiguration>,
    pid_angle_override_config : Option<PIDConfiguration>,
}

impl Asserv {
    pub fn new(
             emergency_stop: EmergencyStop,
             left_wheel_counter: Encoder<'static>,
             right_wheel_counter: Encoder<'static>,
             left_pwm: (MotorPwmType, MotorPwmType),
             right_pwm: (MotorPwmType, MotorPwmType),
             event: &EventSystem,
        ) -> AsservMutexProtected {  
        let instance = Arc::new(Mutex::new(Self {
            emergency_stop,
            left_wheel_counter,
            right_wheel_counter,
            left_pwm,
            right_pwm,
            last_encoders_read: [0, 0],
            encoders: [0, 0],

            position: Arc::new(Mutex::new(Position::new(
                POSITION_CONFIG, 
                [
                    ANGLE_RAMP_CONFIG.with_timestep_ms(ASSERV_PERIOD_MS),
                    DISTANCE_RAMP_CONFIG.with_timestep_ms(ASSERV_PERIOD_MS)
                ]
            ))),
        
            pid_distance: PID::new(DISTANCE_PID_CONFIG),
            pid_angle: PID::new(ANGLE_PID_CONFIG),

            motor_override_setpoint: None,
            pid_override_setpoint: None,
            pid_distance_override_config: None,
            pid_angle_override_config: None,
        }));

        let cloned_instance = instance.clone();
        let cloned_event = event.clone();
        std::thread::Builder::new()
            .stack_size(8192)
            .name("Asserv".to_string())
            .spawn(move || {
                Asserv::run(cloned_instance, cloned_event);
            })
            .unwrap();

        instance
    }


    pub fn run(instance: AsservMutexProtected, event: EventSystem) {
        let mut last_event_sent= Instant::now();

        loop {
            //run the asserv at 100Hz
            std::thread::sleep(Duration::from_millis(ASSERV_PERIOD_MS));

            let mut instance = instance.lock().unwrap();

            //compute the encoder delta to get rid of the overflow
            let now = Instant::now();
            let new_encoders = [instance.left_wheel_counter.get_value().unwrap(), instance.right_wheel_counter.get_value().unwrap()];
            let mut moving_forward: [bool; 2] = [false; 2];
            let mut moving_backward: [bool; 2] = [false; 2];
            for i in 0..2 {
                let delta = -new_encoders[i].overflowing_sub(instance.last_encoders_read[i]).0;
                
                instance.encoders[i] += delta as i32;
                instance.last_encoders_read[i] = new_encoders[i];

                if delta > 0 {
                    moving_forward[i] = true;
                }
                else if delta < 0 {
                    moving_backward[i] = true;
                }
            }
            //log::info!("Encoders: {} {}", instance.encoders[0], instance.encoders[1]);
        
            //compute new position
            let mut position = instance.position.lock().unwrap();
            position.set_new_encoder_values(now, instance.encoders, [1.0, 1.0]);
            
            let robot_coord = position.get_coordinates();
            if (now - last_event_sent).as_millis() > 100 {
                event.send_event(Event::Position { coords: *robot_coord });
            }
            let robot_distance = robot_coord.get_distance_mm();
            let robot_angle = robot_coord.get_angle_rad();

            //log::info!("Position: {:?} {} {} {}", instance.encoders, robot_distance, robot_angle.to_degrees(), now.elapsed().as_millis());
            
            //compute new control loop target
            let ramp = position.get_ramps_as_mut();
            let mut distance_target = ramp[1].compute();
            let mut angle_target = ramp[0].compute();
            drop(position);

            if let Some(ovr) = &instance.pid_override_setpoint {
                distance_target = ovr.distance;
                angle_target = ovr.angle;
            }

            if let Some(ovr) = instance.pid_distance_override_config {
                instance.pid_distance.set_configuration(ovr);
            }
            if let Some(ovr) = instance.pid_angle_override_config {
                instance.pid_angle.set_configuration(ovr);
            }

            //compute the control loop
            let distance_sp = instance.pid_distance.compute(distance_target - robot_distance);
            let angle_sp = instance.pid_angle.compute(angle_target - robot_angle);

          //  log::info!("PID D: {:8.3} {:8.3} {:8.3} {:8.3} || PID A: {:8.3} {:8.3} {:8.3} {:8.3}", distance_target, robot_distance, distance_sp, distance_target - robot_distance, angle_target, robot_angle, angle_sp, angle_target - robot_angle);
            //assign the control loop output to the motors
            let left_speed = distance_sp - angle_sp;
            let right_speed = distance_sp + angle_sp;

            let mut left_pwm : i16 = left_speed.clamp(-1000.0, 1000.0) as i16;
            let mut right_pwm : i16 = right_speed.clamp(-1000.0, 1000.0) as i16;

            if let Some(ovr) = &instance.motor_override_setpoint {
                if !ovr.after_filter {
                    left_pwm = ovr.left;
                    right_pwm = ovr.right;
                }
            }

            let mut left_pwm_filtered = left_pwm;
            let mut right_pwm_filtered = right_pwm;

            if let Some(ovr) = &instance.motor_override_setpoint {
                if ovr.after_filter {
                    left_pwm_filtered = ovr.left;
                    right_pwm_filtered = ovr.right;
                }
            }

            if instance.emergency_stop.is_low() {
                left_pwm_filtered = 0;
                right_pwm_filtered = 0;
            }

            if (now - last_event_sent).as_millis() > 100 {
                last_event_sent = now;
               
                event.send_event(Event::MotorDebug { timestamp: (now.elapsed().as_millis() & 0xFFFF) as u16, left_tick: instance.encoders[0], right_tick: instance.encoders[1], left_pwm: left_pwm_filtered, right_pwm: right_pwm_filtered });
                event.send_event(Event::PIDDebug { 
                    timestamp: (now.elapsed().as_millis() & 0xFFFF) as u16,
                    target_d: distance_target,
                    current_d: robot_distance,
                    output_d: distance_sp,
                    target_a: angle_target,
                    current_a: robot_angle,
                    output_a: angle_sp,
                });
            }
            
            let pwm_max: u32 = 1023;
            if left_pwm_filtered >= 0 {
                let min_max = left_pwm_filtered.clamp(0, pwm_max as i16) as u32;
                instance.left_pwm.0.set_duty(pwm_max).ok();
                instance.left_pwm.1.set_duty(pwm_max - min_max).ok();
            }
            else {
                let min_max = (-left_pwm_filtered).clamp(0, pwm_max as i16) as u32;
                instance.left_pwm.1.set_duty(pwm_max).ok();
                instance.left_pwm.0.set_duty(pwm_max - min_max).ok();
            }

            if right_pwm_filtered >= 0 {
                let min_max = right_pwm_filtered.clamp(0, pwm_max as i16) as u32;
                instance.right_pwm.0.set_duty(pwm_max).ok();
                instance.right_pwm.1.set_duty(pwm_max - min_max).ok();
            }
            else {
                let min_max = (-right_pwm_filtered).clamp(0, pwm_max as i16) as u32;
                instance.right_pwm.1.set_duty(pwm_max).ok();
                instance.right_pwm.0.set_duty(pwm_max - min_max).ok();
            }
            /*
            if right_pwm_filtered > 0 {
                instance.right_dir.set_high().unwrap();
                instance.right_pwm.set_duty(1023 - right_pwm_filtered.clamp(0, 1023) as u32).ok();
            }
            else {
                instance.right_dir.set_low().unwrap();
                instance.right_pwm.set_duty((-right_pwm_filtered).clamp(0, 1023) as u32).ok();
            }
            */
        }

    }

    pub fn get_position(&self) -> PositionMutex<2> {
        self.position.clone()
    }
}