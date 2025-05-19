use std::{sync::{Arc, Mutex}, time::{Duration, Instant}};

use board_pami_2023::{encoder::Encoder, EmergencyStop, MotorLeftDirType, MotorPwmType, MotorRightDirType};
use cocotter::{pid::{PIDConfiguration, PID}, position::{Position, PositionMutex}};
use esp_idf_svc::hal::{task::thread::ThreadSpawnConfiguration};

use crate::{config::{MotorConfiguration, MotorQuadrantConfiguration, PAMIConfig, ANGLE_PID_CONFIG, ANGLE_RAMP_CONFIG, ASSERV_PERIOD_MS, DISTANCE_PID_CONFIG, DISTANCE_RAMP_CONFIG, POSITION_CONFIG}, events::{Event, EventSystem}};

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

    left_pwm: MotorPwmType,
    right_pwm: MotorPwmType,
    left_dir: MotorLeftDirType,
    right_dir: MotorRightDirType,

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
             left_pwm: MotorPwmType,
             right_pwm: MotorPwmType,
             left_dir: MotorLeftDirType,
             right_dir: MotorRightDirType,
             event: &EventSystem,
        ) -> AsservMutexProtected {  
        let instance = Arc::new(Mutex::new(Self {
            emergency_stop,
            left_wheel_counter,
            right_wheel_counter,
            left_pwm,
            right_pwm,
            left_dir,
            right_dir,
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
        ThreadSpawnConfiguration {
            name: Some("Asserv\0".as_bytes()),
            stack_size: 8192,
            ..Default::default()
        }.set().unwrap();
        std::thread::spawn(|| Asserv::run(cloned_instance, cloned_event));

        /* 
        let ble_instance = instance.clone();
        event.register_receiver_callback(
            Some(Asserv::filter_events), 
            move |evt| {
                let ble_instance = ble_instance.clone();
                async move {
                    match evt {
                        Event::MotorOverrideSetpoint { ovr }  => {
                            ble_instance.lock().await.motor_override_setpoint = ovr;
                        }
                        Event::PidOverrideSetpoint { ovr } => {
                            ble_instance.lock().await.pid_override_setpoint = ovr;
                        }
                        Event::PidOverrideConfiguration { ovr, pid_id } => {
                            match pid_id {
                                0 => ble_instance.lock().await.pid_distance_override_config = ovr,
                                1 => ble_instance.lock().await.pid_angle_override_config = ovr,
                                _ => {}
                            }
                        }
                        _ => {}
                    }
                }
            }
        ).await;
        */
        instance
    }

    fn filter_events(evt: &Event) -> bool {
        match evt {
            Event::MotorOverrideSetpoint { .. } => true,
            Event::PidOverrideSetpoint { .. } => true,
            Event::PidOverrideConfiguration { .. } => true,
            _ => false,
        }
    } 


    pub fn run(instance: AsservMutexProtected, event: EventSystem) {
        let config = PAMIConfig::get_config().unwrap();

        let mut left_motor_filter = MotorFilter::new(&config.left_motor);
        let mut right_motor_filter = MotorFilter::new(&config.right_motor);

        let mut last_event_sent= Instant::now();

        loop {
            //run the asserv at 20Hz
            std::thread::sleep(Duration::from_millis(ASSERV_PERIOD_MS));

            let mut instance = instance.lock().unwrap();

            //compute the encoder delta to get rid of the overflow
            let now = Instant::now();
            let new_encoders = [instance.left_wheel_counter.get_value().unwrap(), instance.right_wheel_counter.get_value().unwrap()];
            let mut moving_forward: [bool; 2] = [false; 2];
            let mut moving_backward: [bool; 2] = [false; 2];
            for i in 0..2 {
                let mut delta = new_encoders[i].overflowing_sub(instance.last_encoders_read[i]).0;
                if config.invert_encoder[i] {
                    delta = -delta;
                }
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
            position.set_new_encoder_values(now, instance.encoders);
            
            let robot_coord = position.get_coordinates();
            event.send_event(Event::Position { coords: *robot_coord });
            let robot_distance = robot_coord.get_distance_mm();
            let robot_angle = robot_coord.get_angle_rad();
            
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

            //log::info!("PID D: {:8.3} {:8.3} {:8.3} {:8.3}", distance_target, robot_distance, distance_sp, distance_target - robot_distance);
            //log::info!("PID A: {:8.3} {:8.3} {:8.3} {:8.3}", angle_target, robot_angle, angle_sp, angle_target - robot_angle);
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

            //let mut left_pwm_filtered = left_motor_filter.apply_pwm(left_pwm, moving_forward[0], moving_backward[0]);
            //let mut right_pwm_filtered = right_motor_filter.apply_pwm(right_pwm, moving_forward[1], moving_backward[1]);

            if let Some(ovr) = &instance.motor_override_setpoint {
                if ovr.after_filter {
                    left_pwm_filtered = ovr.left;
                    right_pwm_filtered = ovr.right;
                }
            }

            let gain = 7;

            left_pwm_filtered = -75;
            right_pwm_filtered = -75* gain;

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
            
            if left_pwm_filtered >= 0 {
                instance.left_dir.set_low().unwrap();
                instance.left_pwm.set_duty(left_pwm_filtered.clamp(0, 1023) as u32).ok();
            }
            else {
                instance.left_dir.set_high().unwrap();
                instance.left_pwm.set_duty(1023 - (-left_pwm_filtered).clamp(0, 1023) as u32).ok();
            }

            if right_pwm_filtered >= 0 {
                instance.right_dir.set_high().unwrap();
                instance.right_pwm.set_duty(1023 - right_pwm_filtered.clamp(0, 1023) as u32).ok();
            }
            else {
                instance.right_dir.set_low().unwrap();
                instance.right_pwm.set_duty((-right_pwm_filtered).clamp(0, 1023) as u32).ok();
            }
        }
    }

    pub fn get_position(&self) -> PositionMutex<2> {
        self.position.clone()
    }
}


enum MotorFilterState {
    Off,
    Forward,
    Backward,
}

struct MotorFilter {
    state: MotorFilterState,
    config: MotorConfiguration,
}

impl MotorFilter {
    const THRESHOLD: i16 = 5;

    fn new(config: &MotorConfiguration) -> Self {
        MotorFilter {
            state: MotorFilterState::Off,
            config: *config,
        }
    }

    pub fn apply_pwm(&mut self, pwm: i16, moving_forward: bool, moving_backward: bool) -> i16 {
        let apply_gain =|pwm: i16, config: MotorQuadrantConfiguration| {
            (((pwm - config.min_pwm) as f32) * config.gain) as i16 
        };

        let res = if pwm > MotorFilter::THRESHOLD {
            match self.state {
                MotorFilterState::Off | MotorFilterState::Backward => {
                    if moving_forward {
                        self.state = MotorFilterState::Forward;
                        return self.apply_pwm(pwm, moving_forward, moving_backward);
                    }
                    else {
                        self.config.forward.boost_pwm + apply_gain(pwm, self.config.forward)
                    }
                }
                MotorFilterState::Forward => {
                    self.config.forward.min_pwm + apply_gain(pwm, self.config.forward)
                }
            }
        }
        else if pwm < -MotorFilter::THRESHOLD {
            match self.state {
                MotorFilterState::Off | MotorFilterState::Forward => {
                    if moving_backward {
                        self.state = MotorFilterState::Backward;
                        return self.apply_pwm(pwm, moving_forward, moving_backward);
                    }
                    else {
                        -self.config.backward.boost_pwm - apply_gain(-pwm, self.config.backward)
                    }
                }
                MotorFilterState::Backward => {
                    -self.config.backward.min_pwm - apply_gain(-pwm, self.config.backward)
                }
            }
        }
        else {
            self.state = MotorFilterState::Off;
            0
        };

        if self.config.invert {
            -res
        }
        else {
            res
        }
    }
}