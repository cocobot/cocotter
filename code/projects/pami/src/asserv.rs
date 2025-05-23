use board_pami_2023::{LeftMotorPwms, RightMotorPwms, LeftWheelEncoder, RightWheelEncoder};
use cocotter::{pid::PID, position::{Position, PositionMutex}};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use alloc::sync::Arc;
use esp_hal::gpio::Input;

use crate::{config::{MotorConfiguration, MotorQuadrantConfiguration, PAMIConfig, ANGLE_PID_CONFIG, ANGLE_RAMP_CONFIG, ASSERV_PERIOD_MS, DISTANCE_PID_CONFIG, DISTANCE_RAMP_CONFIG, POSITION_CONFIG}, events::{Event, EventSystem}};

pub type AsservMutexProtected = Arc<Mutex<CriticalSectionRawMutex, Asserv>>;

#[derive(Debug, Clone, Copy)]
pub struct MotorSetpointOverride {
    pub after_filter: bool,
    pub left: i16,
    pub right: i16,
}

pub struct Asserv {
    emergency_stop: Input<'static>,

    left_wheel_counter: LeftWheelEncoder,
    right_wheel_counter: RightWheelEncoder,

    left_pwm: LeftMotorPwms,
    right_pwm: RightMotorPwms,

    last_encoders_read: [i16; 2],
    encoders: [i32; 2],

    position: PositionMutex<CriticalSectionRawMutex, 2>,

    pid_distance: PID,
    pid_angle: PID,

    motor_override_setpoint: Option<MotorSetpointOverride>,
}

impl Asserv {
    pub async fn new(
             emergency_stop: Input<'static>,
             left_wheel_counter: LeftWheelEncoder,
             right_wheel_counter: RightWheelEncoder,
             left_pwm: LeftMotorPwms,
             right_pwm: RightMotorPwms,
             spawner: Spawner,
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
        }));

        spawner.spawn(start_asserv_thread(instance.clone(), event.clone())).unwrap();

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
                        _ => {}
                    }
                }
            }
        ).await;

        instance
    }

    fn filter_events(evt: &Event) -> bool {
        match evt {
            Event::MotorOverrideSetpoint { .. } => true,
            _ => false,
        }
    } 


    pub async fn run(instance: AsservMutexProtected, event: EventSystem) {
        let config = PAMIConfig::get_config().unwrap();

        let mut left_motor_filter = MotorFilter::new(&config.left_motor);
        let mut right_motor_filter = MotorFilter::new(&config.right_motor);


        loop {
            //run the asserv at 20Hz
            Timer::after(Duration::from_millis(ASSERV_PERIOD_MS)).await;

            let mut instance = instance.lock().await;

            //compute the encoder delta to get rid of the overflow
            let now = Instant::now().as_millis();
            let new_encoders = [instance.left_wheel_counter.get(), instance.right_wheel_counter.get()];
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

            //compute new position
            let mut position = instance.position.lock().await;
            position.set_new_encoder_values(now, instance.encoders);
            
            let robot_coord = position.get_coordinates();
            event.send_event(Event::Position { coords: *robot_coord });
            let robot_distance = robot_coord.get_distance_mm();
            let robot_angle = robot_coord.get_angle_rad();
            
            //compute new control loop target
            let ramp = position.get_ramps_as_mut();
            let distance_target = ramp[1].compute();
            let angle_target = ramp[0].compute();
            drop(position);

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

            let mut left_pwm_filtered = left_motor_filter.apply_pwm(left_pwm, moving_forward[0], moving_backward[0]);
            let mut right_pwm_filtered = right_motor_filter.apply_pwm(right_pwm, moving_forward[1], moving_backward[1]);

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

            event.send_event(Event::MotorDebug { timestamp: (now & 0xFFFF) as u16, left_tick: instance.encoders[0], right_tick: instance.encoders[1], left_pwm: left_pwm_filtered, right_pwm: right_pwm_filtered });

            if left_pwm_filtered >= 0 {
                instance.left_pwm.1.set_timestamp(left_pwm_filtered as u16);
                instance.left_pwm.0.set_timestamp(0);
            }
            else {
                instance.left_pwm.1.set_timestamp(0);
                instance.left_pwm.0.set_timestamp((-left_pwm_filtered) as u16);
            }

            if right_pwm_filtered >= 0 {
                instance.right_pwm.0.set_timestamp(right_pwm_filtered as u16);
                instance.right_pwm.1.set_timestamp(0);
            }
            else {
                instance.right_pwm.0.set_timestamp(0);
                instance.right_pwm.1.set_timestamp((-right_pwm_filtered) as u16);
            }
        }
    }

    pub fn get_position(&self) -> PositionMutex<CriticalSectionRawMutex, 2> {
        self.position.clone()
    }
}

#[embassy_executor::task]
async fn start_asserv_thread(instance: AsservMutexProtected, event: EventSystem) {
    Asserv::run(instance, event).await;
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