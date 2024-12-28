use alloc::sync::Arc;
use board_pami_2023::{LeftMotorPwms, LeftWheelEncoder, RightWheelEncoder};
use cocotter::{pid::PID, position::{regular::RegularPosition, Position}, ramp::Ramp};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};

use crate::config::{ANGLE_PID_CONFIG, ANGLE_RAMP_CONFIG, ASSERV_PERIOD_MS, DISTANCE_PID_CONFIG, DISTANCE_RAMP_CONFIG, POSITION_CONFIG};

pub struct Asserv {
    left_wheel_counter: LeftWheelEncoder,
    right_wheel_counter: RightWheelEncoder,

    left_pwm: LeftMotorPwms,

    last_encoders_read: [i16; 2],
    encoders: [i32; 2],

    position: RegularPosition,

    ramp_distance: Ramp,
    ramp_angle: Ramp,

    pid_distance: PID,
    pid_angle: PID,
}

impl Asserv {
    pub fn new(spawner: Spawner, left_wheel_counter: LeftWheelEncoder, right_wheel_counter: RightWheelEncoder, left_pwm: LeftMotorPwms) -> Arc<Mutex<CriticalSectionRawMutex, Self>> {  
        let instance = Arc::new(Mutex::new(Self {
            left_wheel_counter,
            right_wheel_counter,
            left_pwm,
            last_encoders_read: [0, 0],
            encoders: [0, 0],

            position: RegularPosition::new(POSITION_CONFIG),

            ramp_distance: Ramp::new(DISTANCE_RAMP_CONFIG.with_timestep_ms(ASSERV_PERIOD_MS)),
            ramp_angle: Ramp::new(ANGLE_RAMP_CONFIG.with_timestep_ms(ASSERV_PERIOD_MS)),
        
            pid_distance: PID::new(DISTANCE_PID_CONFIG),
            pid_angle: PID::new(ANGLE_PID_CONFIG),
        }));

        spawner.spawn(start_asserv_thread(instance.clone())).unwrap();

        instance
    }

    pub async fn run(instance: Arc<Mutex<CriticalSectionRawMutex, Self>>) {

        let mut test = instance.lock().await;
       // test.ramp_distance.set_target(10.0);
        drop(test);

        loop {
            //run the asserv at 20Hz
            Timer::after(Duration::from_millis(ASSERV_PERIOD_MS)).await;

            let mut instance = instance.lock().await;

            //compute the encoder delta to get rid of the overflow
            let now = Instant::now().as_millis();
            let new_encoders = [instance.left_wheel_counter.get(), instance.right_wheel_counter.get()];
            for i in 0..2 {
                let delta = new_encoders[i].overflowing_sub(instance.last_encoders_read[i]).0;
                instance.encoders[i] += delta as i32;
                log::info!("delta: {:?}", delta);
                instance.last_encoders_read[i] = new_encoders[i];
            }

            //compute new position
            log::info!("encoders: {:?}", instance.encoders);
            let encoder_latched = (instance.encoders[0], instance.encoders[1]);
            instance.position.set_new_encoder_values(now, encoder_latched.0, encoder_latched.1);
            
            let robot_coord = instance.position.get_coordinates();
            let robot_distance = instance.position.get_distance_mm();
            
            //compute new control loop target
            let distance_target = instance.ramp_distance.compute();
            let angle_target = instance.ramp_angle.compute();

            //compute the control loop
            let distance_sp = instance.pid_distance.compute(distance_target - robot_distance);
            let angle_sp = instance.pid_angle.compute(angle_target - robot_coord.get_a_deg());
            
            //assign the control loop output to the motors
            let left_speed = distance_sp - angle_sp;
            let right_speed = distance_sp + angle_sp;
        
            let left_speed = (left_speed * 0.001).clamp(-25.0, 25.0);

            match left_speed < 0.0 {
                true => {
                    instance.left_pwm.0.set_timestamp( (-left_speed) as u16);
                    instance.left_pwm.1.set_timestamp(0);
                }
                false => {
                    instance.left_pwm.0.set_timestamp(0);
                    instance.left_pwm.1.set_timestamp(left_speed as u16);
                }
            }

            log::info!("encoder L:{} R:{} / coord {:?}", encoder_latched.0, encoder_latched.1, robot_coord);
            //log::info!("ramp distance: {} / ramp angle: {}", instance.ramp_distance.get_output(), instance.ramp_angle.get_output());
            log::info!("left speed: {} / right speed: {}", left_speed, right_speed);
        }
    }
}

#[embassy_executor::task]
async fn start_asserv_thread(instance: Arc<Mutex<CriticalSectionRawMutex, Asserv>>) {
    Asserv::run(instance).await;
}