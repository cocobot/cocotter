use alloc::sync::Arc;
use board_pami_2023::{LeftMotorPwms, RightMotorPwms, LeftWheelEncoder, RightWheelEncoder};
use cocotter::{pid::PID, position::{regular::RegularPosition, Position}, ramp::Ramp};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};

use crate::config::{ANGLE_PID_CONFIG, ANGLE_RAMP_CONFIG, ASSERV_PERIOD_MS, DISTANCE_PID_CONFIG, DISTANCE_RAMP_CONFIG, POSITION_CONFIG, ASSERV_PWM_OFFSET_MEAS_PERIOD_MS, ASSERV_DEAD_ZONE_SPEED};

pub type AsservMutexProtected = Arc<Mutex<CriticalSectionRawMutex, Asserv>>;

pub struct Asserv {
    left_wheel_counter: LeftWheelEncoder,
    right_wheel_counter: RightWheelEncoder,

    left_pwm: LeftMotorPwms,
    left_pwm_offset: [u16; 2],
    right_pwm: RightMotorPwms,
    right_pwm_offset: [u16; 2],

    last_encoders_read: [i16; 2],
    encoders: [i32; 2],

    position: RegularPosition,

    ramp_distance: Ramp,
    ramp_angle: Ramp,

    pid_distance: PID,
    pid_angle: PID,
}

impl Asserv {
    pub fn new(spawner: Spawner,
             left_wheel_counter: LeftWheelEncoder,
             right_wheel_counter: RightWheelEncoder,
             left_pwm: LeftMotorPwms,
             right_pwm: RightMotorPwms) -> AsservMutexProtected {  
        let instance = Arc::new(Mutex::new(Self {
            left_wheel_counter,
            right_wheel_counter,
            left_pwm,
            left_pwm_offset: [0, 0],
            right_pwm,
            right_pwm_offset: [0, 0],
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

    pub async fn run(instance: AsservMutexProtected) {

        let test = instance.lock().await;
       // test.ramp_distance.set_target(10.0);
        drop(test);

        let mut inst = instance.lock().await;

        let mut beg_cnt_val = inst.left_wheel_counter.get();

        let wait_duration_ms : u64 = 500;

        inst.left_pwm.0.set_timestamp(0);
        inst.left_pwm.1.set_timestamp(0);
        inst.right_pwm.0.set_timestamp(0);
        inst.right_pwm.1.set_timestamp(0);
        Timer::after(Duration::from_millis(wait_duration_ms)).await;
        for cnt in 0..500 {
            inst.left_pwm.0.set_timestamp( cnt);
            inst.left_pwm.1.set_timestamp(0);
            Timer::after(Duration::from_millis(ASSERV_PWM_OFFSET_MEAS_PERIOD_MS)).await;
            if (inst.left_wheel_counter.get() - beg_cnt_val).abs() > 10{
                inst.left_pwm_offset[0] = cnt as u16 -1;
                break;
            }
        }
        inst.left_pwm.0.set_timestamp(0);
        inst.left_pwm.1.set_timestamp(0);
        Timer::after(Duration::from_millis(wait_duration_ms)).await;
        beg_cnt_val = inst.left_wheel_counter.get();

        for cnt in 0..500 {
            inst.left_pwm.0.set_timestamp(0);
            inst.left_pwm.1.set_timestamp( cnt);
            Timer::after(Duration::from_millis(ASSERV_PWM_OFFSET_MEAS_PERIOD_MS)).await;
            if (inst.left_wheel_counter.get() - beg_cnt_val).abs() > 10{
                inst.left_pwm_offset[1] = cnt as u16 -1;
                break;
            }
        }

        inst.left_pwm.0.set_timestamp(0);
        inst.left_pwm.1.set_timestamp(0);
        inst.right_pwm.0.set_timestamp(0);
        inst.right_pwm.1.set_timestamp(0);
        Timer::after(Duration::from_millis(wait_duration_ms)).await;
        beg_cnt_val = inst.right_wheel_counter.get();
        for cnt in 0..500 {
            inst.right_pwm.0.set_timestamp( cnt);
            inst.right_pwm.1.set_timestamp(0);
            Timer::after(Duration::from_millis(ASSERV_PWM_OFFSET_MEAS_PERIOD_MS)).await;
            if (inst.right_wheel_counter.get() - beg_cnt_val).abs() > 10{
                inst.right_pwm_offset[0] = cnt as u16 -1;
                break;
            }
        }

        inst.right_pwm.0.set_timestamp(0);
        inst.right_pwm.1.set_timestamp(0);
        Timer::after(Duration::from_millis(wait_duration_ms)).await;
        beg_cnt_val = inst.right_wheel_counter.get();

        for cnt in 0..500 {
            inst.right_pwm.0.set_timestamp(0);
            inst.right_pwm.1.set_timestamp( cnt);
            Timer::after(Duration::from_millis(ASSERV_PWM_OFFSET_MEAS_PERIOD_MS)).await;
            if (inst.right_wheel_counter.get() - beg_cnt_val).abs() > 10{
                inst.right_pwm_offset[1] = cnt as u16 -1;
                break;
            }
        }
        inst.right_pwm.0.set_timestamp(0);
        inst.right_pwm.1.set_timestamp(0);
        Timer::after(Duration::from_millis(wait_duration_ms)).await;
        log::info!("pwm offsets left : {}|{} / right {}|{}", inst.left_pwm_offset[0], inst.left_pwm_offset[1], inst.right_pwm_offset[0], inst.right_pwm_offset[1]);


        drop(inst);
        

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
            let robot_angle = instance.position.get_angle_rad();
            
            //compute new control loop target
            let distance_target = instance.ramp_distance.compute();
            let angle_target = instance.ramp_angle.compute();

            //compute the control loop
            let distance_sp = instance.pid_distance.compute(distance_target - robot_distance);
            let angle_sp = instance.pid_angle.compute(angle_target - robot_angle);//robot_coord.get_a_deg());
            //let distance_sp = 0.0;
            //let angle_sp = 0.0;
            
            //assign the control loop output to the motors
            let left_speed = distance_sp - angle_sp;
            let right_speed = distance_sp + angle_sp;
        
            //let left_speed = (left_speed * 0.001).clamp(-25.0, 25.0);

            let left_pwm_offset = instance.left_pwm_offset.clone();
            let right_pwm_offset = instance.right_pwm_offset.clone();

            if (left_speed >= -ASSERV_DEAD_ZONE_SPEED) && (left_speed <= ASSERV_DEAD_ZONE_SPEED){
                    instance.left_pwm.0.set_timestamp(0);
                    instance.left_pwm.1.set_timestamp(0);
                }
            else if left_speed <= -ASSERV_DEAD_ZONE_SPEED {
                    instance.left_pwm.0.set_timestamp( (-left_speed) as u16 + left_pwm_offset[0]);
                    instance.left_pwm.1.set_timestamp(0);
                }
            else if left_speed >= ASSERV_DEAD_ZONE_SPEED {
                    instance.left_pwm.0.set_timestamp(0);
                    instance.left_pwm.1.set_timestamp(left_speed as u16 + left_pwm_offset[1]);
                }
            else {
                    instance.left_pwm.0.set_timestamp(0);
                    instance.left_pwm.1.set_timestamp(0);
                }

            /*match right_speed < 0.0 {
                true => {
                    instance.right_pwm.1.set_timestamp((-right_speed) as u16 + right_pwm_offset[0]);
                    instance.right_pwm.0.set_timestamp(0);
                }
                false => {
                    instance.right_pwm.1.set_timestamp(0);
                    instance.right_pwm.0.set_timestamp(right_speed as u16 + right_pwm_offset[1]);
                }
            }*/
            if (right_speed >= -ASSERV_DEAD_ZONE_SPEED) && (right_speed <= ASSERV_DEAD_ZONE_SPEED){
                    instance.right_pwm.0.set_timestamp(0);
                    instance.right_pwm.1.set_timestamp(0);
                }
            else if right_speed <= -ASSERV_DEAD_ZONE_SPEED {
                    instance.right_pwm.1.set_timestamp( (-right_speed) as u16 + right_pwm_offset[0]);
                    instance.right_pwm.0.set_timestamp(0);
                }
            else if right_speed >= ASSERV_DEAD_ZONE_SPEED {
                    instance.right_pwm.1.set_timestamp(0);
                    instance.right_pwm.0.set_timestamp(right_speed as u16 + right_pwm_offset[1]);
                }
            else {
                    instance.right_pwm.0.set_timestamp(0);
                    instance.right_pwm.1.set_timestamp(0);
                }


            log::info!("encoder L:{} R:{} / coord {:?}", encoder_latched.0, encoder_latched.1, robot_coord);
            //log::info!("ramp distance: {} / ramp angle: {}", instance.ramp_distance.get_output(), instance.ramp_angle.get_output());
            log::info!("left speed: {} / right speed: {}", left_speed, right_speed);
        }
    }

    pub async fn set_angle_target(instance : AsservMutexProtected, target : f32)
    {
        let mut instance = instance.lock().await;
        instance.ramp_angle.set_target(target);
    }

    pub async fn set_distance_target(instance : AsservMutexProtected, target : f32)
    {
        let mut instance = instance.lock().await;
        instance.ramp_distance.set_target(target);
    }

}

#[embassy_executor::task]
async fn start_asserv_thread(instance: AsservMutexProtected) {
    Asserv::run(instance).await;
}