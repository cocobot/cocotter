use core::f32::consts::{PI, TAU}; // TAU = 2 * PI

use either::Either;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Timer};
use libm::{atan2f, floorf, sqrtf};

use crate::position::PositionMutex;

use super::{OrderConfig, TrajectorError};

#[derive(Debug)]
pub enum Order {
    GotoD { d_mm: f32 },
    GotoA { a_rad: f32 },
    GotoXY { x_mm: f32, y_mm: f32 },
    
    MoveUntil { callback: fn() -> bool },

    Label { label: &'static str },
    GotoLabel { label: &'static str },
    GotoRel { rel_step: usize },
    GotoRelStepOrLabel { callback: fn() -> Either<usize, &'static str> },
}

impl Order {
    const A_INDEX: usize = 0;
    const D_INDEX_IN_NON_HOLONOMIC_ROBOT: usize = 1;

    pub async fn execute<M: RawMutex, const N: usize>(&self, order_index: usize, config: &OrderConfig<N>, position: &PositionMutex<M, N>) -> Result<usize, TrajectorError> {
        match self {
            Order::GotoD { d_mm } => {
                match N {
                    2 => {
                        let mut locked_position = position.lock().await;
                        let initial_position = locked_position.get_coordinates();
                        let initial_d = initial_position.get_raw_linear_coordonate()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                        if config.is_backwards() {
                            locked_position.get_ramps_as_mut()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d - d_mm);
                        }
                        else {
                            locked_position.get_ramps_as_mut()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d + d_mm);
                        }
                        drop(locked_position);
        
                        loop {
                            let locked_position = position.lock().await;
                            let d_ramp =  &locked_position.get_ramps()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                            if d_ramp.is_done() {
                                break;
                            }
                            drop(locked_position);
                            
                            Timer::after(Duration::from_millis(75)).await;
                        }
                        
                        Ok(order_index + 1)
                    }

                    _ => {
                        Err(TrajectorError::InvalidOrder)
                    }
                }
            }
            Order::GotoA { a_rad } => {                        
                let mut locked_position = position.lock().await;
                let initial_position = locked_position.get_coordinates();
                let initial_a = initial_position.get_a_rad();
                if config.is_backwards() {
                    locked_position.get_ramps_as_mut()[Order::A_INDEX].set_target(Order::find_closest_angle(initial_a, *a_rad));
                }
                else {
                    locked_position.get_ramps_as_mut()[Order::A_INDEX].set_target(Order::find_closest_angle(initial_a, *a_rad + PI));
                }
                drop(locked_position);

                loop {
                    let locked_position = position.lock().await;
                    let a_ramp =  &locked_position.get_ramps()[Order::A_INDEX];
                    if a_ramp.is_done() {
                        break;
                    }
                    drop(locked_position);
                    
                    Timer::after(Duration::from_millis(75)).await;
                }
                
                Ok(order_index + 1)
            }

            Order::GotoXY { x_mm, y_mm } => {
                if N != 2 {
                    return Err(TrajectorError::InvalidOrder);
                }

                loop {
                    let mut locked_position = position.lock().await;
                    
                    let initial_position = locked_position.get_coordinates();
                    let delta_x = x_mm - initial_position.get_x_mm();
                    let delta_y = y_mm - initial_position.get_y_mm();

                    //compute ramp targets
                    let target_a = atan2f(delta_y, delta_x);
                    let target_a = match config.is_backwards() {
                        false => Order::find_closest_angle(initial_position.get_a_rad(), target_a),
                        true => Order::find_closest_angle(initial_position.get_a_rad(), target_a + PI),
                    };

                    let angle_diff = (target_a - initial_position.get_a_rad()).abs();
                    let target_d = match angle_diff < config.max_angle_diff_before_stop {
                        true => sqrtf(delta_x * delta_x + delta_y * delta_y),
                        false => 0.0,
                    };
                    let target_d = match config.is_backwards() {
                        false => target_d,
                        true => -target_d,
                    };
                                        
                    let ramps =  locked_position.get_ramps_as_mut();
                    ramps[Order::A_INDEX].set_target(target_a);
                    ramps[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(target_d);

                    let mut done = true;
                    for ramp in ramps.iter() {
                        if !ramp.is_done() {
                            done = false;
                            break;
                        }
                    }
                    if done {
                        break;
                    }
                    drop(locked_position);
                    
                    Timer::after(Duration::from_millis(75)).await;
                }
                
                Ok(order_index + 1)
            }

            Order::MoveUntil { callback } => {
                loop {
                    let mut locked_position = position.lock().await;

                    let initial_position = locked_position.get_coordinates();
                    let initial_d = initial_position.get_raw_linear_coordonate()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT];

                    let ramp_d = locked_position.get_ramps_as_mut();
                    

                    if !callback() {
                        ramp_d[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d);
                        break;
                    }
                    else {
                        match config.is_backwards() {
                            true => ramp_d[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d - 1_000.0),
                            false => ramp_d[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d + 1_000.0),
                        }
                    }

                    Timer::after(Duration::from_millis(75)).await;
                }
                Ok(order_index + 1)
            }

            Order::Label { label: _ } | Order::GotoLabel { label: _ } | Order::GotoRelStepOrLabel { callback: _} | Order::GotoRel { rel_step: _} => {
                //Should have been handled before calling this function
                Err(TrajectorError::InvalidOrder)
            }
        }
    }

    fn find_closest_angle(current_angle: f32, angle: f32) -> f32 {
        //get modulo multiplier
        let abase = floorf(current_angle / TAU);

        //set set point in valid range (-π > π)
        let mut normalized_angle = angle;
        while normalized_angle > PI {
            normalized_angle -= TAU;
        }
        while normalized_angle < -PI {
            normalized_angle += TAU;
        }

        //find best target
        let t1 = abase + normalized_angle;
        let t2 = abase + normalized_angle + TAU;
        let t3 = abase + normalized_angle - TAU;
        let t4 = abase + normalized_angle - 2.0 * TAU;

        let dt1 = (t1 - current_angle).abs();
        let dt2 = (t2 - current_angle).abs();
        let dt3 = (t3 - current_angle).abs();
        let dt4 = (t4 - current_angle).abs();

        let mut target = t1;
        let mut dtarget = dt1;

        if dt2 < dtarget{
            target = t2;
            dtarget = dt2;
        }
        if dt3 < dtarget {
            target = t3;
            dtarget = dt3;
        }
        if dt4 < dtarget {
            target = t4;
        }

        target
    }
}