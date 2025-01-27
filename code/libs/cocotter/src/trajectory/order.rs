use core::f32::consts::{PI, TAU}; // 2 * PI

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Timer};

use crate::position::PositionMutex;

use super::{OrderConfig, TrajectorError};

#[derive(Debug)]
pub enum Order {
    GotoD { d_mm: f32 },
    GotoA { a_rad: f32 },
}

impl Order {
    pub async fn execute<M: RawMutex, const N: usize>(&self, order_index: usize, config: &OrderConfig<N>, position: &PositionMutex<M, N>) -> Result<usize, TrajectorError> {
        match self {
            Order::GotoD { d_mm } => {
                match N {
                    2 => {
                        const D_INDEX_IN_NON_HOLONOMIC_ROBOT: usize = 1;

                        let mut locked_position = position.lock().await;
                        let initial_position = locked_position.get_coordinates();
                        let initial_d = initial_position.get_raw_linear_coordonate()[D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                        if config.is_backwards() {
                            locked_position.get_ramps_as_mut()[D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d - d_mm);
                        }
                        else {
                            locked_position.get_ramps_as_mut()[D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d + d_mm);
                        }
                        drop(locked_position);
        
                        loop {
                            let locked_position = position.lock().await;
                            let d_ramp =  &locked_position.get_ramps()[D_INDEX_IN_NON_HOLONOMIC_ROBOT];
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
                const A_INDEX: usize = 0;
                        
                let mut locked_position = position.lock().await;
                let initial_position = locked_position.get_coordinates();
                let initial_a = initial_position.get_a_rad();
                if config.is_backwards() {
                    locked_position.get_ramps_as_mut()[A_INDEX].set_target(Order::find_closest_angle(initial_a, *a_rad));
                }
                else {
                    locked_position.get_ramps_as_mut()[A_INDEX].set_target(Order::find_closest_angle(initial_a, *a_rad + PI));
                }
                drop(locked_position);

                loop {
                    let locked_position = position.lock().await;
                    let a_ramp =  &locked_position.get_ramps()[A_INDEX];
                    if a_ramp.is_done() {
                        break;
                    }
                    drop(locked_position);
                    
                    Timer::after(Duration::from_millis(75)).await;
                }
                
                Ok(order_index + 1)
            }
        }
    }

    fn find_closest_angle(current_angle: f32, angle: f32) -> f32 {
        //get modulo multiplier (floor division without std)
        let abase = ((current_angle / TAU) as i32 as f32) * TAU;

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