use core::f32::consts::{PI, TAU};
use std::time::Instant; // TAU = 2 * PI

use either::Either;
use libm::{atan2f, floorf, sqrtf};

use crate::position::{self, robot_coordinate::RobotCoordinate, PositionMutex};

use super::{OrderConfig, TrajectorError, Trajectory};

pub struct OrderState<const N: usize> {
    pub initial_coordinate: RobotCoordinate<N>,
    pub state_index: usize,
    pub first_run_call: bool,

    pub forward: bool,
    pub pause_since: Option<Instant>,
}

impl<const N: usize> OrderState<N> {
    pub fn new(coordinate: &RobotCoordinate<N>) -> Self {
        Self { 
            initial_coordinate: coordinate.clone(), 
            state_index: 0,
            first_run_call: true,
            forward: true,
            pause_since: None,
        }
    }
}

#[derive(Debug)]
pub enum Order<const N: usize, Event> {
    GotoD { d_mm: f32 },
    GotoA { a_rad: f32 },
    GotoXY { x_mm: f32, y_mm: f32 },
    
    SetPosition { x_mm: Option<f32>, y_mm: Option<f32>, a_rad: Option<f32>},
    CustomOrder { callback: fn(order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> },


    Label { label: &'static str },
    GotoLabel { label: &'static str },
    GotoRel { rel_step: usize },
    GotoRelStepOrLabel { callback: fn() -> Either<usize, &'static str> },
}

impl<const N: usize, Event> Order<N, Event> {
    pub const A_INDEX: usize = 0;
    pub const D_INDEX_IN_NON_HOLONOMIC_ROBOT: usize = 1;

    pub fn init(&self, position: &PositionMutex<N>) -> OrderState<N> {
        let locked_position = position.lock().unwrap();
        OrderState::new(locked_position.get_coordinates())
    }

    pub fn execute(&self, order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &mut Trajectory<N, Event>) -> Result<usize, TrajectorError> {
        let position = trajectory.get_position().clone();

        let r = match self {
            Order::GotoD { d_mm } => {
                match N {
                    2 => {                        
                        if let Some(stop_distance_mm) = config.opponent_stop_distance_mm {
                            let opponent_distance_mm = trajectory.get_opponent_distance(!config.is_backwards());

                            if opponent_distance_mm < stop_distance_mm && !config.no_detection {                             
                                let mut locked_position = position.lock().unwrap();
                                let current_d = locked_position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                                let target_d = if config.is_backwards() {
                                    current_d - d_mm
                                } else {
                                    current_d + d_mm
                                };
                                let can_move = if config.is_backwards() {
                                    opponent_distance_mm > (current_d - target_d + config.opponent_resume_margin_distance_mm).abs()
                                } else {
                                    opponent_distance_mm > (target_d - config.opponent_resume_margin_distance_mm - current_d).abs()
                                };

                                if !can_move {
                                    if state.pause_since.is_none() {                                
                                        state.pause_since = Some(Instant::now()); 
                                        trajectory.set_opponent_detected(true);
                                        locked_position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(current_d, true);                            
                                    }
                                    return Ok(order_index);
                                }
                            }
                        }

                        if state.first_run_call || state.pause_since.is_some() {
                            state.pause_since = None;
                            trajectory.set_opponent_detected(true);

                            let initial_d = state.initial_coordinate.get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];

                            if config.is_backwards() {
                                let mut locked_position = position.lock().unwrap();
                                locked_position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d - d_mm, false);
                            }
                            else {
                                let mut locked_position = position.lock().unwrap();
                                locked_position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d + d_mm, false);
                            }
                        }
                        

                        let locked_position = position.lock().unwrap();
                        let d_ramp =  &locked_position.get_ramps()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];

                        if d_ramp.is_done() {
                            Ok(order_index + 1)
                        }
                        else {
                            Ok(order_index)
                        }
                    }

                    _ => {
                        Err(TrajectorError::InvalidOrder)
                    }
                }
            }
            Order::GotoA { a_rad } => {
                if state.first_run_call {
                    let mut locked_position = position.lock().unwrap();
                    let initial_position = locked_position.get_coordinates();
                    let initial_a = initial_position.get_a_rad();

                    if config.is_backwards() {
                        locked_position.get_ramps_as_mut()[Order::<N, Event>::A_INDEX].set_target(Order::<N, Event>::find_closest_angle(initial_a, *a_rad + PI), false);
                    }
                    else {
                        locked_position.get_ramps_as_mut()[Order::<N, Event>::A_INDEX].set_target(Order::<N, Event>::find_closest_angle(initial_a, *a_rad), false);
                    }
                }

                let locked_position = position.lock().unwrap();
                let a_ramp =  &locked_position.get_ramps()[Order::<N, Event>::A_INDEX];

                if a_ramp.is_done() {
                    Ok(order_index + 1)
                }
                else {
                    Ok(order_index)
                }                
            }

            Order::GotoXY { x_mm, y_mm } => {
                if N != 2 {
                    return Err(TrajectorError::InvalidOrder);
                }

                let mut locked_position = position.lock().unwrap();
                let initial_position = locked_position.get_coordinates();
                let delta_x = x_mm - initial_position.get_x_mm();
                let delta_y = y_mm - initial_position.get_y_mm();
                let current_a = initial_position.get_a_rad();
                let current_d = initial_position.get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                let ramps =  locked_position.get_ramps_as_mut();
                match state.state_index {
                    0 => {
                        //orient to target without moving
                        
                        
                        let target_a = atan2f(delta_y, delta_x);
                        let target_a = match config.is_backwards() {
                            false => Order::<N, Event>::find_closest_angle(current_a, target_a),
                            true => Order::<N, Event>::find_closest_angle(current_a, target_a + PI),
                        };
                        ramps[Order::<N, Event>::A_INDEX].set_target(target_a, false);
                        state.state_index = 1;
                        Ok(order_index)
                    }

                    1 => {
                        if ramps[Order::<N, Event>::A_INDEX].is_done() {
                            state.state_index = 2;
                        }
                        Ok(order_index)
                    }

                    2 | 3 => {
                        //move to target
                        let target_d = sqrtf(delta_x * delta_x + delta_y * delta_y);

                        if let Some(stop_distance_mm) = config.opponent_stop_distance_mm {
                            let opponent_distance_mm = trajectory.get_opponent_distance(!config.is_backwards());

                            if opponent_distance_mm < stop_distance_mm {                             
                                let target_d = if config.is_backwards() {
                                    current_d - target_d
                                } else {
                                    current_d + target_d
                                };
                                let can_move = if config.is_backwards() {
                                    opponent_distance_mm > (current_d - target_d + config.opponent_resume_margin_distance_mm).abs()
                                } else {
                                    opponent_distance_mm > (target_d - config.opponent_resume_margin_distance_mm - current_d).abs()
                                };

                                if !can_move {
                                    if state.pause_since.is_none() {                                
                                        state.pause_since = Some(Instant::now()); 
                                        trajectory.set_opponent_detected(true);
                                        ramps[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(current_d, true);
                                        ramps[Order::<N, Event>::A_INDEX].set_target(current_a, true);
                                    }
                                    state.state_index = 2; //stay in state 2 to force ramp to be updated
                                    return Ok(order_index);
                                }
                            }
                        }
                        
                        trajectory.set_opponent_detected(false);

                        if (target_d < 50.0) && (state.state_index == 3) {
                            //if less than 50mm, just let the ramp finish
                            if ramps[Order::<N, Event>::A_INDEX].is_done() && ramps [Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].is_done() {
                                return Ok(order_index + 1);
                            }
                            else {
                                return Ok(order_index);
                            }
                        }
                        else {
                            let target_d = match config.is_backwards() {
                                false => current_d + target_d,
                                true => current_d - target_d,
                            };
                            let target_a = atan2f(delta_y, delta_x);
                            let target_a = match config.is_backwards() {
                                false => Order::<N, Event>::find_closest_angle(current_a, target_a),
                                true => Order::<N, Event>::find_closest_angle(current_a, target_a + PI),
                            };

                            ramps[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(target_d, false);
                            ramps[Order::<N, Event>::A_INDEX].set_target(target_a, false);
                        }

                        state.state_index = 3;
                        Ok(order_index)
                    }

                    _ => {
                        log::error!("Should never happen");
                        Err(TrajectorError::InvalidOrder)
                    }
                }
            }

            Order::SetPosition { x_mm, y_mm, a_rad} => {
                let mut locked_position = position.lock().unwrap();

                locked_position.set_coordinates(*x_mm, *y_mm, *a_rad);

                if let Some(a_rad) = *a_rad {
                    let ramps = locked_position.get_ramps_as_mut();
                    ramps[Order::<N, Event>::A_INDEX].set_target(a_rad, true);
                }

                Ok(order_index + 1)
            }

            Order::CustomOrder { callback } => {
                callback(order_index, config, state, custom_events, trajectory)
            }

            Order::Label { label: _ } | Order::GotoLabel { label: _ } | Order::GotoRelStepOrLabel { callback: _} | Order::GotoRel { rel_step: _} => {
                //Should have been handled before calling this function
                Err(TrajectorError::InvalidOrder)
            }
        };
        
        state.first_run_call = false;
        return r;
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
        let t1 = abase * TAU + normalized_angle;
        let t2 = abase * TAU + normalized_angle + TAU;
        let t3 = abase * TAU + normalized_angle - TAU;
        let t4 = abase * TAU + normalized_angle - 2.0 * TAU;

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