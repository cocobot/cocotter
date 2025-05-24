use core::f32::consts::{PI, TAU};
use std::time::Instant; // TAU = 2 * PI

use either::Either;
use libm::{atan2f, floorf, sqrtf};

use crate::position::{robot_coordinate::RobotCoordinate, PositionMutex};

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

    pub fn execute(&self, order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> {
        let position = trajectory.get_position();

        let r = match self {
            Order::GotoD { d_mm } => {
                match N {
                    2 => {                        
                        if let Some(stop_distance_mm) = config.opponent_stop_distance_mm {
                            let opponent_distance_mm = trajectory.get_opponent_distance(!config.is_backwards());

                            if opponent_distance_mm < stop_distance_mm {                             
                                let mut locked_position = position.lock().unwrap();
                                let current_d = locked_position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                                let target_d = if config.is_backwards() {
                                    state.initial_coordinate.get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT] - d_mm
                                } else {
                                    state.initial_coordinate.get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT] + d_mm
                                };
                                let can_move = if config.is_backwards() {
                                    opponent_distance_mm > (current_d - target_d + config.opponent_resume_margin_distance_mm).abs()
                                } else {
                                    opponent_distance_mm > (target_d - config.opponent_resume_margin_distance_mm - current_d).abs()
                                };

                                if !can_move {
                                    if state.pause_since.is_none() {                                
                                        state.pause_since = Some(Instant::now()); 
                                        locked_position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(current_d, true);                            
                                    }
                                    return Ok(order_index);
                                }
                            }
                        }

                        if state.first_run_call || state.pause_since.is_some() {
                            state.pause_since = None;

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
/*
                let mut locked_position = position.lock().unwrap();
                let initial_position = locked_position.get_coordinates();

                let delta_x = x_mm - initial_position.get_x_mm();
                let delta_y = y_mm - initial_position.get_y_mm();

                //check if angle is close enough
                let target_a = atan2f(delta_y, delta_x);
                let initial_d = initial_position.get_raw_linear_coordonate()[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                let target_a = match config.is_backwards() {
                    false => Order::find_closest_angle(initial_position.get_a_rad(), target_a),
                    true => Order::find_closest_angle(initial_position.get_a_rad(), target_a + PI),
                };

                let angle_diff = (target_a - initial_position.get_a_rad()).abs();
                if angle_diff >= config.max_angle_diff_in_xy {
                    let ramps =  locked_position.get_ramps_as_mut();
                    ramps[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d);
                    ramps[Order::A_INDEX].set_target(target_a);
                    drop(locked_position);

                    loop {
                        let locked_position = position.lock().unwrap();
                        let a_ramp =  &locked_position.get_ramps()[Order::A_INDEX];
                        if a_ramp.is_done() {
                            break;
                        }
                        drop(locked_position);
                        
                        std::thread::sleep(Duration::from_millis(75));
                    }
                }
                else {
                    drop(locked_position);
                }

                //run to target
                let mut locked_position =   position.lock().unwrap();
                let initial_position = locked_position.get_coordinates();
                let delta_x = x_mm - initial_position.get_x_mm();
                let delta_y = y_mm - initial_position.get_y_mm();
                let target_d = sqrtf(delta_x * delta_x + delta_y * delta_y);
                let target_d = match config.is_backwards() {
                    false => target_d,
                    true => -target_d,
                };

                let ramps =  locked_position.get_ramps_as_mut();
                ramps[Order::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(initial_d + target_d);
                ramps[Order::A_INDEX].set_target(target_a);
                drop(locked_position);

                loop {
                    let mut locked_position =   position.lock().unwrap();
                    let ramps =  locked_position.get_ramps_as_mut();

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
                    
                    std::thread::sleep(Duration::from_millis(75));
                }
                */
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