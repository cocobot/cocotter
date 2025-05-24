use std::time::Duration;

use cocotter::trajectory::{order::{Order, OrderState}, OrderConfig, TrajectorError, Trajectory, TrajectoryEvent, TrajectoryOrderList};
use linreg::linear_regression;

use crate::{asserv::AsservMutexProtected, events::{Event, EventSystem}, pwm::{OverrideState, PWMEvent}};

pub struct Game {
    trajectory: Trajectory<2, Event>,
    event: EventSystem,
}

impl Game {
    pub fn new(asserv: AsservMutexProtected, event: &EventSystem) {
        let (trajectory, sender) = Trajectory::new(asserv.lock().unwrap().get_position());

        let mut instance = Self {
            trajectory: trajectory,
            event: event.clone(),
        };

        std::thread::Builder::new()
            .stack_size(18192)
            .name("Game".to_string())
            .spawn(move || {
                instance.run();
            })
            .unwrap();

        event.register_receiver_callback(Some(EventSystem::no_debug_filter), move |event| {
            match event {
                Event::BackDistance { distance } => {
                    let min = (*distance.iter().min().unwrap()) as f32;
                    //TODO: remove comment: desactivate the back opponent detection for now....
                    //sender.send(TrajectoryEvent::BackOpponentDetected(min)).unwrap();

                    sender.send(TrajectoryEvent::CustomEvent(event.clone())).unwrap();
                }
                Event::Line { activated } => {
                    sender.send(TrajectoryEvent::CustomEvent(event.clone())).unwrap();
                }
                _ => {}
            }
        });
    }

    fn strat_superstar(&mut self) {

        self.event.send_event(Event::OverridePwm {
            pwm_event: PWMEvent::LedBottom([0.0, 0.0, 1.0]),
            override_state: OverrideState::Override,
        });
         
        let orders = TrajectoryOrderList::new()
            .set_backwards(true)
            .add_order(Order::GotoD { d_mm: 1000.0 });


        self.trajectory
            .execute(orders)
            .unwrap();


        let orders = TrajectoryOrderList::new()
            .set_backwards(false)
            .add_order(Order::GotoA { a_rad: -90.0_f32.to_radians() })
            .set_max_speed(cocotter::trajectory::RampCfg::Linear, 0.2)
            .add_order(Order::CustomOrder { callback: move_until_void })
            ;

        self.trajectory
            .execute(orders)
            .unwrap();

        let colors = [
            [1.0, 0.0, 0.0], // Red
            [0.0, 1.0, 0.0], // Green
            [0.0, 0.0, 1.0], // Blue
            [1.0, 1.0, 0.0], // Yellow
            [1.0, 0.5, 0.5], // Light Red
        ];
        loop {
            for color in colors.iter() {
                self.event.send_event(Event::OverridePwm {
                    pwm_event: PWMEvent::LedBottom(*color),
                    override_state: OverrideState::Override,
                });

                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    fn run(&mut self) {

        std::thread::sleep(Duration::from_millis(1000));

        let mut position = self.trajectory.get_position().lock().unwrap();
        position.set_coordinates(Some(2900.0), Some(1900.0), Some(180.0_f32.to_radians()));
        drop(position);

        self.strat_superstar();
        loop {          
            loop {
            let orders = TrajectoryOrderList::new()
                .set_backwards(false)
                .set_max_speed(cocotter::trajectory::RampCfg::Linear, 0.2)
                .add_order(Order::CustomOrder { callback: move_until_void })
                ;
            self.trajectory
                .execute(orders)
                .unwrap();    

    
                std::thread::sleep(Duration::from_millis(2500));
        }
        }
    }
}

fn move_until_void<const N: usize>(order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> {
    
    for event in custom_events.iter() {
        if let Event::Line { activated } = event{
            //stop if all is true
            if activated.iter().all(|&x| x) {
                //full stop
                let mut position = trajectory.get_position().lock().unwrap();
                let current_d = position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
                position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(current_d, true);
                return Ok(order_index + 1);
            }

            state.state_index = 1; //set state to 1 to start the order
        }
    }

    if state.state_index == 1 {
        //first line data received, we can start the order
        let mut position = trajectory.get_position().lock().unwrap();
        let current_d = position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
        let target_d = if config.is_backwards() {
            current_d - 100.0 //move backwards
        } else {
            current_d + 100.0 //move forwards
        };
        
        position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(target_d, false);
    }

    Ok(order_index)
}

fn align_with_wall<const N: usize>(order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> {
    if !config.is_backwards() {
        //not supported as front opponent is not detected yet
        return Err(TrajectorError::InvalidOrder);
    }

    match state.state_index {
        0 => {
            let mut distance_detected = [0_f32; 8];
            let mut distance_detected_count = [0; 8];

            for event in custom_events.iter() {
                if let Event::BackDistance { distance } = event {
                    for (i, &d) in distance.iter().enumerate() {
                        let d = d as f32;
                        if d < 100.0 {
                            distance_detected[i] += d;
                            distance_detected_count[i] += 1;
                        }
                    }
                }
            }

            log::info!("Distance detected: {:?} {:?}", distance_detected, distance_detected_count);
            let mut x = Vec::new();
            let mut y = Vec::new();
            for i in 0..8 {
                if distance_detected_count[i] > 0 {
                    x.push(-3.5 + 0.5 * i as f32);
                    y.push(distance_detected[i] / distance_detected_count[i] as f32);
                }
            }
            //if let Some(distance) = distance_detected {
            //    let x = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5];
            //    let y = distance.iter().map(|&d| d as f32).collect::<Vec<_>>();
            //    if let Ok((slope, _intercept)) = linear_regression::<f32, f32, f32>(&x, &y) {
            //        let angle = slope.atan();
//
            //        log::info!("Slope: {} | angle: {}", slope, angle.to_degrees());
            //    }
            //}
        }

        _ => {
            return Err(TrajectorError::InvalidOrder);
        }
    }

    Ok(order_index)
}