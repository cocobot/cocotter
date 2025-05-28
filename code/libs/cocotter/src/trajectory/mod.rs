use std::{sync::mpsc::{self, Receiver}, time::Duration};

use order::Order;

use crate::position::PositionMutex;

pub mod order;

#[derive(Debug, Clone, Copy)]
pub enum TrajectorError {
    InvalidOrder,
    OrderInProgress,
    Interruted,
}

pub enum TrajectoryEvent<Event> {
    BackOpponentDetected(f32),
    CustomEvent(Event),
}

pub enum RampCfg {
    Linear,
    Angular,
}


#[derive(Debug, Clone)]
pub struct OrderConfig<const N: usize> {
    max_speed: [Option<f32>; N],
    acceleration: [Option<f32>; N],

    backwards: Option<bool>,

    opponent_stop_distance_mm: Option<f32>,
    opponent_resume_margin_distance_mm: f32,
}

impl<const N: usize> OrderConfig<N> {
    pub fn new() -> OrderConfig<N> {
        OrderConfig {
            max_speed: [Some(1.0); N],
            acceleration: [Some(1.0); N],
            backwards: Some(false),
            opponent_stop_distance_mm: Some(150.0),
            opponent_resume_margin_distance_mm: 50.0,
        }
    }

    pub fn apply(&mut self, position: &PositionMutex<N>) {
        let mut position = position.lock().unwrap();
        let ramps = position.get_ramps_as_mut();
        for i in 0..N {
            if let Some(max_speed) = self.max_speed[i] {
                ramps[i].set_max_speed_factor(max_speed);
            }
            if let Some(acceleration) = self.acceleration[i] {
                ramps[i].set_acceleration_factor(acceleration);
            }
        }
    }

    pub fn revert(&mut self, position: &PositionMutex<N>) {
        //reset max speed an acceleration
        let mut position = position.lock().unwrap();
        let ramps = position.get_ramps_as_mut();
        for i in 0..N {
            ramps[i].set_max_speed_factor(1.0);
            ramps[i].set_acceleration_factor(1.0);
        }
    }

    pub fn is_backwards(&self) -> bool {
        if let Some(backwards) = self.backwards {
            backwards
        }
        else {
            false
        }
    }
    
}

pub struct Trajectory<const N: usize, Event> {
    position: PositionMutex<N>,
    receiver: Receiver<TrajectoryEvent<Event>>,

    back_opponent_detected: f32,
    front_opponent_detected: f32,
}


impl<const N: usize, Event> Trajectory<N, Event> {
    pub fn new(position: PositionMutex<N>) -> (Trajectory<N, Event>, mpsc::Sender<TrajectoryEvent<Event>>) {
        let (sender, receiver) = mpsc::channel();
        (
            Trajectory {
                position,
                receiver,
                back_opponent_detected: f32::MAX,
                front_opponent_detected: f32::MAX,
            },
            sender,
        )
    }

    pub fn purge(&mut self) {
        // Clear the receiver queue to avoid processing old events
        self.parse_event(None);
    }

    fn get_opponent_distance(&self, forward: bool) -> f32 {
        if forward {
            return self.front_opponent_detected;
        }
        else {
            return self.back_opponent_detected;
        }
    }

    fn parse_event(&mut self, wait: Option<Duration>) -> Vec<Event>{
        let mut events = Vec::new(); 
        loop {
            match self.receiver.recv_timeout(wait.unwrap_or(Duration::from_millis(0))) {
                Ok(event) => {
                    match event {
                        TrajectoryEvent::BackOpponentDetected (distance_mm)=> {
                            self.back_opponent_detected = distance_mm;
                        }
                        TrajectoryEvent::CustomEvent( evt ) => {
                            events.push(evt);
                        }
                    }
                }
                _ => {
                    break;
                }
            }
        }

        return events;
    }

    pub fn execute(&mut self, list: TrajectoryOrderList<N, Event>) -> Result<(), (TrajectorError, TrajectoryOrderList<N, Event>)> {
        list.execute(self)
    }

    pub fn get_position(&self) -> &PositionMutex<N> {
        &self.position
    }
}

#[derive(Debug)]
pub struct TrajectoryOrderList<const N: usize, Event> {
    orders: Vec<(Order<N, Event>, OrderConfig<N>)>,
    current_order_index: usize,
    default_config: OrderConfig<N>,
}

impl<const N: usize, Event>  TrajectoryOrderList<N, Event> {
    pub fn new() -> TrajectoryOrderList<N, Event> {
        TrajectoryOrderList {
            orders: Vec::new(),
            current_order_index: 0,
            default_config: OrderConfig::new(),
        }
    }

    pub fn add_order(mut self, order: Order<N, Event>) -> Self {
        self.orders.push((order, self.default_config.clone()));

        self
    }

    pub fn set_backwards(mut self, backwards: bool) -> Self {
        if self.orders.is_empty() {
            self.default_config.backwards = Some(backwards);
        }
        else {
            self.orders.last_mut().unwrap().1.backwards = Some(backwards);
        }
        
        self
    }

    pub fn set_max_speed(mut self, ramp: RampCfg, max_speed: f32) -> Self {
        let order_config = if self.orders.is_empty() {
            &mut self.default_config
        }
        else {
            &mut self.orders.last_mut().unwrap().1
        };

        match ramp {
            RampCfg::Linear => {
                for i in 1..N {
                    order_config.max_speed[i] = Some(max_speed);
                }
            }
            RampCfg::Angular => {
                order_config.max_speed[0] = Some(max_speed);
            }
        }

        self
    }

    pub fn set_acceleration(mut self, ramp: RampCfg, acceleration: f32) -> Self {
        let order_config = if self.orders.is_empty() {
            &mut self.default_config
        }
        else {
            &mut self.orders.last_mut().unwrap().1
        };

        match ramp {
            RampCfg::Linear => {
                for i in 1..N {
                    order_config.acceleration[i] = Some(acceleration);
                }
            }
            RampCfg::Angular => {
                order_config.acceleration[0] = Some(acceleration);
            }
        }

        self
    }

    fn execute(mut self, trajectory: &mut Trajectory<N, Event>) -> Result<(), (TrajectorError, TrajectoryOrderList<N, Event>)> {
        loop {
            if self.current_order_index >= self.orders.len() {
                return Ok(())
            }

            let (order, config) = &mut self.orders[self.current_order_index];
            config.apply(trajectory.get_position());

            let mut order_state = order.init(trajectory.get_position());

            trajectory.parse_event(None);

            let mut custom_events = Vec::new();
            loop {
                let result = order.execute(self.current_order_index, config, &mut order_state, &mut custom_events, trajectory);
                custom_events.clear();

                match result {
                    Ok(index) => {
                        if self.current_order_index != index {
                            self.current_order_index = index;
                            break;
                        }
                        else {
                            custom_events = trajectory.parse_event(Some(Duration::from_millis(75)));
                        }   
                    }                 
                    Err(e) => {
                        config.revert(trajectory.get_position());
                        return Err((e, self))
                    }            
                }

            }

            config.revert(trajectory.get_position());            
        }
    }
}
