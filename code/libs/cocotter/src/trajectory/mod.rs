use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::RawMutex;
use order::Order;
use log::info;

use crate::position::PositionMutex;

pub mod order;

#[derive(Debug, Clone, Copy)]
pub enum TrajectorError {
    InvalidOrder,
    Interruted,
}

pub enum RampCfg {
    Linear,
    Angular,
}

pub struct Trajectory<M: RawMutex, const N: usize> {
    position: PositionMutex<M, N>,
}

#[derive(Debug)]
pub struct OrderConfig<const N: usize> {
    max_speed: [Option<f32>; N],
    acceleration: [Option<f32>; N],

    backwards: Option<bool>,
    max_angle_diff_in_xy: f32,
}

impl<const N: usize> OrderConfig<N> {
    pub fn new() -> OrderConfig<N> {
        OrderConfig {
            max_speed: [Some(1.0); N],
            acceleration: [Some(1.0); N],
            backwards: Some(false),
            max_angle_diff_in_xy: 30_f32.to_radians(),
        }
    }

    pub async fn apply<M: RawMutex>(&mut self, position: &PositionMutex<M, N>) {
        let mut position = position.lock().await;
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

    pub async fn revert<M: RawMutex>(&mut self, position: &PositionMutex<M, N>) {
        //reset max speed an acceleration
        let mut position = position.lock().await;
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

impl<M: RawMutex, const N: usize> Trajectory<M, N> {
    pub fn new(position: PositionMutex<M, N>) -> Trajectory<M, N> {
        Trajectory {
            position,
        }
    }

    pub async fn execute(&self, list: TrajectoryOrderList<N>) -> Result<(), (TrajectorError, TrajectoryOrderList<N>)> {
        list.execute(&self.position).await
    }
}

#[derive(Debug)]
pub struct TrajectoryOrderList<const N: usize> {
    orders: Vec<(Order, OrderConfig<N>)>,
    current_order_index: usize,
    default_config: OrderConfig<N>,
}

impl<const N: usize>  TrajectoryOrderList<N> {
    pub fn new() -> TrajectoryOrderList<N> {
        TrajectoryOrderList {
            orders: Vec::new(),
            current_order_index: 0,
            default_config: OrderConfig::new(),
        }
    }

    pub fn add_order(mut self, order: Order) -> Self {
        self.orders.push((order, OrderConfig::new()));

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

    async fn execute<M: RawMutex>(mut self, position: &PositionMutex<M, N>) -> Result<(), (TrajectorError, TrajectoryOrderList<N>)> {
        loop {
            if self.current_order_index >= self.orders.len() {
                return Ok(())
            }

            let (order, config) = &mut self.orders[self.current_order_index];
            log::info!("execute order");
            config.apply(position).await;
            let result = order.execute(self.current_order_index, config, position).await;
            config.revert(position).await;

            match result {
                Ok(index) => {
                    self.current_order_index = index;
                }
                Err(e) => {
                    return Err((e, self))
                }
            }
        }
    }
}
