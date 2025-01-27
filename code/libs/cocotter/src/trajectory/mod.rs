use core::mem::swap;

use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::RawMutex;
use order::Order;

use crate::{position::PositionMutex, ramp};

pub mod order;

#[derive(Debug, Clone, Copy)]
pub enum TrajectorError {
    InvalidOrder,
    Interruted,
}

pub struct Trajectory<M: RawMutex, const N: usize> {
    position: PositionMutex<M, N>,
}

#[derive(Debug)]
pub struct OrderConfig<const N: usize> {
    ramp_configuration: [Option<ramp::RampConfiguration>; N],

    backwards: bool,
}

impl<const N: usize> OrderConfig<N> {
    pub fn new() -> OrderConfig<N> {
        OrderConfig {
            ramp_configuration: [None; N],
            backwards: false,
        }
    }

    pub async fn apply<M: RawMutex>(&mut self, position: &PositionMutex<M, N>) {
        let mut position = position.lock().await;
        let ramps = position.get_ramps_as_mut();
        for i in 0..N {
            if let Some(config) = self.ramp_configuration[i].as_mut() {
                swap(ramps[i].get_conf_as_mut(), config);
            }
        }
    }

    pub async fn revert<M: RawMutex>(&mut self, position: &PositionMutex<M, N>) {
        //we can just apply the configuration again to revert it
        //this is because the configuration is swapped
        //so the revert will just swap it back
        self.apply(position).await;
    }

    pub fn is_backwards(&self) -> bool {
        self.backwards
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
}

impl<const N: usize>  TrajectoryOrderList<N> {
    pub fn new() -> TrajectoryOrderList<N> {
        TrajectoryOrderList {
            orders: Vec::new(),
            current_order_index: 0,
        }
    }

    pub fn add_order(mut self, order: Order) -> Self {
        self.orders.push((order, OrderConfig::new()));

        self
    }

    async fn execute<M: RawMutex>(mut self, position: &PositionMutex<M, N>) -> Result<(), (TrajectorError, TrajectoryOrderList<N>)> {
        loop {
            if self.current_order_index >= self.orders.len() {
                return Ok(())
            }

            let (order, config) = &mut self.orders[self.current_order_index];

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
