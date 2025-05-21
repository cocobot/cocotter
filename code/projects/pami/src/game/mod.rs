use std::time::Duration;

use cocotter::trajectory::{self, order::Order, Trajectory, TrajectoryEvent, TrajectoryOrderList};

use crate::{asserv::AsservMutexProtected, config::DETECTION_DISTANCE_MM, events::{Event, EventSystem}};

pub struct Game {
    trajectory: Trajectory<2, Event>,
}

impl Game {
    pub fn new(asserv: AsservMutexProtected, event: &EventSystem) {
        let (trajectory, sender) = Trajectory::new(asserv.lock().unwrap().get_position());

        let mut instance = Self {
            trajectory: trajectory,
        };

        std::thread::Builder::new()
            .stack_size(18192)
            .name("Game".to_string())
            .spawn(move || {
                instance.run();
            })
            .unwrap();

        let mut opponent_detected = false;
        event.register_receiver_callback(Some(EventSystem::no_debug_filter), move |event| {
            match event {
                Event::BackDistance { distance } => {
                    let min = (*distance.iter().min().unwrap()) as f32;
                    if (min <= DETECTION_DISTANCE_MM) && !opponent_detected {
                        opponent_detected = true;
                        sender.send(TrajectoryEvent::OpponentDetected).unwrap();
                    }
                    else if (min > DETECTION_DISTANCE_MM) && opponent_detected {
                        opponent_detected = false;
                        sender.send(TrajectoryEvent::OpponentLost).unwrap();
                    }
                }
                _ => {}
            }
            sender.send(TrajectoryEvent::CustomEvent(event.clone())).unwrap();
        });
    }

    fn run(&mut self) {

        std::thread::sleep(Duration::from_millis(1000));

        loop {            

        loop {
            let orders = TrajectoryOrderList::new()
                .set_backwards(true)
                .add_order(Order::GotoD { d_mm: 200.0_f32 })
                .add_order(Order::GotoA { a_rad: 90.0_f32.to_radians() })
                .add_order(Order::GotoD { d_mm: 200.0_f32 })
                .add_order(Order::GotoD { d_mm: -200.0_f32 })
                .add_order(Order::GotoA { a_rad: 180.0_f32.to_radians() })
                .add_order(Order::GotoD { d_mm: 200.0_f32 })
                //.add_order(Order::GotoA { a_rad: 90.0_f32.to_radians() })
                .add_order(Order::GotoA { a_rad: 0.0_f32.to_radians() });
            self.trajectory
                .execute(orders)
                .unwrap();    

    
                std::thread::sleep(Duration::from_millis(1000));
        }

            let orders = TrajectoryOrderList::new()
                .add_order(Order::GotoA { a_rad: 0.0_f32.to_radians() });
               // .add_order(Order::GotoD { d_mm: -200.0_f32 });
            self.trajectory
                .execute(orders)
                .unwrap();            

            std::thread::sleep(Duration::from_millis(2500));
        }
    }
}