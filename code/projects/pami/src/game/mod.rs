use std::{sync::{Arc, Mutex}, thread, time::{Duration, Instant}};

use board_pami_2023::Starter;
use cocotter::trajectory::{order::{Order, OrderState}, OrderConfig, TrajectorError, Trajectory, TrajectoryEvent, TrajectoryOrderList};
use crate::{asserv::AsservMutexProtected, config::{GAME_TIME_SECONDS, PAMI_START_TIME_SECONDS}, events::{Event, EventSystem}, pwm::{OverrideState, PWMEvent}};

#[derive(Debug, Clone, Copy)]
pub enum GameStrategy {
    Superstar,
    NearPit,
    MidPit,
    FarPit,
}

pub struct GameConfiguration {
    pub starter: Starter,

    pub test_mode: bool,
    pub x_negative_color: bool,
    pub strategy: GameStrategy,
}

pub struct FunnyAction {
    start_time: Option<Instant>,
    test_mode : bool,
    event: EventSystem,
}

impl FunnyAction {
    pub fn new(event: &EventSystem) -> Arc<Mutex<Self>> {
        let instance = Arc::new(Mutex::new(Self {
            start_time: None,
            test_mode: false,
            event: event.clone(),
        }));

        let cloned_instance = instance.clone();
        event.register_receiver_callback(Some(FunnyAction::event_filter), move |e| {
            match e {
                Event::GameStarted { timestamp, test_mode } => {
                    let mut instance = cloned_instance.lock().unwrap();
                    instance.start_time = Some(*timestamp);
                    instance.test_mode = *test_mode;
                }
                _ => {}
            }
        });
        
        let cloned_instance = instance.clone();
        std::thread::Builder::new()
            .stack_size(8192)
            .name("EndGame".to_string())
            .spawn(move || {
                Self::run(cloned_instance);
            })
            .unwrap();

        instance
    }

    fn event_filter(e: &Event) -> bool {
        match e {
            Event::GameStarted{ .. } => true,
            _ => false,
        }
    }

    fn run(instance: Arc<Mutex<Self>>) {
        loop {
            thread::sleep(Duration::from_millis(250));

            let locked_instance = instance.lock().unwrap();
            if locked_instance.test_mode {
                if let Some(start_time) = locked_instance.start_time {
                    if start_time.elapsed().as_secs() >= GAME_TIME_SECONDS - PAMI_START_TIME_SECONDS {
                        break;
                    }
                }
            }
            else {
                if let Some(start_time) = locked_instance.start_time {
                    if start_time.elapsed().as_secs() >= GAME_TIME_SECONDS {
                        break;
                    }
                }
            }
        }

        loop {
            instance.lock().unwrap().event.send_event(Event::Pwm { pwm_event: PWMEvent::Vaccum(1.0)});
            thread::sleep(Duration::from_millis(1000));
            instance.lock().unwrap().event.send_event(Event::Pwm { pwm_event: PWMEvent::Vaccum(0.0)});
            thread::sleep(Duration::from_millis(500));
        }
    }
}

pub struct Game {
    config: GameConfiguration,

    funny_action: Arc<Mutex<FunnyAction>>,

    start_time: Option<Instant>,
    trajectory: Trajectory<2, Event>,
    event: EventSystem,
}

impl Game {
    pub fn new(config: GameConfiguration, asserv: AsservMutexProtected, event: &EventSystem) {
        let event_clone = event.clone();
        let (trajectory, sender) = Trajectory::new(
            
            asserv.lock().unwrap().get_position(), 
            Box::new(move |opponent_detected| {
                event_clone.send_event(Event::TrajectoryStatus { opponent_detected: opponent_detected });
            })
        );

        let mut instance = Self {
            config,
            trajectory,
            start_time: None,
            event: event.clone(),
            funny_action: FunnyAction::new(event),
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
                    sender.send(TrajectoryEvent::BackOpponentDetected(min)).unwrap();

                    sender.send(TrajectoryEvent::CustomEvent(event.clone())).unwrap();
                }
                Event::Line { activated: _ } => {
                    sender.send(TrajectoryEvent::CustomEvent(event.clone())).unwrap();
                }
                _ => {}
            }
        });
    }

    fn wait_for_start(&mut self) {
        // Wait for the start signal to be low
        loop {
            if self.config.starter.is_low() {
                break;
            }
            else {
                std::thread::sleep(Duration::from_millis(50));
            }

            self.trajectory.purge();
        }

        //set the bottom led to the color of the team
        let team_color = if self.config.x_negative_color {
            [1.0, 1.0, 0.0] // yellow
        } else {
            [0.0, 0.0, 1.0] // blue
        };
        self.event.send_event(Event::OverridePwm {
            pwm_event: PWMEvent::LedBottom(team_color),
            override_state: OverrideState::Override,
        });

        //wait a little bit to avoid bouncing
        std::thread::sleep(Duration::from_millis(500));

        // Wait for the start signal to be high
        loop {
            if self.config.starter.is_high() {
                break;
            }
            else {
                if self.config.test_mode {
                    // In test mode, we need to inform user with blinking led
                    self.event.send_event(Event::OverridePwm {
                        pwm_event: PWMEvent::LedBottom([1.0, 0.0, 0.0]),
                        override_state: OverrideState::Override,
                    });
                    std::thread::sleep(Duration::from_millis(100));
                    self.event.send_event(Event::OverridePwm {
                        pwm_event: PWMEvent::LedBottom(team_color),
                        override_state: OverrideState::Override,
                    });
                    std::thread::sleep(Duration::from_millis(100));
                }
                else {
                    std::thread::sleep(Duration::from_millis(50));
                }
            }

            self.trajectory.purge();
        }

        let start_time = Instant::now();
        self.start_time = Some(start_time);
        self.event.send_event(Event::GameStarted { timestamp: start_time, test_mode: self.config.test_mode });
        log::info!("Game started (test: {})", self.config.test_mode);

        //set the bottom led to white while waiting for the first order
        self.event.send_event(Event::OverridePwm {
            pwm_event: PWMEvent::LedBottom([1.0, 1.0, 1.0]),
            override_state: OverrideState::Override,
        });

        //wait for our time to shine
        if !self.config.test_mode {
            loop {
                if start_time.elapsed().as_secs() >= PAMI_START_TIME_SECONDS {
                    break;
                }
                else {
                    std::thread::sleep(Duration::from_millis(100));
                }

                self.trajectory.purge();
            }
        }

        //set the bottom led to green
        log::info!("Go go go!");
        self.event.send_event(Event::OverridePwm {
            pwm_event: PWMEvent::LedBottom([0.0, 1.0, 0.0]),
            override_state: OverrideState::Override,
        });

    }

    fn strat_superstar(&mut self) {

        let init_a = 180.0_f32.to_radians();
        let mut position = self.trajectory.get_position().lock().unwrap();
        position.set_coordinates(Some(2900.0), Some(1900.0), Some(init_a));
        drop(position);


        self.wait_for_start();
         
        let orders = TrajectoryOrderList::new()
            .set_backwards(true)
            .add_order(Order::GotoD { d_mm: 1000.0 });


        self.trajectory
            .execute(orders)
            .unwrap();


        let orders = TrajectoryOrderList::new()
            .set_backwards(false)
            .add_order(Order::GotoA { a_rad: self.mirror_angle(-90.0_f32.to_radians()) })
            .set_max_speed(cocotter::trajectory::RampCfg::Linear, 0.2)
            .add_order(Order::CustomOrder { callback: move_until_void })
            ;

        self.trajectory
            .execute(orders)
            .unwrap();
    }

    fn start_pit(&mut self, game: GameStrategy) {

        let init_a = 0.0;
        let mut position = self.trajectory.get_position().lock().unwrap();
        position.set_coordinates(Some(2900.0), Some(1900.0), Some(init_a));
        drop(position);


        //
        self.wait_for_start();

        let (spd, d1, d2, d3, d4) = match game {
            GameStrategy::FarPit => {
                if self.config.x_negative_color {
                    (1.2 , 500.0, 300.0, 1000.0, 400.0)
                } else {
                    (1.2 , 450.0, 650.0, 800.0, 400.0)
                }
            }

            GameStrategy::MidPit => {
                thread::sleep(Duration::from_millis(1000)); //let a bit for farPit
                (1.1, 300.0, 500.0, 850.0, 200.0)
            }
            
            GameStrategy::NearPit => {
                thread::sleep(Duration::from_millis(1750)); //let a bit for farPit
                (1.0, 150.0, 650.0, 350.0, 200.0)
            }

            _ => {
                log::error!("bad strat");
                (1.0, 0.0, 0.0, 0.0, 0.0)
            }
        };

        let (a1, a2, a3) = if self.config.x_negative_color {
            (135.0_f32, 180.0_f32, -90.0_f32)
        }
        else {
            (-140.0_f32, 180.0_f32, 90.0_f32)
        };

        let orders = TrajectoryOrderList::new()
            .set_no_detection(true)
            .set_backwards(true)
            .set_max_speed(cocotter::trajectory::RampCfg::Linear, spd)
            .add_order(Order::GotoD { d_mm: d1 })
            .add_order(Order::GotoA { a_rad: a1.to_radians()})
            .set_no_detection(false)
            .add_order(Order::GotoD { d_mm: d2 })
            .add_order(Order::GotoA { a_rad: a2.to_radians() })
            .add_order(Order::GotoD { d_mm: d3 })
            .add_order(Order::GotoA { a_rad: a3.to_radians() })
            .add_order(Order::GotoD { d_mm: d4 })
            ;

        self.trajectory
            .execute(orders)
            .unwrap();
    }

    fn run(&mut self) {

        std::thread::sleep(Duration::from_millis(1000));

        match self.config.strategy {
            GameStrategy::Superstar => self.strat_superstar(),
            GameStrategy::FarPit | GameStrategy::MidPit | GameStrategy::NearPit => self.start_pit(self.config.strategy),
        }

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
                self.trajectory.purge();
            }
        }
    }

    fn mirror_angle(&self, angle: f32) -> f32 {
        if self.config.x_negative_color {
            //ensure the angle is between -PI and PI
            let angle = angle.rem_euclid(std::f32::consts::PI * 2.0) - std::f32::consts::PI; // shift to range [-PI, PI]
            //ensure the angle is mirrored for the negative X color
            if angle < 0.0 {
                angle + std::f32::consts::PI // mirror the angle by adding PI
            } else {
                std::f32::consts::PI - angle // mirror the angle by subtracting from PI
            }
        } else {
            // Keep the angle as is for the positive X color
            angle
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

fn _follow_wall<const N: usize>(target_mm : f32, left: bool, order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> {
    let mut dst: Option<f32> = None;
    for event in custom_events.iter() {
        if let Event::BackDistance { distance } = event {
            if left {
                dst = Some(distance[7] as f32);
            } else {
                dst = Some(distance[0] as f32);
            }
        }
    }
    log::info!("Distance to wall: {:?}", dst);

    if let Some(distance) = dst {
        if target_mm < distance {
            //we need to move closer to the wall
            let mut position = trajectory.get_position().lock().unwrap();
            let current_a = position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::A_INDEX];
            let target_a = if left {
                current_a + 0.075 //turn left
            } else {
                current_a - 0.075 //turn right
            };
            position.get_ramps_as_mut()[Order::<N, Event>::A_INDEX].set_target(target_a, false);
        }
        else if target_mm > distance {
            //we need to move away from the wall
            let mut position = trajectory.get_position().lock().unwrap();
            let current_a = position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::A_INDEX];
            let target_a = if left {
                current_a - 0.075 //turn right
            } else {
                current_a + 0.075 //turn left
            };
            position.get_ramps_as_mut()[Order::<N, Event>::A_INDEX].set_target(target_a, false);
        }   
    }

    let mut position = trajectory.get_position().lock().unwrap();
    let current_d = position.get_coordinates().get_raw_linear_coordonate()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT];
    position.get_ramps_as_mut()[Order::<N, Event>::D_INDEX_IN_NON_HOLONOMIC_ROBOT].set_target(current_d - 100.0, false);

    Ok(order_index)
}

fn _align_with_wall<const N: usize>(order_index: usize, config: &OrderConfig<N>, state: &mut OrderState<N>, custom_events: &mut Vec<Event>, _trajectory: &Trajectory<N, Event>) -> Result<usize, TrajectorError> {
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