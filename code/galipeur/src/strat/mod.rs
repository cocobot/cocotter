use core::f32;
use std::mem::Discriminant;
use std::sync::{Arc, Mutex};
use std::{thread::sleep, time::Duration};
use amatheur::XYA;
use asserv::differential::conf::TrajectoryConf;
use asserv::holonomic::{Asserv, RobotSide, TableSide};
use asserv::maths::XY;
use board_common::Team;
use cancaner::GroundThresholdMode;
use embedded_hal::digital::InputPin;
use board_sabotter::{SabotterBoard, SabotterInputs, RGB8};
use flume::Sender;
use log::info;
use pathfinding::{PathGraph, PathGraphBuilder, PathObstacle};

use crate::arfast;
use crate::led::LedMessage;
use crate::meca::{Meca, CleatSide};
use crate::strat::errors::StrategyError;
use crate::strat::realign::{LidarSelect, WallMeasurement};
use board_sabotter::movement::MovementLowLevelHardware;
use crate::opponent_detection::{DetectionMode, OpponentDetection};
use crate::sensors::Sensors;
use crate::strat::utils::{AsservHelper, arfast};

pub mod utils;
pub mod errors;
mod calibration;
pub mod realign;

pub struct Strat<B: SabotterBoard> {
    team: Team,
    leds: Sender<LedMessage>,
    sensors: Sensors<B>,
    meca: Meca<B>,
    asserv: AsservHelper<B>,
    opponent_detection: OpponentDetection,
    rlogger: Sender<String>,

    inputs: SabotterInputs<B::ExInputPin, B::ExInputPin>,

    robot_main: RobotSide,
    robot_aux: RobotSide,
    table_main: TableSide,
    table_aux: TableSide,
    kx: f32,
    pathfinder: PathGraph,

    releases: u64,
    takes: u64,
}

impl<B : SabotterBoard + 'static> Strat<B> {
    pub fn init(
        board: &mut B,
        leds: Sender<LedMessage>,
        sensors: Sensors<B>,
        meca: Meca<B>,
        asserv: Arc<Mutex<Asserv<MovementLowLevelHardware<B>>>>,
        rlogger: Sender<String>,
        opponent_detection: OpponentDetection,
    ) {
        // Build the pathfinding graph
        let pathfinder = {
            let mut builder = PathGraphBuilder::new();

            // Starting zones
            const STARTING_POS: XY = XY::new(1500.0 - 600.0/2.0, 2000.0 - 450.0/2.0);
            const STARTING_EXIT_POS: XY = XY::new(1500.0 - 600.0/2.0, 2000.0 - 600.0);
            let starts = builder.add_mirror_nodes(STARTING_POS);
            let start_exits = builder.add_mirror_nodes(STARTING_EXIT_POS);
            builder.add_mirror_edges(starts, start_exits);

            let grid_index = builder.node_count();
            for ix in 0..=3 {
                for y in [475.0, 800.0, 1125.0] {
                    let xy = XY::new(ix as f32 * 350.0, y);
                    builder.add_node(xy);
                    if ix != 0 {
                        builder.add_node(xy.xflip());
                    }
                }
            }

            builder.add_edges_in_group(grid_index, 1000.0 * 1000.0);
            builder.add_edges_between_groups(grid_index, 500.0 * 500.0);

            builder.build(10.0)
        };

        let instance = Self {
            team: Team::None,
            leds,
            sensors,
            meca,
            asserv: AsservHelper::new(asserv, opponent_detection.clone()),
            opponent_detection,
            rlogger,
            robot_main: RobotSide::Left,
            robot_aux: RobotSide::Right,
            table_main: TableSide::Right,
            table_aux: TableSide::Left,
            kx: 1.0,
            pathfinder,

            takes: 0,
            releases: 0,

            inputs: board.inputs().take().unwrap(),
        };

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            ThreadSpawnConfiguration {
                priority: 15,
                ..Default::default()
            }
            .set()
            .unwrap();
        }

        std::thread::Builder::new()
            .name("strat".into())
            .stack_size(8192)
            .spawn(move || {
                instance.run();
            })
            .expect("spawn strat");

        #[cfg(target_os = "espidf")]
        {
            use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
            ThreadSpawnConfiguration::default().set().unwrap();
        }
    }

    fn run(mut self) {
        sleep(Duration::from_millis(100));

        //adversare
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(0.0, 0.0), 0.0));
        
        
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new( 350.0, 800.0), 300.0)); // spot 4
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(-350.0, 800.0), 300.0)); // spot 5

        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(-700.0, 800.0), 0.0)); // spot 15
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(700.0, 800.0), 0.0)); // spot 14
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(0.0, 800.0), 0.0)); // spot 18

        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(-400.0, 800.0), 0.0)); //
        self.pathfinder.obstacles.push(PathObstacle::new(&XY::new(400.0, 800.0), 0.0)); //


        self.prepare_match();
        //self.pathfinding_test();
        //self.test_movement();
        self.test_eirbot_2();
        //self.take_first_crates();

        self.return_to_start();
        self.end_of_match();

    }

    //----------

    fn setup_position(&mut self) -> Result<(), StrategyError>{
        self.sensors.set_ground_mode(GroundThresholdMode::Raw, 10);

        self.opponent_detection.set_mode(DetectionMode::Off);

        let end_angle = arfast(RobotSide::Back, TableSide::Up);

        #[cfg(not(target_os = "espidf"))]
        {
            self.asserv.reset_position(self.kx * 1150.0, 1740.0, end_angle);
            self.asserv.teleport(self.kx * 1150.0, 1740.0, end_angle);
            return Ok(())
        }

        
        {
            let initial_angle = arfast(RobotSide::Back, self.table_main);

            self.asserv.teleport(self.kx * 1300.0, 1500.0, initial_angle);
            self.asserv.reset_position(0.0, 0.0, initial_angle);

            self.asserv.enable_motor_control();

            self.asserv.goto_xya(- self.kx * 75.0, 0.0, initial_angle)?;

            self.meca.init(self.team);

            let x_back = match realign::measure_wall(&self.sensors, RobotSide::Back, self.table_main) {
                Some(measure) => if let Some(x) = measure.x { x } else { return Err(StrategyError::SensorUnavailable)},
                None => return Err(StrategyError::SensorUnavailable),
            };

            let current = self.asserv.position();
            self.asserv.reset_position(x_back, current.y, current.a);


            self.asserv.goto_a(end_angle)?;
            std::thread::sleep(Duration::from_secs(1));

            let y_back = match realign::measure_wall(&self.sensors, RobotSide::Back, TableSide::Up) {
                Some(measure) => if let Some(y) = measure.y { y } else { return Err(StrategyError::SensorUnavailable)},
                None => return Err(StrategyError::SensorUnavailable),
            };

            let current = self.asserv.position();
            self.asserv.reset_position(current.x, y_back, current.a);

            self.asserv.goto_xya(self.kx * 1150.0, 1740.0, end_angle)?;
            self.asserv.teleport(self.kx * 1150.0, 1740.0, end_angle);

            Ok(())
        }
    }

    fn prepare_match(&mut self) {
        log::info!("Color selection");
        sl(1500);
        log::info!("OK ?");

        self.asserv.disable_motor_control();

        //waiting for starter to be inserted
        loop {
            self.sensors.set_ground_mode(GroundThresholdMode::Delta, 40);

            let team = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };
            self.leds.send(LedMessage::GameTeam { team }).ok();
            
            let _back = self.sensors.ground_lidar(RobotSide::Back);
           //let left = self.sensors.ground_lidar(RobotSide::Left);
           //let right = self.sensors.ground_lidar(RobotSide::Right);

          //elf.asserv.reset_position(0.0, 100.0, arfast(RobotSide::Left, TableSide::Down));

          // let wall_meas = realign::measure_wall(&self.sensors, RobotSide::Left, TableSide::Down);
          // let def = WallMeasurement {x: Some(-1.0), y: Some(-1.0)};
          // log::info!("B both {:?}", wall_meas.unwrap_or(def).y.unwrap());

            sleep(Duration::from_millis(250));

            if self.inputs.starter.is_low().unwrap_or(false) {
                log::info!("Color selected: {}", team.name());
                self.team = team;
                if self.team == Team::Left {
                    self.robot_main = RobotSide::Right;
                    self.robot_aux  = RobotSide::Left;
                    self.table_main = TableSide::Left;
                    self.table_aux  = TableSide::Right;
                    self.kx = -1.0;
                }
                else {
                    self.robot_main = RobotSide::Left;
                    self.robot_aux  = RobotSide::Right;
                    self.table_main = TableSide::Right;
                    self.table_aux  = TableSide::Left;
                    self.kx = 1.0;
                }     
                break;
            }

        }

       
       //         self.opponent_detection.set_mode(DetectionMode::Off);
//
       // self.asserv.reset_position(0.0, 0.0, arfast(RobotSide::Back, TableSide::Left));
       //
       // self.asserv.enable_motor_control();
       //
       // loop {
       //     if self.approach_and_take(RobotSide::Back, TableSide::Left).is_ok() {
       //         sleep(Duration::from_millis(1000));
       //         self.release(RobotSide::Back, TableSide::Left).ok();
       //         self.asserv.goto_xya(0.0, 0.0, arfast(RobotSide::Back, TableSide::Left)).ok();
       //     }
       //     else {
       //         sleep(Duration::from_millis(250));
       //     }
       // }

        if self.setup_position().is_err() {
            log::warn!("LJDJKSQKLDSQ");
        }
        //self.sensors.ground_lidar_power_off();

        self.opponent_detection.set_mode(DetectionMode::OnTable);


        //waiting for starter to be removed
        let mut blink = false;
        loop {
            let team = if blink { self.team } else { Team::None };
            self.leds.send(LedMessage::GameTeam { team }).ok();
            blink = !blink;

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_high().unwrap_or(false) {
                self.asserv.set_match_started();
                self.leds.send(LedMessage::GameTeam { team: self.team }).ok();
                log::info!("Match started");
                break;
            }
        }

        //loop {
        //²    if self.asserv.goto_xya(self.kx * 1150.0, 1300.0, arfast(RobotSide::Back, TableSide::Up)).is_ok() {
        //²        break
        //²    }
        //²    else {
        //²        sleep(Duration::from_millis(100));
        //²    }
        //²}
        //²loop {
        //²    self.asserv.goto_xya(self.kx * 1150.0, 1300.0, arfast(RobotSide::Back, TableSide::Up)).ok();
        //²    self.asserv.goto_xya(self.kx * 1150.0, 400.0, arfast(RobotSide::Back, TableSide::Up)).ok();
        //²    sleep(Duration::from_millis(250));
        //²}


    }

    fn take_crate_spot(&mut self, x: f32, y: f32, face: RobotSide, side: TableSide, recallage: bool) -> Result<(), StrategyError>{
        const PRETAKE_DISTANCE : f32 = 325.0;

        let offset_take_xy = match side {
            TableSide::Down => (0.0, PRETAKE_DISTANCE),
            TableSide::Up => (0.0, -PRETAKE_DISTANCE),
            TableSide::Left => (PRETAKE_DISTANCE, 0.0),
            TableSide::Right => (-PRETAKE_DISTANCE, 0.0),
        };

        let face = match self.meca.prepare_direct_take(Some(face), CleatSide::Both) {
            Some(face) => face,
            None => {return Err(StrategyError::StupidOrder)}
        };

        log::info!("Pathfinder ");
        self.pathfinder_xya(x + offset_take_xy.0, y + offset_take_xy.1, arfast(face, side))?;

        //sl(2000);
        log::info!("Pathfinder ");
        self.approach_and_take(face, side, recallage)?;

        Ok(())
    }

    fn pathfinder_xya(&mut self, x: f32, y: f32, a: f32) -> Result<(), StrategyError> {
        self.asserv.goto_a(a).ok();
        self.update_opponent();
        let start = self.pathfinder.nearest_node(&self.asserv.position().xy());
        let goal = self.pathfinder.nearest_node(&XY::new(x, y));
        if let Some(path) = self.pathfinder.find_path(start, goal) {
            let asserv_path: Vec<XY> = path.into_iter().map(|id| self.pathfinder.get_node_xy(id)).collect();
            self.asserv.run_path(&asserv_path)?;
        } else {
            rome::warn!(self.rlogger, "Cannot find a path");
        }

        self.asserv.goto_xya(x, y, a)
    }

    fn release_on_spot(&mut self, x: f32, y: f32, face: RobotSide, side: TableSide) -> Result<(), StrategyError>{
        const PRERELEASE_DISTANCE : f32 = 325.0;

        let offset_take_xy = match side {
            TableSide::Down => (0.0, PRERELEASE_DISTANCE),
            TableSide::Up => (0.0, -PRERELEASE_DISTANCE),
            TableSide::Left => (PRERELEASE_DISTANCE, 0.0),
            TableSide::Right => (-PRERELEASE_DISTANCE, 0.0),
        };

        let face = match self.meca.prepare_release(Some(face)) {
            Some(face) => face,
            None => {return Err(StrategyError::StupidOrder)}
        };

        self.pathfinder_xya(x + offset_take_xy.0, y + offset_take_xy.1, arfast(face, side))?;
        self.release(face, side)?;

        Ok(())
    }

    fn recallage(&self, side : TableSide) {
        if self.asserv.goto_a(arfast(RobotSide::Back, side)).is_err() {
            return;
        }
        sleep(Duration::from_millis(500));



        let measure = realign::measure_wall(&self.sensors, RobotSide::Back, side);
        log::info!("Measure: {:?}", measure);

        if let Some(measure) = measure {
            let mut current = self.asserv.position();
            if let Some(x) = measure.x {
                current.x = x;
            }
            if let Some(y) = measure.y {
                current.y = y;
            }
            self.asserv.reset_position(current.x, current.y, current.a);
        }

    }

    fn take_id(&mut self, id: usize, alternate: bool) -> Result<(), StrategyError> {
        log::info!("Req Tak {}", id);
        if self.takes & (1 << id) != 0 {
            return Ok(())
        }
        
        if id == 0 {
            //celui qui est tout proche
            self.take_crate_spot(self.kx * 1350.0, 1200.0, RobotSide::Back, self.table_main, false)?;
            self.recallage(self.table_main);
        }
        if id == 1 {
            //celui qui est tout proche
            self.take_crate_spot(self.kx * 1350.0,  400.0, RobotSide::Back, self.table_main, true).ok();
            self.recallage(self.table_main);
            let pos = self.asserv.position();
            self.asserv.goto_xya(pos.x, 300.0, arfast(RobotSide::Back, TableSide::Down)).ok();
            self.recallage(TableSide::Down);
        }
        if id == 2 {
            //celui qui est tout proche
            self.take_crate_spot(self.kx * 400.0,  175.0, RobotSide::Back, TableSide::Down, true).ok();
            self.recallage(TableSide::Down);            
        }
        if id == 3 {
            if alternate {
                self.take_crate_spot(self.kx * 350.0,  800.0, RobotSide::Back, TableSide::Down, false).ok();

            }
            else {
                self.take_crate_spot(self.kx * 350.0,  800.0, self.robot_main, TableSide::Up, false).ok();
            }
            if self.kx > 0.0 {
                self.pathfinder.obstacles[1].new_radius(0.0);
            }
            else {
                self.pathfinder.obstacles[2].new_radius(0.0);
            }
        }
        if id == 4 {
            self.take_crate_spot(-self.kx * 350.0,  800.0, RobotSide::Back, TableSide::Down, true).ok();

            if self.kx > 0.0 {
                self.pathfinder.obstacles[2].new_radius(0.0);
            }
            else {
                self.pathfinder.obstacles[1].new_radius(0.0);
            }
        }
        if id == 5 {
            self.take_crate_spot(-self.kx * 400.0,  175.0, RobotSide::Back, TableSide::Down, true).ok();
            self.recallage(TableSide::Down);
        }

        self.takes |= 1 << id;

        Ok(())
    }

    fn release_id(&mut self, id: usize, alternate: bool) -> Result<(), StrategyError> {
        log::info!("Req Rel {}", id);
        if self.releases & (1 << id) != 0 {
            return Ok(())
        }

        if id == 0 {
            self.release_on_spot(self.kx * 1400.0, 800.0, RobotSide::Back, self.table_main)?;
        }
        if id == 1 {
            self.release_on_spot(self.kx * 800.0, 100.0, RobotSide::Back, TableSide::Down)?;
        }
        if id == 2 {
            self.release_on_spot(self.kx * 0.0, 800.0, RobotSide::Back, self.table_aux)?;
            self.pathfinder.obstacles[5].new_radius(300.0);

        }
        if id == 3 {
            self.release_on_spot(self.kx * 700.0, 800.0, self.robot_main, TableSide::Down)?;
        
            if self.kx > 0.0 {
                self.pathfinder.obstacles[4].new_radius(300.0);
            }
            else {
                self.pathfinder.obstacles[3].new_radius(300.0);
            }

        }
        if id == 4 {
            self.release_on_spot(self.kx * 0.0, 100.0, RobotSide::Back, TableSide::Down)?;
        }

        self.releases |= 1 << id;

        Ok(())
    }

    fn check_goto_eom(&mut self) {
        let tme = self.asserv.ellapsed_time_since_start();
        log::info!("Match time {}s", tme.as_secs());
        if tme > Duration::from_secs(90) {
            self.return_to_start();
        }
    }

    fn strat_2(&mut self) -> Result<(), StrategyError> {
        log::info!("USE STRAT 2");
        self.leds.send(LedMessage::StratFlash(RGB8 { r: 0, g: 0, b: 255 })).ok();

        op(self.take_id(3, true))?;
            self.check_goto_eom();

        op(self.release_id(3, false))?;
        self.check_goto_eom();

        self.take_id(4, false).ok();
        self.check_goto_eom();

        self.strat_1()
    }

    fn strat_1(&mut self) -> Result<(), StrategyError> {
        log::info!("USE STRAT 1");
        self.leds.send(LedMessage::StratFlash(RGB8 { r: 255, g: 200, b: 0 })).ok();
        op(self.release_id(1, false))?;
        self.check_goto_eom();
        op(self.take_id(2, false))?;
        self.check_goto_eom();
        op(self.take_id(3, false))?;
        self.check_goto_eom();
        op(self.release_id(2, false))?;
        self.check_goto_eom();
        op(self.release_id(3, false))?;
        self.check_goto_eom();
        op(self.pathfinder_xya(self.kx * 1250.0, 1400.0, arfast(RobotSide::Back, self.table_main)))?;
        self.check_goto_eom();
        op(self.take_id(4, false))?;
        self.check_goto_eom();
        self.return_to_start();
    }

    fn test_eirbot_2 (&mut self){ 

       let init_pos = self.asserv.position();
       //self.opponent_detection.set_mode(DetectionMode::OnTable);
       //self.asserv.set_stop_mode(utils::StopMode::WaitAndResume);

       //self.asserv.goto_xya(init_pos.x , 1200.0, init_pos.a).ok();
       //loop {
       //    //self.asserv.goto_xya(init_pos.x , 1350.0, init_pos.a).ok();
       //    //self.asserv.goto_xya(init_pos.x , 1000.0, init_pos.a).ok();
       //    sleep(Duration::from_millis(10));
       //}

        self.recallage(TableSide::Up);

        self.opponent_detection.set_mode(DetectionMode::OnTable);
        //self.opponent_detection.set_mode(DetectionMode::Off);
        self.asserv.set_stop_mode(utils::StopMode::WaitAndResume);
        self.asserv.goto_xya(self.kx * 1150.0, 1400.0, arfast(RobotSide::Back, TableSide::Up)).ok();
        self.asserv.goto_xya(self.kx * 900.0, 1200.0, arfast(RobotSide::Back, TableSide::Up)).ok();


       // self.return_to_start();
        if self.kx > 0.0 {
            self.pathfinder.obstacles[3].new_radius(300.0);
            self.pathfinder.obstacles[6].new_radius(300.0);
        }
        else {
            self.pathfinder.obstacles[4].new_radius(300.0);
            self.pathfinder.obstacles[7].new_radius(300.0);

        }
        
        self.asserv.set_stop_mode(utils::StopMode::Reject);
        self.take_id(0, false).ok();
        self.take_id(1, false).ok();
        self.release_id(0, false).ok();


        let mut last_move_pos = self.asserv.position();
        let mut last_move_time = std::time::Instant::now();

        loop {
            let pos = self.asserv.position();
            let dx = pos.x - last_move_pos.x;
            let dy = pos.y - last_move_pos.y;
            if dx * dx + dy * dy > 5.0 * 5.0 {
                last_move_pos = pos;
                last_move_time = std::time::Instant::now();
            } else if last_move_time.elapsed() > Duration::from_secs(4) {
                self.unstuck_me();
                last_move_pos = self.asserv.position();
                last_move_time = std::time::Instant::now();
            }

            let r = self.strat_1();
            if matches!(r, Err(StrategyError::EndOfMatch)) {
                self.end_of_match();
            }

            let r = self.strat_2();
            if matches!(r, Err(StrategyError::EndOfMatch)) {
                self.end_of_match();
            }

            sl(10);
        }
        
        


        loop {        sleep(Duration::from_millis(500)); }

       
       self.asserv.goto_a(arfast(RobotSide::Back, TableSide::Down)).ok();
        let y_back = realign::measure_wall(&self.sensors, RobotSide::Back, TableSide::Down);
        log::info!("POkio {:?}", y_back);
        if let Some(pos) = y_back {
            let current = self.asserv.position();
            if let Some(y) = pos.y {
                self.asserv.reset_position(current.x, y ,current.a);
            }
        }

        self.release_on_spot(self.kx * 1400.0, 800.0, RobotSide::Back, self.table_main).ok();

        
        self.asserv.goto_xya(self.kx * 1050.0, 850.0, self.asserv.position().a).ok();
        self.asserv.goto_xya(self.kx * 1050.0, 400.0, self.asserv.position().a).ok();

        self.asserv.goto_xya(self.kx * 1050.0, 400.0, arfast(RobotSide::Back, TableSide::Down)).ok();


        let y_back = realign::measure_wall(&self.sensors, RobotSide::Back, TableSide::Down);
        log::info!("YBACK {:?}", y_back);

        if let Some(pos) = y_back {
            let current = self.asserv.position();
            if let Some(y) = pos.y {
                self.asserv.reset_position(current.x, y,current.a);
            }
        }

               self.asserv.goto_a(arfast(RobotSide::Back, self.table_main)).ok();

        sleep(Duration::from_millis(500));       
        let x_back = realign::measure_wall(&self.sensors, RobotSide::Back, self.table_main);

        if let Some(pos) = x_back {
            let current = self.asserv.position();
            if let Some(x) = pos.x {
                self.asserv.reset_position(x, current.y,current.a);
            }
        }



        self.release_on_spot(self.kx * 700.0, 850.0, RobotSide::Back, TableSide::Up).ok();

        


        //self.asserv.goto_xya(self.kx * 1100.0, 475.0, self.asserv.position().a).ok();
        //self.asserv.goto_xya(self.kx * 1250.0, 230.0, arfast(RobotSide::Left, TableSide::Up)).ok();
        //self.asserv.goto_xya(self.kx * 800.0, 230.0, arfast(RobotSide::Left, TableSide::Up)).ok();

        self.take_crate_spot(self.kx * 350.0,  800.0, RobotSide::Back, TableSide::Up, false).ok();
        self.take_crate_spot(self.kx * 400.0,  150.0, RobotSide::Back, TableSide::Down, true).ok();
////
        self.asserv.set_stop_mode(utils::StopMode::Reject);
        self.release_on_spot(self.kx * 0.0, 850.0, RobotSide::Back, TableSide::Up).ok();
////
        self.asserv.set_stop_mode(utils::StopMode::WaitAndResume);
////
////
        self.asserv.goto_xya(self.kx * 800.0, 550.0, arfast(RobotSide::Back, TableSide::Down)).ok();
////
         sleep(Duration::from_millis(500));
         let y_back = realign::measure_wall(&self.sensors, RobotSide::Back, TableSide::Down);
        log::info!("YBACK {:?}", y_back);
////
////
        if let Some(pos) = y_back {
            let current = self.asserv.position();
            if let Some(y) = pos.y {
                self.asserv.reset_position(current.x, y,current.a);
            }
        }
////
////
        self.release_on_spot(self.kx * 800.0, 150.0, self.robot_aux, TableSide::Down).ok();
////
        self.asserv.goto_xya(self.kx * 800.0, 550.0, self.asserv.position().a).ok();
        self.asserv.set_stop_mode(utils::StopMode::Reject);
        //self.take_crate_spot(-self.kx * 350.0,  850.0, RobotSide::Back, TableSide::Up).ok();
        //self.take_crate_spot(-self.kx * 400.0,  150.0, self.robot_aux, TableSide::Down).ok();
////
        self.asserv.set_stop_mode(utils::StopMode::WaitAndResume);
////
        self.asserv.goto_xya(self.kx * 600.0, 500.0, self.asserv.position().a).ok();
        self.asserv.goto_xya(self.kx * 1100.0, 500.0, self.asserv.position().a).ok();
        self.asserv.goto_xya(self.kx * 1100.0, 1200.0, self.asserv.position().a).ok();
        self.asserv.goto_xya(self.kx * 600.0, 1200.0, self.asserv.position().a).ok();
////
       
        //self.release_on_spot(self.kx * 150.0, 1450.0, self.robot_aux, TableSide::Up).ok();


        
        //self.take_crate_spot(-self.kx * 350.0,  800.0, RobotSide::Back, TableSide::Up).ok();
        //self.release_on_spot(self.kx * 0.0, 800.0, RobotSide::Back, TableSide::Up).ok();
//
        //
        //self.release_on_spot(self.kx * 0.0, 100.0, self.robot_aux, TableSide::Down).ok();
        //self.release_on_spot(self.kx * 800.0, 800.0, self.robot_aux, self.table_aux).ok();
        
        //self.end_of_match();
        self.return_to_start();  
    }

    fn take_first_crates (&mut self){ 
        self.asserv.goto_xya(self.kx * 1000.0, 1200.0, arfast(self.robot_main, self.table_main)).ok();
        let side = self.meca.prepare_direct_take(Some(self.robot_main), CleatSide::Right).unwrap();
        self.asserv.goto_xya(self.kx * 1130.0, 1200.0, arfast(side, self.table_main)).ok();
        self.meca.direct_take(side);
        rome::info!(self.rlogger, "first bunch of crates taken !");
   
        let side = self.meca.prepare_direct_take(Some(self.robot_main), CleatSide::Right).unwrap();
        self.asserv.goto_xya(self.kx * 1000.0, 380.0, arfast(self.robot_main, self.table_main)).ok();
        self.asserv.goto_xya(self.kx * 1130.0, 380.0, arfast(self.robot_main, self.table_main)).ok();
        self.meca.direct_take(side);
        rome::info!(self.rlogger, "first bunch of crates taken !");
        
        let prefered_side = RobotSide::Back;
        let side = self.meca.prepare_direct_take(Some(prefered_side), CleatSide::Right).unwrap();
        self.asserv.goto_xya(self.kx * 315.0, 500.0, arfast(side, TableSide::Down)).ok();
        self.asserv.goto_xya(self.kx * 315.0, 400.0, arfast(side, TableSide::Down)).ok();
        self.meca.direct_take(side);

        self.asserv.goto_xya(self.kx * 315.0, 500.0, arfast(side, TableSide::Up)).ok();        
        let side = self.meca.prepare_direct_take(Some(prefered_side), CleatSide::Right).unwrap();
        self.asserv.goto_xya(self.kx * 290.0, 610.0, arfast(side, TableSide::Up)).ok();        
        self.meca.direct_take(side);

        std::thread::sleep(Duration::from_secs(5));
        

        self.meca.end_of_match();

   
    }

    #[allow(dead_code)]
    fn test_eirbot(&mut self){
        // Right side doesn't work well, so for use of left side
        self.robot_main = RobotSide::Left;

        self.asserv.goto_xya(self.kx * 1200.0, 1600.0, arfast(RobotSide::Back, TableSide::Up)).ok();
        self.asserv.goto_xya(self.kx * 1200.0, 1500.0, arfast(self.robot_main, self.table_main)).ok();
        _ = self.meca.prepare_direct_take(Some(self.robot_main), CleatSide::Both);
        self.asserv.run_path(&[
            XY::new(self.kx*1000.0, 1400.0),
            XY::new(self.kx*1000.0, 1300.0),
            XY::new(self.kx*1000.0, 1200.0),
        ]).ok();
        std::thread::sleep(Duration::from_secs(1));

        self.asserv.goto_xya(self.kx * 1130.0, 1200.0, arfast(self.robot_main, self.table_main)).ok();
        self.meca.direct_take(self.robot_main);

        std::thread::sleep(Duration::from_secs(1));

        self.asserv.goto_xya(self.kx * 1100.0, 800.0, arfast(self.robot_main, self.table_main)).ok();
        self.meca.release(self.robot_main);

        std::thread::sleep(Duration::from_secs(1));
        self.asserv.goto_xya(self.kx * 1000.0, 800.0, arfast(self.robot_main, self.table_main)).ok();
    }

    #[allow(dead_code)]
    fn pathfinding_test(&mut self) {
        let start = self.pathfinder.nearest_node(&self.asserv.position().xy());
        let goal = self.pathfinder.nearest_node(&XY::new(0.0, 1000.0));
        if let Some(path) = self.pathfinder.find_path(start, goal) {
            let asserv_path: Vec<XY> = path.into_iter().map(|id| self.pathfinder.get_node_xy(id)).collect();
            self.asserv.run_path(&asserv_path).ok();
        } else {
            rome::warn!(self.rlogger, "Cannot find a path");
        }
    }

    fn update_opponent(&mut self) {
        match self.opponent_detection.opponent_positon() {
            Some(pos) => {
                log::info!("Pos {:?}", pos);
                self.pathfinder.obstacles[0].new_radius(460.0);
                self.pathfinder.obstacles[0].new_xy(pos);

                let robot_pos = self.asserv.position();
                let dx = pos.x - robot_pos.x;
                let dy = pos.y - robot_pos.y;
                let angle_table = dy.atan2(dx);
                let angle_body = (angle_table - robot_pos.a).to_degrees().rem_euclid(360.0);
                let led_offset = self.opponent_detection.conf().led_angle_offset;
                let idx = ((angle_body + led_offset).rem_euclid(360.0) / 360.0 * 40.0) as usize % 40;
                self.leds.send(LedMessage::OpponentDot(Some(idx))).ok();
            },
            None => {
                self.pathfinder.obstacles[0].new_radius(0.0);
                self.leds.send(LedMessage::OpponentDot(None)).ok();
            }
        }
    }

    fn unstuck_me(&mut self) {
        log::warn!("Stuck for 4s, attempting unstuck");
        self.leds.send(LedMessage::Rainbow).ok();
        // TODO: actual unstuck logic
    }

    fn return_to_start(&mut self) -> !{
        self.asserv.goto_a(arfast(RobotSide::Back, TableSide::Up)).ok();
        self.update_opponent();
        let start = self.pathfinder.nearest_node(&self.asserv.position().xy());
        let goal = self.pathfinder.nearest_node(&XY::new(self.kx*700.0, 1400.0));
        if let Some(path) = self.pathfinder.find_path(start, goal) {
            let asserv_path: Vec<XY> = path.into_iter().map(|id| self.pathfinder.get_node_xy(id)).collect();
            if self.asserv.run_path(&asserv_path).is_ok() {
                    self.asserv.goto_a(arfast(RobotSide::Back, self.table_main)).ok();
                    self.recallage(self.table_main);
                    while self.asserv.ellapsed_time_since_start().as_secs() < 96 {
                        sleep(Duration::from_millis(100));
                        log::info!("Not now : {}", self.asserv.ellapsed_time_since_start().as_secs());
                    }

                    self.asserv.goto_xya(self.kx * 1125.0, self.asserv.position().y, arfast(RobotSide::Back, self.table_main)).ok();
                    self.asserv.goto_xya(self.kx * 1125.0, 1760.0, arfast(RobotSide::Back, self.table_main)).ok();
                    self.meca.end_of_match();
            }
        } else {
            rome::warn!(self.rlogger, "Cannot find a path");
        }


        self.end_of_match();
    }

    #[allow(dead_code)]
    fn test_movement(&mut self) {
        let prefered_side = self.robot_main;

        self.asserv.reset_position(0.0, 0.0, arfast(RobotSide::Back, TableSide::Down));

        //meca raise drop
        let side = self.meca.prepare_direct_take(Some(prefered_side), CleatSide::None);
        if side.is_none() {
            log::warn!("All sides are full, cannot prepare direct take");
            return;
        }
        let side = side.unwrap();

        //meca take
        self.meca.direct_take(side);
        self.asserv.goto_a(arfast(self.robot_main, TableSide::Up)).ok();
        self.meca.prepare_direct_take(Some(prefered_side), CleatSide::Both);

        std::thread::sleep(Duration::from_secs(1));

        self.asserv.run_path(&[
            XY::new(self.kx*50.0, 200.0),
        ]).ok();

        let side = self.meca.prepare_release(Some(prefered_side));
        if side.is_none() {
            log::warn!("Nothing to release");
            return;
        }
        else {
            let side = side.unwrap();
            self.meca.release(side);
        }

        self.asserv.goto_xya(0.0, 0.0, arfast(self.robot_main, TableSide::Up)).ok();
        self.asserv.goto_xya(0.0, 0.0, arfast!(Back, Down)).ok();
        std::thread::sleep(Duration::from_secs(1));
    }

    pub fn release(&self, face: RobotSide, wall: TableSide) -> Result<(), StrategyError> {
        const BACK_MV_RELEASE : f32 = 140.0;
       
        let face = match self.meca.prepare_release(Some(face)) {
            Some(face) => face,
            None => {return Err(StrategyError::StupidOrder)}
        };
        self.asserv.goto_a(arfast(face, wall))?;

        self.meca.release(face);

        
        let normal_body = realign::face_normal_angle(face);
        let heading = self.asserv.position().a;
        let normal_world = normal_body + heading;

        let dx = BACK_MV_RELEASE * normal_world.cos();
        let dy = BACK_MV_RELEASE * normal_world.sin();
        self.asserv.goto_xy_rel(-dx, -dy)?;

        let dx = (BACK_MV_RELEASE + 30.0) * normal_world.cos();
        let dy = (BACK_MV_RELEASE  + 30.0) * normal_world.sin();
        self.asserv.goto_xy_rel(dx, dy)?;
        self.asserv.goto_xy_rel(-dx, -dy)?;

     
       Ok(())
    }

    pub fn approach_and_take(
        &self,
        face: RobotSide,
        wall: TableSide,
        recallage: bool
    ) -> Result<(), StrategyError> {
        approach_and_take(&self.sensors, &self.meca, &self.asserv, face, wall, recallage)
    }

    fn end_of_match(&mut self) -> ! {
        self.sensors.set_ground_mode(GroundThresholdMode::Delta, 40);
        self.sensors.ground_lidar_power_off();
        self.meca.end_of_match();
        loop {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}

/// Approach a cleat using lidar-based realignment, then take.
///
/// Sequence: prepare meca → slow → measure distance → advance → measure lateral → correct → take.
pub fn approach_and_take<B: SabotterBoard + 'static>(
    sensors: &Sensors<B>,
    meca: &Meca<B>,
    asserv: &AsservHelper<B>,
    face: RobotSide,
    wall: TableSide,
    recallage: bool,
) -> Result<(), StrategyError> {
    const CENTER_TO_FACE : f32 = 120.0;
    const SHIFT_TRANSLATION : f32 = 35.0;
    const REJECT_DISTANCE : f32 = 350.0;

    const TARGET_DISTANCE: f32 = 150.0; // mm face→cleat
    const TARGET_LATERAL: f32 = 0.0;    // mm, 0 = centered
    const SLOW_SPEED: f32 = 200.0;
    const SLOW_ACC: f32 = 500.0;

log::info!("AA");

    // 0. Prepare meca
    let face = match meca.prepare_direct_take(Some(face), CleatSide::Both) {
        Some(face) => face,
        None => {return Err(StrategyError::StupidOrder)},
    };
    log::info!("A4");
        //sl(2000);

    asserv.goto_a(arfast(face, wall))?;
    log::info!("A3");
        //sl(2000);


log::info!("Azzz2");
    sleep(Duration::from_millis(250));

    let normal_body = realign::face_normal_angle(face);
    let tangent_body = realign::face_tangent_angle(face);

    // 1. Slow speed
    let (saved_speed, saved_acc) = asserv.xy_cruise_speed();
    //asserv.set_xy_cruise_speed(SLOW_SPEED, SLOW_ACC);

    let result = (|| -> Result<(), StrategyError> {
        // Body→world rotation: add robot heading to body-frame angles
        let heading = asserv.position().a;
        let normal_world = normal_body + heading;
        let tangent_world = normal_world + f32::consts::FRAC_PI_2;
log::info!("A2dsds54d56sq");

        // 2. Measure perpendicular distance (face → obstacle, in body frame)
        let mut face_distance = realign::measure_face_distance(sensors, face, LidarSelect::Low)
            .or(Some(100.0)).unwrap();
        log::info!("approach_and_take: face_distance={:.1} mm", face_distance);

        if face_distance > REJECT_DISTANCE {
            face_distance = 80.0 + CENTER_TO_FACE;
            //return Err(StrategyError::StupidOrder);
        }

        // 3. Advance to target distance (move along face normal, in world frame)
        let dist_correction = face_distance - CENTER_TO_FACE;
        let dx = dist_correction * normal_world.cos();
        let dy = dist_correction * normal_world.sin();
        let dx_lat = SHIFT_TRANSLATION * tangent_world.cos();
        let dy_lat = SHIFT_TRANSLATION * tangent_world.sin();
        log::info!("D {} {}", dx + dx_lat, dy + dy_lat);
                //sl(2000);
   
        asserv.goto_xy_rel(-dx_lat, -dy_lat)?;
       // sl(2000);
 meca.prepare_direct_take(Some(face), CleatSide::Left);
sl(250);
        asserv.goto_xy_rel(dx, dy)?;
         //       sl(2000);

        asserv.goto_xy_rel(2.0 * dx_lat, 2.0 * dy_lat)?;

       /// // 4. Measure lateral offset
       /// let lateral = realign::measure_edge(sensors, face, LidarSelect::Low)
       ///     .ok_or(StrategyError::SensorUnavailable)?;
///
       /// // 5. Correct lateral position (in world frame)
       /// let lateral_correction = TARGET_LATERAL - lateral;
       /// let dx = lateral_correction * tangent_world.cos();
       /// let dy = lateral_correction * tangent_world.sin();
       // asserv.goto_xy_rel(dx, dy)?;

        Ok(())
    })();

    // Restore speed regardless of outcome
   // asserv.set_xy_cruise_speed(saved_speed, saved_acc);
    if result.is_err() {
        meca.prepare_direct_take(Some(face), CleatSide::Both);
        return result;
    }
        //sl(2000);

    // 6. Take
    meca.direct_take(face);
        //sl(2000);

    if face == RobotSide::Back {
        if recallage {             if let Some(pos) = realign::measure_wall(&sensors, face, wall) {
                let mut current_pos = asserv.position();
                if let Some(x) = pos.x {
                    current_pos.x = x;
                }
                if let Some(y) = pos.y {
                    current_pos.y = y;
                }

                asserv.reset_position(current_pos.x, current_pos.y, current_pos.a);
            }
        }
    }
    

    Ok(())
}


fn sl(millis : u64) {
    sleep(Duration::from_millis(millis))
}

fn op(ret: Result<(), StrategyError>) -> Result<(), StrategyError> {
            match ret {
                Err(StrategyError::OpponentDetected) => Err(StrategyError::OpponentDetected),
                _ => Ok(()),
            }
       }