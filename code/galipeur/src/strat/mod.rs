use std::sync::{Arc, Mutex};
use std::{thread::sleep, time::Duration};
use asserv::holonomic::{Asserv, RobotSide, TableSide};
use asserv::maths::XY;
use board_common::Team;
use embedded_hal::digital::InputPin;
use board_sabotter::{SabotterBoard, SabotterInputs};
use flume::Sender;
use pathfinding::{PathGraph, PathGraphBuilder};

use crate::arfast;
use crate::led::LedMessage;
use crate::meca::{Meca, CleatSide};
use board_sabotter::movement::MovementLowLevelHardware;
use crate::opponent_detection::OpponentDetection;
use crate::sensors::Sensors;
use crate::strat::utils::{AsservHelper, arfast};

pub mod utils;
pub mod errors;
mod calibration;
mod realign;

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

            inputs: board.inputs().take().unwrap(),
        };

        std::thread::Builder::new()
            .name("strat".into())
            .stack_size(8192)
            .spawn(move || {
                instance.run();
            })
            .expect("spawn strat");
    }

    fn run(mut self) {
        self.prepare_match();
        //self.pathfinding_test();
        //self.test_movement();
        self.take_first_crates();

        self.return_to_start();
        self.end_of_match();

    }

    //----------

    fn prepare_match(&mut self) {
        log::info!("Color selection");

        self.sensors.ground_lidar_power_off();
        //self.sensors.ground_lidar(RobotSide::Back);

        //waiting for starter to be inserted
        loop {
            let team = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };
            self.leds.send(LedMessage::GameTeam { team }).ok();

            sleep(Duration::from_millis(100));

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
                self.meca.init(team);
                break;
            }

        }

        std::thread::sleep(Duration::from_secs(1));

        //start robot with back on the up side of table in the start area
        //there's a crate between the robot and up side
        self.asserv.reset_position(self.kx*(900.0 + 170.0 + 50.0), 2000.0 - 130.0 - 50.0, arfast(RobotSide::Back, TableSide::Up));

        //TODO : add autoset and go to final position in start area
        //self.sensors.ground_lidar(RobotSide::Back);

        self.asserv.goto_xya(self.kx*1150.0, 1800.0, arfast(RobotSide::Back, TableSide::Up)).ok();

        //waiting for starter to be removed
        let mut blink = false;
        loop {
            let team = if blink { self.team } else { Team::None };
            self.leds.send(LedMessage::GameTeam { team }).ok();
            blink = !blink;

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_high().unwrap_or(false) {
                self.leds.send(LedMessage::GameTeam { team: self.team }).ok();
                log::info!("Match started");
                break;
            }
        }



    }

    fn take_first_crates (&mut self){
        // Right side doesn't work well, so for use of left side
        self.robot_main = RobotSide::Right;

        self.asserv.goto_xya(self.kx * 1180.0, 1600.0,arfast(RobotSide::Back, TableSide::Up)).ok();
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

    fn return_to_start(&mut self){
        self.asserv.goto_a(arfast(RobotSide::Back, TableSide::Up)).ok();
        let start = self.pathfinder.nearest_node(&self.asserv.position().xy());
        let goal = self.pathfinder.nearest_node(&XY::new(self.kx*1200.0, 1700.0));
        if let Some(path) = self.pathfinder.find_path(start, goal) {
            let asserv_path: Vec<XY> = path.into_iter().map(|id| self.pathfinder.get_node_xy(id)).collect();
            self.asserv.run_path(&asserv_path).ok();
        } else {
            rome::warn!(self.rlogger, "Cannot find a path");
        }
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

    fn end_of_match(&mut self) {
        self.sensors.ground_lidar_power_off();
        self.meca.release(RobotSide::Back);
        self.meca.release(RobotSide::Left);
        self.meca.release(RobotSide::Right);
        loop {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}
