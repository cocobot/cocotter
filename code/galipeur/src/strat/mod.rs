use std::{thread::sleep, time::Duration};
use board_common::Team;
use embedded_hal::digital::InputPin;
use board_sabotter::{SabotterBoard, SabotterInputs};
use flume::Sender;

use crate::led::LedMessage;

pub mod types;

pub struct Strat<B: SabotterBoard> {
    side: Team,
    leds: Sender<LedMessage>,

    inputs: SabotterInputs<B::ExInputPin, B::ExInputPin>,
}

impl<B : SabotterBoard + 'static> Strat<B> {
    pub fn init(board: &mut B, leds: Sender<LedMessage>) {
        let instance = Self {
            side: Team::None,
            leds,

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
    }


    //----------

    fn prepare_match(&mut self) {
        log::info!("Color selection");
        loop {
            let side = match self.inputs.color.is_high().unwrap_or(false) {
                true => Team::Left,
                false => Team::Right,
            };

            self.side = side;
            self.leds.send(LedMessage::GameSide { side }).ok();

            sleep(Duration::from_millis(100));

            if self.inputs.starter.is_low().unwrap_or(false) {
                log::info!("Color selected");                
                break;
            }
        }

        log::info!("Set meca in init position");
        //self.meca.prepare();
        

        log::warn!("Todo: recallage");
    }
    
}