use std::sync::mpsc::Sender;

pub enum GameControllerEvent {
    Connected,
    Disconnected,
} 

struct GameController {
    event : Sender<GameControllerEvent>,
}

impl GameController {
    fn new(event : Sender<GameControllerEvent>) -> Self {
        GameController {
            event,
        }
    }
}