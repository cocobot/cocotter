use simple_logger::SimpleLogger;

pub struct FakeOutputPin {
}


impl FakeOutputPin {
    pub fn new() -> Self {
        Self {}
    }

    pub fn toggle(&mut self) -> Result<(), ()> {
        println!("toggle");
        Ok(())
    }
}

pub type LedHeartbeat = FakeOutputPin;

pub struct BoardPami {
    pub led_heartbeat: Option<LedHeartbeat>,
}

impl BoardPami {
    pub fn new() -> Self {
        SimpleLogger::new().init().unwrap();

        Self {
            led_heartbeat: Some(FakeOutputPin::new()),
        }
    }
}
pub struct BoardSabotter {
    pub led_heartbeat: Option<LedHeartbeat>,
}

impl BoardSabotter {
    pub fn new() -> Self {
        SimpleLogger::new().init().unwrap();

        Self {
            led_heartbeat: Some(FakeOutputPin::new()),
        }
    }
}