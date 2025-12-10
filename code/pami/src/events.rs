use std::time::{Duration, Instant};
use asserv::maths::XYA;
use board_pami::DpadState;


pub struct Periodicity {
    next: Instant,
    period: Duration,
}

impl Periodicity {
    pub fn new(period: Duration) -> Self {
        Self {
            next: Instant::now(),
            period,
        }
    }

    pub fn update(&mut self, instant: &Instant) -> bool {
        if instant >= &self.next {
            self.next = *instant + self.period;
            true
        } else {
            false
        }
    }

    pub fn next(&self) -> &Instant {
        &self.next
    }

    pub fn period(&self) -> &Duration {
        &self.period
    }
}


pub enum UiEvent {
    Battery { percent: u8 },
    Dpad(DpadState),
    KeypassNotif(u32), 
}


pub enum AsservOrder {
    GotoXy(f32, f32),
    GotoA(f32),
    GotoXyRel(f32, f32),
    GotoARel(f32),
    ResetPosition(XYA),
    //TODO Configuration
}

