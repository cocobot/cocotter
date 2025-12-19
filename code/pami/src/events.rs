use asserv::maths::XYA;
use board_common::Team;
use board_pami::DpadState;


#[derive(Clone, Copy, PartialEq, Eq)]
pub enum UiPamiMode {
    Match,
    QuickStart,
    Debug,
}

/// Event to sent to UI to update its state
//TODO UiUpdateEvent
pub enum UiEvent {
    Battery { percent: u8 },
    EmergencyStop(bool),
    Dpad(DpadState),
    KeypassNotif(u32), 
    ChangeTeam(Team),
    ChangeMode(UiPamiMode),
}

/// Event triggered by the UI, usually in response to user actions
pub enum UiTrigger {
    ChangeTeam(Team),
    ChangeMode(UiPamiMode),
    Reboot,
}


pub enum AsservOrder {
    GotoXy(f32, f32),
    GotoA(f32),
    GotoXyRel(f32, f32),
    GotoARel(f32),
    ResetPosition(XYA),
    //TODO Configuration
}

