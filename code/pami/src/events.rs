pub use board_common::Team;
pub use board_pami::DpadState;


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PamiRole {
    None,
    Granary,
    Land,
}

/// Event to sent to UI to update its state
//TODO UiUpdateEvent
pub enum UiEvent {
    Battery { percent: u8 },
    EmergencyStop(bool),
    Dpad(DpadState),
    ShowMessage(&'static str),
    ShowMatchConf(MatchConf),
    KeypassNotif(u32), 
    ChangeTeam(Team),
    ChangeStartDelay(u8),
    ChangeRole(PamiRole),
}

/// Event triggered by the UI, usually in response to user actions
pub enum UiTrigger {
    ChangeTeam(Team),
    ChangeStartDelay(u8),
    ChangeRole(PamiRole),
}


/// Match configuration
#[derive(Clone)]
pub struct MatchConf {
    pub team: Team,
    pub start_delay: u8,
    pub role: PamiRole,
}

