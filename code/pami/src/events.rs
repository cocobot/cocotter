pub use board_common::Team;
pub use board_pami::DpadState;


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PamiRole {
    Granary,
    Land,
}

/// Event to sent to UI to update its state
//TODO UiUpdateEvent
pub enum UiEvent {
    Battery { percent: u8 },
    EmergencyStop(bool),
    Dpad(DpadState),
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


/// Steps for the main routine
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MainStep {
    /// Selecting team or waiting to unplug cord
    Free,
    /// Starting cord plugged in
    CordPlugged,
    /// Match started
    Match,
}

