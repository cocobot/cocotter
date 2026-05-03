use board_common::Team;

pub type MecaState = [MecaSideState; 3];

#[derive(Default, Clone)]
pub struct MecaSideState {
    pub ready_to_take: bool,
    pub upper_stage_up: bool,
    pub lower_stage: [Team; 4],
    pub upper_stage: [Team; 4],
}

impl MecaSideState {
    pub fn is_lower_stage_empty(&self) -> bool {
        self.lower_stage.iter().all(|&t| t == Team::None)
    }

    pub fn is_upper_stage_empty(&self) -> bool {
        self.upper_stage.iter().all(|&t| t == Team::None)
    }

    pub fn transfer_to_clamp(&mut self) {
        self.upper_stage = std::mem::take(&mut self.lower_stage);
    }

    pub fn transfer_to_lower_stage(&mut self) -> [Team; 4]{
        self.lower_stage = std::mem::take(&mut self.upper_stage);
    
        self.lower_stage
    }

    pub fn ready_to_take(&mut self, ready: bool) {
        self.ready_to_take = ready;
        log::info!("READY TO TAKE SET TO {}", ready);
    }

    pub fn is_ready_to_take(&self) -> bool {
        self.ready_to_take
    }

    pub fn upper_stage_up(&mut self, up: bool) {
        self.upper_stage_up = up;
        log::info!("Set clamp up {}", up)
    }

    pub fn is_upper_stage_up(&self) -> bool {
        self.upper_stage_up
    }

    pub fn set_lower_stage(&mut self, teams: [Team; 4]) -> [Team; 4] {
        std::mem::replace(&mut self.lower_stage, teams)
    }
}
