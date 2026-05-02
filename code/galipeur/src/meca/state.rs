use board_common::Team;

#[derive(Default)]
pub struct MecaSideState {
    ready_to_take: bool,
    lower_stage: [Team; 4],
    upper_stage: [Team; 4],
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

    pub fn transfer_to_lower_stage(&mut self) {
        self.lower_stage = std::mem::take(&mut self.upper_stage);
    }

    pub fn ready_to_take(&mut self, ready: bool) {
        self.ready_to_take = ready;        
    }

    pub fn is_ready_to_take(&self) -> bool {
        self.ready_to_take
    }

    pub fn set_lower_stage(&mut self, teams: [Team; 4]) -> [Team; 4] {
        std::mem::replace(&mut self.lower_stage, teams)
    }
}

#[derive(Default)]
pub struct MecaState {
    sides: [MecaSideState; 3],
}

impl MecaState {
    pub fn get_side(&self, side: u8) -> &MecaSideState {
        &self.sides[side as usize]
    }

    pub fn get_side_mut(&mut self, side: u8) -> &mut MecaSideState {
        &mut self.sides[side as usize]
    }
}
