use board_common::Team;

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
        self.upper_stage.copy_from_slice(&self.lower_stage);
        self.lower_stage.fill(Team::None);
    }

    pub fn transfer_to_lower_stage(&mut self) {
        self.lower_stage.copy_from_slice(&self.upper_stage);
        self.upper_stage.fill(Team::None);
    }

    pub fn ready_to_take(&mut self, ready: bool) {
        self.ready_to_take = ready;        
    }

    pub fn is_ready_to_take(&self) -> bool {
        self.ready_to_take
    }

    pub fn set_lower_stage(&mut self, teams: [Team; 4]) -> [Team; 4] {
        let previous_data = self.lower_stage;
        self.lower_stage.copy_from_slice(&teams);

        previous_data
    }
}

pub struct MecaState {
    sides: [MecaSideState; 3],
    own_color: Team,
}

impl Default for MecaState {
    fn default() -> Self {
        Self {
            own_color: Team::None,
            sides: [
                MecaSideState { lower_stage: [Team::None; 4], upper_stage: [Team::None; 4], ready_to_take: false},
                MecaSideState { lower_stage: [Team::None; 4], upper_stage: [Team::None; 4], ready_to_take: false},
                MecaSideState { lower_stage: [Team::None; 4], upper_stage: [Team::None; 4], ready_to_take: false},
            ],
        }
    }
}

impl MecaState {
    pub fn get_side_state(&self, side: u8) -> &MecaSideState {
        &self.sides[side as usize]
    }

    pub fn get_side_state_mut(&mut self, side: u8) -> &mut MecaSideState {
        &mut self.sides[side as usize]
    }

    pub fn set_own_color(&mut self, color: Team) {
        self.own_color = color;
    }

    pub fn get_own_color(&self) -> Team {
        self.own_color
    }
}