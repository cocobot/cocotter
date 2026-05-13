use asserv::maths::XY;

pub const MATCH_DURATION: f32 = 120.0;

/// Compact world state for the planner search.
///
/// Cloned at each search node — kept small and cheap to copy.
#[derive(Clone, Debug)]
pub struct WorldState {
    /// Current robot position on the table (mm).
    pub position: XY,
    /// Seconds remaining in the match.
    pub time_remaining: f32,
    /// Points locked in (scored regardless of match end).
    pub secured_points: i32,
    /// Points currently "in flight" (e.g. crates held, lost if match ends).
    pub potential_points: i32,
    /// Generic bitflags (e.g. bit 3 = "spot 3 already taken").
    pub flags: u64,
    /// Generic counters (e.g. counter 0 = total crates held on robot).
    pub counters: [isize; 1],
}

impl WorldState {
    /// Evaluate the state's expected score.
    ///
    /// Potential points devalue linearly with time: worth 100% at t=120s,
    /// 0% at t=0s. This naturally prioritises securing points as time runs out.
    pub fn score(&self) -> f32 {
        let time_factor = (self.time_remaining / MATCH_DURATION).clamp(0.0, 1.0);
        self.secured_points as f32 + self.potential_points as f32 * time_factor
    }

    pub fn has_flag(&self, bit: u8) -> bool {
        self.flags & (1u64 << bit) != 0
    }

    pub fn set_flag(&mut self, bit: u8) {
        self.flags |= 1u64 << bit;
    }

    pub fn clear_flag(&mut self, bit: u8) {
        self.flags &= !(1u64 << bit);
    }
}
