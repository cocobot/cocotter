use core::ops;

use canon_2019::hardware::{UHallPin, VHallPin, WHallPin};
use embassy_futures::select::select3;
use embassy_stm32::exti::ExtiInput;

use crate::motor_controller::MotorState;

//define all phases combinations
pub enum Phase {
    W,
    VW,
    V,
    UV,
    U,
    UW,
}

impl Phase {
    //compute phase from hall sensor pins
    fn from_pins(u : bool, v : bool, w: bool) -> Option<Phase> {
        if !u && !v && w {
            Some(Phase::W)
        }
        else if !u && v && w {
            Some(Phase::VW)
        }
        else if !u && v && !w {
            Some(Phase::V)
        }
        else if u && v && !w {
            Some(Phase::UV)
        }
        else if u && !v && !w {
            Some(Phase::U)
        }
        else if u && !v && w {
            Some(Phase::UW)
        }
        else {
            None
        }
    }

    //convert phase to number to perform arithmetics on it
    pub fn as_isize(&self) -> isize {
        match self {
            Phase::W => 0,
            Phase::VW => 1,
            Phase::V => 2,
            Phase::UV => 3,
            Phase::U => 4,
            Phase::UW => 5,
        }
    }

    //convert phase to string for debug output
    pub fn to_str(&self) -> &str {
        match self {
            Phase::W =>     "..W",
            Phase::VW =>    ".VW",
            Phase::V =>     ".V.",
            Phase::UV =>    "UV.",
            Phase::U =>     "U..",
            Phase::UW =>    "U.W",
        }
    }
}

//implement arithmetic substaction on phase
impl ops::Sub<Phase> for Phase {
    type Output = isize;

    fn sub(self, rhs: Phase) -> isize {
        self.as_isize() - rhs.as_isize()
    }
}

//RTOS task for decoding hall sensor inputs
#[embassy_executor::task]
pub async fn hall_sensor_decoder_task(
    mut u: ExtiInput<'static, UHallPin>,
    mut v: ExtiInput<'static, VHallPin>,
    mut w: ExtiInput<'static, WHallPin>,    
) {
    loop {
        //check if the current phase is decodable
        if let Some(phase) = Phase::from_pins(u.is_high(), v.is_high(), w.is_high()) {
            //get mutable motor instance
            let mut motor = MotorState::get_mutex().lock().await;

            //upadte phase
            motor.update_phase(phase);
        }        

        //prepare waiting futures
        let u_wait = u.wait_for_any_edge();
        let v_wait = v.wait_for_any_edge();
        let w_wait = w.wait_for_any_edge();

        //wait until any of the hall sensors change state
        select3(u_wait, v_wait, w_wait).await;
    }
}