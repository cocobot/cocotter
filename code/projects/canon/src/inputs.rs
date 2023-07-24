use canon_2019::hardware::{StepPin, DirPin};
use embassy_stm32::{exti::ExtiInput, gpio::Input};

use crate::motor_controller::MotorState;


//RTOS task for decoding step and direction inputs
#[embassy_executor::task]
pub async fn input_decoder_task(
    mut step: ExtiInput<'static, StepPin>,
    dir: Input<'static, DirPin>
) {
    loop {
        //wait until stat board requests a motor step
        step.wait_for_rising_edge().await;

        //get direction
        let forward = dir.is_high();

        //update motor state
        let mut motor = MotorState::get_mutex().lock().await;
        motor.step_requested(forward);
        drop(motor);
    }
}


