//define update setpoint error
pub enum MotorControlUpdateError {
    InvalidMotorId,             //returned if the current motor control structure does not support the selected motor
    CommunicationError,
}


//Interface of a motor control output
pub trait MotorControl {
    //modify setpoint
    fn update_setpoint(motor_id: usize, position_target: i32, velocity_target: i32) -> Result<(), MotorControlUpdateError>;
}