enum TrajectoryOrderError {
    NotEnoughMemory                               //Not enough memory to store the trajectory checkpoint
}

enum TrajectoryError {
    ObstacleDetected,                             //Obstacle detected during the trajectory
    Timeout,                                      //End of the trajectory not reached in time
}

enum TrajectoryInterruptMode {
    Wait {timeout_ms: Option<u32>},               //Robot wait until the obstacle is removed
    Abort,                                        //Robot abort the trajectory
}

//Shared function of a trajectory manager structure
pub trait Trajectory {
    //change the action when an obstacle is detected
    fn set_interrupt_mode(&mut self, mode: TrajectoryInterruptMode);

    //set a new target for the robot absolute angle
    fn goto_a(&mut self) -> Result<(), TrajectoryOrderError>;

    //set a new target for the robot absolute position
    fn goto_xya(&mut self, x_mm : f32, y_mm: f32, a_rad: f32) -> Result<(), TrajectoryOrderError>;

    //wait for the end of the current trajectory
    fn wait_end(&mut self, timeout_ms: Option<u32>) -> Result<(), TrajectoryError>;
}

// >>> specific
// goto_d
// set forward/backward