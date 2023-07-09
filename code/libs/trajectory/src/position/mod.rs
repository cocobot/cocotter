use self::robot_coordinate::RobotCoordinate;

pub mod robot_coordinate;
pub mod regular;

//Shared function of a position manager structure
pub trait Position {
    //Return the current robot position
    fn get_coordinates(&self) -> RobotCoordinate;

    //set one or more coordinate and mark them as precise
    fn set_coordinates(&mut self, x_mm : Option<f32>, y_mm : Option<f32>, a_rad : Option<f32>);

    //set position as imprecise
    fn mark_position_as_imprecise(&mut self);

    //get the current angle velocity in rad/s
    fn get_angle_speed_rad_s(&self) -> f32;

    //get the current angle velocity in degree/s
    fn get_angle_speed_deg_s(&self) -> f32;
}