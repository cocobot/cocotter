use self::robot_coordinate::PositionRobotCoordinate;

pub mod robot_coordinate;

pub trait Position {
    //Return the current robot position
    fn get_coordinates(&self) -> PositionRobotCoordinate;
}
