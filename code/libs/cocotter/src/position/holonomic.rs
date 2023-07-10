use super::{robot_coordinate::RobotCoordinate, Position};

pub struct HolonomicPositionConfiguration {
    tick_to_mm : f32,
    tick_to_rad : f32,
}

//create a strucut for regular robot position decoder
pub struct HolonomicPosition {
    //configuration
    configuration: HolonomicPositionConfiguration, 

    //encoder data
    encoders: [i32; 3],

    //robot position
    current_position : RobotCoordinate,
    angle_velocity_rad_s : f32,

    //extra position for regular robot
    velocity_mm_s : [f32; 2],
}

//Specific implementation of a regular robot
impl HolonomicPosition {
    pub fn new(configuration: HolonomicPositionConfiguration) -> HolonomicPosition {
        RegularPosition { 
            configuration, 
            encoders: [0, 0, 0], 
            current_position: RobotCoordinate::from(0.0, 0.0, 0.0), 
            velocity_mm_s: [0.0, 0.0], 
            angle_velocity_rad_s: 0.0, 
        }
    }

    //compute new position from updated encoder values
    pub fn set_new_encoder_values(&mut self, timestamp_ms: u64, encoders: [i32; 3]) {
        unimplemented!()
    }

    //get the velocity vector in mm/s
    pub fn get_speed_mm_s(&self) -> [f32; 2] {
        unimplemented!()
    }
}

//general functions implementation
impl Position for HolonomicPosition {
    //Return the current robot position
    fn get_coordinates(&self) -> RobotCoordinate {
        unimplemented!()
    }

    //set one or more coordinate and mark them as precise
    fn set_coordinates(&mut self, x_mm : Option<f32>, y_mm : Option<f32>, a_rad : Option<f32>) {
        unimplemented!()
    }

    //set position as imprecise
    fn mark_position_as_imprecise(&mut self) {
        unimplemented!()
    }

    //get the current angle velocity in rad/s
    fn get_angle_speed_rad_s(&self) -> f32 {
        unimplemented!()
    }

    //get the current angle velocity in degree/s
    fn get_angle_speed_deg_s(&self) -> f32 {
        unimplemented!()
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn todo_holonmic_unit_tests() {
        assert_eq!(false, true);
    }
}