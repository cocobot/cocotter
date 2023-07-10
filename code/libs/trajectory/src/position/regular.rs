use super::{robot_coordinate::RobotCoordinate, Position};

pub struct RegularPositionConfiguration {
    tick_to_mm : f32,
    tick_to_rad : f32,
}

//create a strucut for regular robot position decoder
pub struct RegularPosition {
    //configuration
    configuration: RegularPositionConfiguration, 

    //encoder data
    left_encoder : i32,
    right_encoder : i32,

    //robot position
    current_position : RobotCoordinate,

    //extra position for regular robot
    distance_mm : f32,
    distance_velocity_mm_s : f32,
    angle_velocity_rad_s : f32,
}

//Specific implementation of a regular robot
impl RegularPosition {
    pub fn new(configuration: RegularPositionConfiguration) -> RegularPosition {
        RegularPosition { 
            configuration, 
            left_encoder: 0, 
            right_encoder: 0, 
            current_position: RobotCoordinate::from(0.0, 0.0, 0.0), 
            distance_mm: 0.0, 
            distance_velocity_mm_s: 0.0, 
            angle_velocity_rad_s: 0.0, 
        }
    }

    pub fn set_new_encoder_values(&mut self, timestamp_ms: u64, left_encoder: i32, right_encoder: i32) {
        unimplemented!()
    }

    //get the current curvilinear distance in mm
    pub fn get_distance_mm(&self) -> f32 {
        unimplemented!()
    }

    //get the current curvilinear distance velocity in mm/s
    pub fn get_distance_speed_mm_s(&self) -> f32 {
        unimplemented!()
    }
}

//general functions implementation
impl Position for RegularPosition {
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

    const CONFIG : RegularPositionConfiguration = RegularPositionConfiguration {
        tick_to_mm: 10.0,
        tick_to_rad: 100.0,
    };

    #[test]
    fn check_coordinates_overwrite() {
        let mut position = RegularPosition::new(CONFIG);

        //check initial coordininates
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);

        //set imprecise flag
        position.mark_position_as_imprecise();
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), false);
        assert_eq!(coordinates.is_y_precise(), false);
        assert_eq!(coordinates.is_a_precise(), false);

        //set x coordinate
        position.set_coordinates(Some(1.0), None, None);
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), false);
        assert_eq!(coordinates.is_a_precise(), false);

        //set y coordinate
        position.set_coordinates(None, Some(2.0), None);
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 2.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), false);

        //set y coordinate
        position.set_coordinates(None, None, Some(3.0));
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 2.0);
        assert_eq!(coordinates.get_a_rad(), 3.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
    }

    #[test]
    fn check_position_computation() {
        let mut position = RegularPosition::new(CONFIG);
        let mut timestamp = 0;
        let mut left_encoder = 0.0;
        let mut right_encoder = 0.0;

        //set initial encoder values
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);

        //check initial coordininates
        let coordinates: RobotCoordinate = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(position.get_distance_mm(), 0.0);
        assert_eq!(position.get_distance_speed_mm_s(), 0.0);
        assert_eq!(position.get_angle_speed_rad_s(), 0.0);
        

        //go 5mm forward in 500ms
        timestamp       += 500;
        left_encoder    += 5.0 * CONFIG.tick_to_mm / 2.0;
        right_encoder   += 5.0 * CONFIG.tick_to_mm / 2.0;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        assert_eq!(coordinates.get_x_mm(), 5.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(position.get_distance_mm(), 5.0);
        assert_eq!(position.get_distance_speed_mm_s(), 10.0);
        assert_eq!(position.get_angle_speed_rad_s(), 0.0);

        //turn -90° in 500ms
        timestamp       += 500;
        left_encoder    += std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        right_encoder   -= std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        assert_eq!(coordinates.get_x_mm(), 5.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), std::f32::consts::FRAC_PI_2);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(position.get_distance_mm(), 5.0);
        assert_eq!(position.get_distance_speed_mm_s(), 0.0);
        assert_eq!(position.get_angle_speed_rad_s(), -std::f32::consts::PI);

        //go 5mm forward in 1000ms
        timestamp       += 1000;
        left_encoder    += 5.0 * CONFIG.tick_to_mm / 2.0;
        right_encoder   += 5.0 * CONFIG.tick_to_mm / 2.0;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        assert_eq!(coordinates.get_x_mm(), 5.0);
        assert_eq!(coordinates.get_y_mm(), 5.0);
        assert_eq!(coordinates.get_a_rad(), std::f32::consts::FRAC_PI_2);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(position.get_distance_mm(), 10.0);
        assert_eq!(position.get_distance_speed_mm_s(), 5.0);
        assert_eq!(position.get_angle_speed_rad_s(), 0.0);
    }

    #[test]
    fn check_degree_conversion() {
        let mut position = RegularPosition::new(CONFIG);

        //set initial encoder values
        position.set_new_encoder_values(0, 0, 0);

        //turn 90° in 500ms
        position.set_new_encoder_values(500, (-std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0) as i32, (std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0) as i32);

        //check velocity
        assert_eq!(position.get_angle_speed_rad_s(), std::f32::consts::PI);
        assert_eq!(position.get_angle_speed_deg_s(), 90.0);
    }
}