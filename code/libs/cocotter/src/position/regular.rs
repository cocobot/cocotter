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
    timestamp_ms : u64,

    //robot position
    current_position : RobotCoordinate,

    //extra position for regular robot
    distance_mm : f32,
    angle_rad : f32,
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
            timestamp_ms : 0,
            current_position: RobotCoordinate::from(0.0, 0.0, 0.0), 
            distance_mm: 0.0, 
            angle_rad : 0.0,
            distance_velocity_mm_s: 0.0, 
            angle_velocity_rad_s: 0.0, 

        }
    }

    //compute new position from updated encoder values
    pub fn set_new_encoder_values(&mut self, timestamp_ms: u64, left_encoder: i32, right_encoder: i32) {
        
        if timestamp_ms != self.timestamp_ms
        {
            let delta_tick_left_encoder  : f32 = (left_encoder  - self.left_encoder) as f32;
            let delta_tick_right_encoder : f32 = (right_encoder - self.right_encoder) as f32;

            let delta_distance_mm = (delta_tick_left_encoder + delta_tick_right_encoder)/2.0 / self.configuration.tick_to_mm;
            let delta_angle_rad = (delta_tick_right_encoder - delta_tick_left_encoder) / self.configuration.tick_to_rad;
            self.distance_mm += delta_distance_mm;
            self.angle_rad += delta_angle_rad;

            let delta_time_ms : f32 = (timestamp_ms - self.timestamp_ms) as f32;
            
            self.distance_velocity_mm_s = delta_distance_mm / (delta_time_ms / 1000.0);
            self.angle_velocity_rad_s = delta_angle_rad / (delta_time_ms / 1000.0);

            self.left_encoder = left_encoder;
            self.right_encoder = right_encoder;
            self.timestamp_ms = timestamp_ms;

            self.current_position.compute_new_position(delta_distance_mm, delta_angle_rad);
        }
    }

    //get the current curvilinear distance in mm
    pub fn get_distance_mm(&self) -> f32 {
        self.distance_mm
    }

    //get the current curvilinear distance velocity in mm/s
    pub fn get_distance_speed_mm_s(&self) -> f32 {
        self.distance_velocity_mm_s
    }

    //get the current curvilinear angle in rad
    pub fn get_angle_rad(&self) -> f32 {
        self.angle_rad
    }
}

//general functions implementation
impl Position for RegularPosition {
    //Return the current robot position
    fn get_coordinates(&self) -> RobotCoordinate {
        self.current_position.clone()
    }

    //set one or more coordinate and mark them as precise
    fn set_coordinates(&mut self, x_mm : Option<f32>, y_mm : Option<f32>, a_rad : Option<f32>) {
        self.current_position.set_position(x_mm, y_mm, a_rad);        
    }

    //set position as imprecise
    fn mark_position_as_imprecise(&mut self) {
        self.current_position.mark_x_as_imprecise();
        self.current_position.mark_y_as_imprecise();
        self.current_position.mark_a_as_imprecise();
    }

    //get the current angle velocity in rad/s
    fn get_angle_speed_rad_s(&self) -> f32 {
        self.angle_velocity_rad_s
    }

    //get the current angle velocity in degree/s
    fn get_angle_speed_deg_s(&self) -> f32 {
        self.get_angle_speed_rad_s() * 180.0 / std::f32::consts::PI
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG : RegularPositionConfiguration = RegularPositionConfiguration {
        tick_to_mm: 1000000.0,
        tick_to_rad: 1000000.0,
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
        let mut diff;

        //set initial encoder values
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);

        //check initial coordininates
        let mut coordinates: RobotCoordinate = position.get_coordinates();
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
        left_encoder    += 5.0 * CONFIG.tick_to_mm ;
        right_encoder   += 5.0 * CONFIG.tick_to_mm ;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 5.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(position.get_distance_mm(), 5.0);
        assert_eq!(position.get_distance_speed_mm_s(), 10.0);
        assert_eq!(position.get_angle_speed_rad_s(), 0.0);       
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        
        //turn -90° in 500ms
        timestamp       += 500;
        left_encoder    += std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        right_encoder   -= std::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        coordinates = position.get_coordinates();
        assert_eq!(position.get_distance_mm(), 5.0);
        diff = position.get_angle_rad() - (-std::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        diff = coordinates.get_x_mm() - 5.0;
        assert!(diff.abs()<0.001, "x not equal to 5.0 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        diff = coordinates.get_a_rad()- (-std::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(position.get_distance_speed_mm_s(), 0.0);
        diff = position.get_angle_speed_rad_s()- (-std::f32::consts::PI);
        assert!(diff.abs()<0.001, "angle speed not equal to -PI : diff to expected value :  {}", diff);

        //go 5mm forward in 1000ms
        timestamp       += 1000;
        left_encoder    += 5.0 * CONFIG.tick_to_mm;
        right_encoder   += 5.0 * CONFIG.tick_to_mm;
        position.set_new_encoder_values(timestamp, left_encoder as i32, right_encoder as i32);
        coordinates = position.get_coordinates();
        assert_eq!(position.get_distance_mm(), 10.0);
        diff = position.get_angle_rad() - (-std::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        diff = coordinates.get_x_mm() - 5.0;
        assert!(diff.abs()<0.001, "x not equal to 5.0 : diff to expected value :  {}", diff);
        diff = coordinates.get_y_mm() - (-5.0);
        assert!(diff.abs()<0.001, "y not equal to 5.0 : diff to expected value :  {}", diff);
        diff = coordinates.get_a_rad()- (-std::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
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
        let mut read_val = position.get_angle_speed_rad_s();
        let mut expected_val = std::f32::consts::PI;
        assert!( (read_val - expected_val).abs() < 0.001, "radian : new angle speed error: read {}, expected : {}", read_val, expected_val);
        read_val = position.get_angle_speed_deg_s();
        expected_val *= 180.0 / std::f32::consts::PI;
        assert!( (read_val - expected_val).abs() < 0.001, "degree : new angle speed error: read {}, expected : {}", read_val, expected_val);
    }
}