use std::{sync::{Arc, Mutex}, time::Instant};

use crate::ramp::{Ramp, RampConfiguration};

use self::robot_coordinate::RobotCoordinate;

pub mod robot_coordinate;

pub type PositionMutex<const N: usize> = Arc<Mutex<Position<N>>>;

pub struct PositionConfiguration {
    pub tick_to_mm : f32,
    pub tick_to_rad : f32,
}

pub struct Position<const N: usize> {
    //configuration
    configuration: PositionConfiguration, 

    //encoder data
    encoders : [i32; N],
    timestamp_ms : Instant,

    //robot position
    current_position : RobotCoordinate<N>,

    //robot asserv
    ramps : [Ramp; N],

    no_angle: bool,
}

//Specific implementation of a regular robot
impl<const N: usize> Position<N> {
    pub fn new(configuration: PositionConfiguration, ramp_configuration: [RampConfiguration; N]) -> Position<N> {
        let ramps: [Ramp; N] = core::array::from_fn(|i| Ramp::new(ramp_configuration[i]));

        Position { 
            configuration, 
            encoders: [0; N],
            timestamp_ms : Instant::now(),
            current_position: RobotCoordinate::from(0.0, 0.0, 0.0),
            ramps,
            no_angle: false,
        }
    }

    //compute new position from updated encoder values
    pub fn set_new_encoder_values(&mut self, timestamp_ms: Instant, encoders: [i32; N], gain: [f32; N]) {
        
        if timestamp_ms != self.timestamp_ms
        {
            let mut delta_tick_encoder = [0.0; N];
            for i in 0..N {
                delta_tick_encoder[i] = (encoders[i] - self.encoders[i]) as f32;
            }

            let delta_time_ms : f32 = (timestamp_ms - self.timestamp_ms).as_millis() as f32;
            let mut delta_computation = [0.0; N];
            match N {
                2 => {
                    //argument 0 is the angle in rad, argument 1 is the linear distance in
                    delta_computation[0] = ((encoders[1] as f32) * gain[1] - (encoders[0] as f32) * gain[0]) / self.configuration.tick_to_rad;
                    delta_computation[1] = (delta_tick_encoder[0] + delta_tick_encoder[1]) / 2.0 / self.configuration.tick_to_mm; 
                },
                _ => {
                    unimplemented!("Only 2 encoders are supported");
                }
            }

            self.encoders = encoders;
            self.timestamp_ms = timestamp_ms;

            //log::info!("delta_tick_encoder : {:?} delta_time_ms : {} delta_computation : {:?}", delta_tick_encoder, delta_time_ms, delta_computation);

            self.current_position.compute_new_position(delta_computation, delta_time_ms);
        }
    }

    pub fn set_no_angle(&mut self, no_angle: bool) {
        self.no_angle = no_angle;
    }

    pub fn get_no_angle(&self) -> bool {
        self.no_angle
    }

     //Return the current robot position
    pub fn get_coordinates(&self) -> &RobotCoordinate<N> {
        &self.current_position
    }

    //Return the ramp array instance
    pub fn get_ramps(&self) -> &[Ramp; N] {
        &self.ramps
    }

    //Return the ramp array instance as mutable
    pub fn get_ramps_as_mut(&mut self) -> &mut [Ramp; N] {
        &mut self.ramps
    }

    //set one or more coordinate and mark them as precise
    pub fn set_coordinates(&mut self, x_mm : Option<f32>, y_mm : Option<f32>, a_rad : Option<f32>) {
        self.current_position.set_position(x_mm, y_mm, a_rad);        
    }

    //set position as imprecise
    pub fn mark_position_as_imprecise(&mut self) {
        self.current_position.mark_x_as_imprecise();
        self.current_position.mark_y_as_imprecise();
        self.current_position.mark_a_as_imprecise();
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG : PositionConfiguration = PositionConfiguration {
        tick_to_mm: 1000000.0,
        tick_to_rad: 1000000.0,
    };

    const RAMP_CONFIG : [RampConfiguration; 2] = [
        RampConfiguration {
            acceleration: 1.0,
            max_speed: 1.0,
        },
        RampConfiguration {
            acceleration: 1.0,
            max_speed: 1.0,
        }
    ];

    #[test]
    fn check_coordinates_overwrite() {
        let mut position = Position::<2>::new(CONFIG,RAMP_CONFIG);

        //check initial coordininates
        let coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);

        //set imprecise flag
        position.mark_position_as_imprecise();
        let coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), false);
        assert_eq!(coordinates.is_y_precise(), false);
        assert_eq!(coordinates.is_a_precise(), false);

        //set x coordinate
        position.set_coordinates(Some(1.0), None, None);
        let coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), false);
        assert_eq!(coordinates.is_a_precise(), false);

        //set y coordinate
        position.set_coordinates(None, Some(2.0), None);
        let coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 2.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), false);

        //set y coordinate
        position.set_coordinates(None, None, Some(3.0));
        let coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 1.0);
        assert_eq!(coordinates.get_y_mm(), 2.0);
        assert_eq!(coordinates.get_a_rad(), 3.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
    }

    #[test]
    fn check_position_computation() {
        let mut position = Position::<2>::new(CONFIG,RAMP_CONFIG);
        let mut timestamp = 0;
        let mut left_encoder = 0.0;
        let mut right_encoder = 0.0;
        let mut diff;

        //set initial encoder values
        position.set_new_encoder_values(timestamp, [left_encoder as i32, right_encoder as i32]);

        //check initial coordininates
        let mut coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 0.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(coordinates.get_distance_mm(), 0.0);
        assert_eq!(coordinates.get_distance_speed_mm_s(), 0.0);
        assert_eq!(coordinates.get_angle_speed_rad_s(), 0.0);
        

        //go 5mm forward in 500ms
        timestamp       += 500;
        left_encoder    += 5.0 * CONFIG.tick_to_mm ;
        right_encoder   += 5.0 * CONFIG.tick_to_mm ;
        position.set_new_encoder_values(timestamp, [left_encoder as i32, right_encoder as i32]);
        coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_x_mm(), 5.0);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        assert_eq!(coordinates.get_a_rad(), 0.0);
        assert_eq!(coordinates.get_distance_mm(), 5.0);
        assert_eq!(coordinates.get_distance_speed_mm_s(), 10.0);
        assert_eq!(coordinates.get_angle_speed_rad_s(), 0.0);       
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        
        //turn -90° in 500ms
        timestamp       += 500;
        left_encoder    += core::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        right_encoder   -= core::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0;
        position.set_new_encoder_values(timestamp, [left_encoder as i32, right_encoder as i32]);
        coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_distance_mm(), 5.0);
        diff = coordinates.get_angle_rad() - (-core::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        diff = coordinates.get_x_mm() - 5.0;
        assert!(diff.abs()<0.001, "x not equal to 5.0 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.get_y_mm(), 0.0);
        diff = coordinates.get_a_rad()- (-core::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(coordinates.get_distance_speed_mm_s(), 0.0);
        diff = coordinates.get_angle_speed_rad_s()- (-core::f32::consts::PI);
        assert!(diff.abs()<0.001, "angle speed not equal to -PI : diff to expected value :  {}", diff);

        //go 5mm forward in 1000ms
        timestamp       += 1000;
        left_encoder    += 5.0 * CONFIG.tick_to_mm;
        right_encoder   += 5.0 * CONFIG.tick_to_mm;
        position.set_new_encoder_values(timestamp, [left_encoder as i32, right_encoder as i32]);
        coordinates = position.get_coordinates();
        assert_eq!(coordinates.get_distance_mm(), 10.0);
        diff = coordinates.get_angle_rad() - (-core::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        diff = coordinates.get_x_mm() - 5.0;
        assert!(diff.abs()<0.001, "x not equal to 5.0 : diff to expected value :  {}", diff);
        diff = coordinates.get_y_mm() - (-5.0);
        assert!(diff.abs()<0.001, "y not equal to 5.0 : diff to expected value :  {}", diff);
        diff = coordinates.get_a_rad()- (-core::f32::consts::FRAC_PI_2);
        assert!(diff.abs()<0.001, "angle not equal to -PI/2 : diff to expected value :  {}", diff);
        assert_eq!(coordinates.is_x_precise(), true);
        assert_eq!(coordinates.is_y_precise(), true);
        assert_eq!(coordinates.is_a_precise(), true);
        assert_eq!(coordinates.get_distance_speed_mm_s(), 5.0);
        assert_eq!(coordinates.get_angle_speed_rad_s(), 0.0);
    }

    #[test]
    fn check_degree_conversion() {
        let mut position = Position::<2>::new(CONFIG,RAMP_CONFIG);

        //set initial encoder values
        position.set_new_encoder_values(0, [0, 0]);

        //turn 90° in 500ms
        position.set_new_encoder_values(500, [(-core::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0) as i32, (core::f32::consts::FRAC_PI_2 * CONFIG.tick_to_rad / 2.0) as i32]);

        //check velocity
        let coordinates = position.get_coordinates();
        let mut read_val = coordinates.get_angle_speed_rad_s();
        let mut expected_val = core::f32::consts::PI;
        assert!( (read_val - expected_val).abs() < 0.001, "radian : new angle speed error: read {}, expected : {}", read_val, expected_val);
        read_val = coordinates.get_angle_speed_deg_s();
        expected_val *= 180.0 / core::f32::consts::PI;
        assert!( (read_val - expected_val).abs() < 0.001, "degree : new angle speed error: read {}, expected : {}", read_val, expected_val);
    }
}