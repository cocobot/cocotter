use core::f32;

use libm::{cosf, sinf};

#[derive(Clone, Debug, Copy)]
//Create an holder for current robot coordinates
pub struct RobotCoordinate<const N: usize> {
    //store robot position in standardized unit (mm/rad)
    x_mm : f32,                 
    y_mm : f32,

    //store integrated coordinate
    //first element is always the angle in rad
    //others are coordonate of the distance vector in mm 
    linear_coordonate : [f32; N],
    linear_coordonate_offset: [f32; N],
    linear_velocity : [f32; N],

    //store if axis has been recalibrated recentrly
    x_is_precise : bool,
    y_is_precise : bool,
    a_is_precise : bool,
}

impl<const N: usize> RobotCoordinate<N> {
    //create a new structure from independant data
    pub fn from(x_mm : f32, y_mm : f32, a_rad : f32) -> RobotCoordinate<N> {
        let mut linear_coordonate = [0.0; N];
        linear_coordonate[0] = a_rad;

        RobotCoordinate { 
            x_mm, 
            y_mm, 
            
            linear_coordonate: linear_coordonate, 
            linear_velocity: [0.0; N],
            linear_coordonate_offset: [0.0; N],
            
            x_is_precise: true, 
            y_is_precise: true, 
            a_is_precise: true}
    }

    //return the x position only
    pub fn get_x_mm(&self) -> f32 {
        self.x_mm
    }

    //return the y position only
    pub fn get_y_mm(&self) -> f32 {
        self.y_mm
    }

    //return the angle position only (radians)
    pub fn get_a_rad(&self) -> f32 {
        self.linear_coordonate[0]
    }

    //return the angle position only (degrees)
    pub fn get_a_deg(&self) -> f32 {
        self.get_a_rad().to_degrees()
    }

    pub fn get_raw_linear_coordonate(&self) -> [f32; N] {
        self.linear_coordonate
    }

    //sets the new position
    pub fn set_position(&mut self, x_mm : Option<f32>, y_mm : Option<f32>, a_rad : Option<f32>) {
        if let Some(x_mm) = x_mm {
            self.x_mm = x_mm;
            self.x_is_precise = true;
        }

        if let Some(y_mm) = y_mm {
            self.y_mm = y_mm;
            self.y_is_precise = true;
        }
        if let Some(a_rad) = a_rad{
            self.linear_coordonate_offset[0] = a_rad - self.linear_coordonate[0];
            self.a_is_precise = true;
        }
    }

    pub fn compute_new_position(&mut self, delta_computation: [f32; N], delta_time_ms : f32)
    {
        for i in 0..N {
            self.linear_velocity[i] = delta_computation[i] / (delta_time_ms / 1000.0);
            if i == 0 {
               self.linear_coordonate[0] = delta_computation[i] + self.linear_coordonate_offset[0];
            }
            else {
                self.linear_coordonate[i] += delta_computation[i];
            }            
        }

        match N {
            2 => {
                self.x_mm += cosf(self.get_a_rad()) * delta_computation[1];
                self.y_mm += sinf(self.get_a_rad()) * delta_computation[1];
            },
            _ => {
                unimplemented!("Only 2D coordinates are supported");
            }
        }
    }

    //return the x axis precision flag
    pub fn is_x_precise(&self) -> bool {
        self.x_is_precise
    }

    //return the y axis precision flag
    pub fn is_y_precise(&self) -> bool {
        self.y_is_precise
    }

    //return the angle axis precision flag
    pub fn is_a_precise(&self) -> bool {
        self.a_is_precise
    }

    //set calibration needed flag for the x axis
    pub fn mark_x_as_imprecise(&mut self)  {
        self.x_is_precise = false;
    }

    //set calibration needed flag for the y axis
    pub fn mark_y_as_imprecise(&mut self) {
        self.y_is_precise = false;
    }

    //set calibration needed flag for the angle axis
    pub fn mark_a_as_imprecise(&mut self) {
        self.a_is_precise = false;
    }
    
    //copy precision data from an old coordinate structure
    pub fn copy_precision_from(&mut self, old : &RobotCoordinate<N>) {
        self.x_is_precise = old.is_x_precise();
        self.y_is_precise = old.is_y_precise();
        self.a_is_precise = old.is_a_precise();
    }

    //return the current angle in radian
    pub fn get_angle_rad(&self) -> f32 {
        self.linear_coordonate[0]
    }

    //return the current angle in degree
    pub fn get_angle_deg(&self) -> f32 {
        self.get_angle_rad().to_degrees()
    }

    //return the current angle speed in rad/s
    pub fn get_angle_speed_rad_s(&self) -> f32 {
        self.linear_velocity[0]
    }

    //return the current angle speed in degree/s
    pub fn get_angle_speed_deg_s(&self) -> f32 {
        self.get_angle_speed_rad_s().to_degrees()
    }
}

impl RobotCoordinate<2> {
    //return the linear distance in mm
    pub fn get_distance_mm(&self) -> f32 {
        self.linear_coordonate[1]
    }

    //return the linear speed in mm/s
    pub fn get_distance_speed_mm_s(&self) -> f32 {
        self.linear_velocity[1]
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const X_INIT : f32 = 1.0;
    const Y_INIT : f32 = 2.0;
    const A_INIT : f32 = 3.0;
    fn prepare_structure_for_test<const N: usize>() -> RobotCoordinate<N> {
        RobotCoordinate::from(X_INIT, Y_INIT, A_INIT)
    }

    #[test]
    fn check_field_initialization() {
        //create data
        let position = prepare_structure_for_test::<2>();

        //check internal fields
        assert_eq!(position.get_x_mm(), X_INIT);
        assert_eq!(position.get_y_mm(), Y_INIT);
        assert_eq!(position.get_a_rad(), A_INIT);
    }

    #[test]
    fn check_degree_conversion() {
        //create data
        let position = prepare_structure_for_test::<2>();

        //check conversion
        assert_eq!(position.get_a_deg(), A_INIT * 180.0 / core::f32::consts::PI);
    }

    #[test]
    fn check_precision_flags() {
        //create initla data
        let mut position = prepare_structure_for_test::<2>();

        //check precision
        let mut cpy_position = prepare_structure_for_test();
        cpy_position.copy_precision_from(&position);
        assert_eq!(position.is_x_precise(), true);
        assert_eq!(position.is_y_precise(), true);
        assert_eq!(position.is_a_precise(), true);
        assert_eq!(cpy_position.is_x_precise(), true);
        assert_eq!(cpy_position.is_y_precise(), true);
        assert_eq!(cpy_position.is_a_precise(), true);

        //set x axis
        position.mark_x_as_imprecise();

        //check precision
        let mut cpy_position = prepare_structure_for_test();
        cpy_position.copy_precision_from(&position);
        assert_eq!(position.is_x_precise(), false);
        assert_eq!(position.is_y_precise(), true);
        assert_eq!(position.is_a_precise(), true);
        assert_eq!(cpy_position.is_x_precise(), false);
        assert_eq!(cpy_position.is_y_precise(), true);
        assert_eq!(cpy_position.is_a_precise(), true);

        //set y axis
        position.mark_y_as_imprecise();

        //check precision
        let mut cpy_position = prepare_structure_for_test();
        cpy_position.copy_precision_from(&position);
        assert_eq!(position.is_x_precise(), false);
        assert_eq!(position.is_y_precise(), false);
        assert_eq!(position.is_a_precise(), true);
        assert_eq!(cpy_position.is_x_precise(), false);
        assert_eq!(cpy_position.is_y_precise(), false);
        assert_eq!(cpy_position.is_a_precise(), true);

        //set a axis
        position.mark_a_as_imprecise();

        //check precision
        let mut cpy_position = prepare_structure_for_test();
        cpy_position.copy_precision_from(&position);
        assert_eq!(position.is_x_precise(), false);
        assert_eq!(position.is_y_precise(), false);
        assert_eq!(position.is_a_precise(), false);
        assert_eq!(cpy_position.is_x_precise(), false);
        assert_eq!(cpy_position.is_y_precise(), false);
        assert_eq!(cpy_position.is_a_precise(), false);
    }
}