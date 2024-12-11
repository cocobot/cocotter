use core::f32;

use libm::{cosf, sinf};

#[derive(Clone, Debug)]
//Create an holder for current robot coordinates
pub struct RobotCoordinate {
    //store robot position in standardized unit (mm/rad)
    x_mm : f32,                 
    y_mm : f32,
    a_rad : f32,

    //store if axis has been recalibrated recentrly
    x_is_precise : bool,
    y_is_precise : bool,
    a_is_precise : bool,
}


impl RobotCoordinate {
    //create a new structure from independant data
    pub fn from(x_mm : f32, y_mm : f32, a_rad : f32) -> RobotCoordinate {
        RobotCoordinate { x_mm, y_mm, a_rad, x_is_precise: true, y_is_precise: true, a_is_precise: true}
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
        self.a_rad
    }

    //return the angle position only (degrees)
    pub fn get_a_deg(&self) -> f32 {
        self.a_rad  * 180.0 / core::f32::consts::PI
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
            self.a_rad = a_rad;
            self.a_is_precise = true;
        }
    }

    pub fn compute_new_position(&mut self, distance_mm: f32, angle_rad: f32)
    {
        self.a_rad += angle_rad;
        self.x_mm  += cosf(self.a_rad) * distance_mm;
        self.y_mm  += sinf(self.a_rad) * distance_mm;
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
    pub fn copy_precision_from(&mut self, old : &RobotCoordinate) {
        self.x_is_precise = old.is_x_precise();
        self.y_is_precise = old.is_y_precise();
        self.a_is_precise = old.is_a_precise();
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const X_INIT : f32 = 1.0;
    const Y_INIT : f32 = 2.0;
    const A_INIT : f32 = 3.0;
    fn prepare_structure_for_test() -> RobotCoordinate {
        RobotCoordinate::from(X_INIT, Y_INIT, A_INIT)
    }

    #[test]
    fn check_field_initialization() {
        //create data
        let position = prepare_structure_for_test();

        //check internal fields
        assert_eq!(position.get_x_mm(), X_INIT);
        assert_eq!(position.get_y_mm(), Y_INIT);
        assert_eq!(position.get_a_rad(), A_INIT);
    }

    #[test]
    fn check_degree_conversion() {
        //create data
        let position = prepare_structure_for_test();

        //check conversion
        assert_eq!(position.get_a_deg(), A_INIT * 180.0 / core::f32::consts::PI);
    }

    #[test]
    fn check_precision_flags() {
        //create initla data
        let mut position = prepare_structure_for_test();

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