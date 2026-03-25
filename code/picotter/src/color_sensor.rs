//! Color sensor driver (placeholder)

use cancaner::Color;
use crate::arm::ArmError;
use crate::module::ColorSensor;

/// Dummy color sensor that always returns Unknown
pub struct DummyColorSensor;

impl DummyColorSensor {
    pub fn new() -> Self {
        Self
    }
}

impl ColorSensor for DummyColorSensor {
    async fn read_color(&mut self, _arm: u8) -> Result<Color, ArmError> {
        Ok(Color::Unknown)
    }
}
