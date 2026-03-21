use crate::{BatteryLevel, hal::{BatteryReader, Encoder}};


#[derive(Default)]
pub struct MockBatteryReader(BatteryLevel);

impl BatteryReader for MockBatteryReader {
    fn read_vbatt(&mut self) -> BatteryLevel {
        self.0
    }
}


#[derive(Default)]
pub struct MockEncoder<T: Default + Copy>(T);

impl<T: Default + Copy> Encoder<T> for MockEncoder<T> {
    type Error = ();

    fn get_value(&self) -> Result<T, Self::Error> {
        Ok(self.0)
    }
}

