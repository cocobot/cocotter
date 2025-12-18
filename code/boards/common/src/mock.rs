use crate::{BatteryLevel, Encoder};


#[derive(Clone, Copy, Default)]
pub struct MockBatteryLevel {
    mv: u16,
    percent: u8,
}

impl BatteryLevel for MockBatteryLevel {
    fn read_vbatt(&mut self) -> (u16, u8) {
        (self.mv, self.percent)
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

