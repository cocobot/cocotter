use crate::{BatteryLevel, hal::{BatteryReader, Encoder}};


/// Default mock battery: a fully-charged 4S pack at 16 V. Avoids the
/// "battery low" spam you'd get from `BatteryLevel::default()` (0 mV).
pub struct MockBatteryReader(BatteryLevel);

impl Default for MockBatteryReader {
    fn default() -> Self {
        Self(BatteryLevel { mv: 16_000, percent: 100 })
    }
}

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

