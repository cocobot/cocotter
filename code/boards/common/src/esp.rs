use crate::hal::Encoder;
pub use esp32_encoder::Encoder as EspEncoder;
use esp_idf_svc::sys::EspError;


impl Encoder<i32> for EspEncoder<'_> {
    type Error = EspError;

    fn get_value(&self) -> Result<i32, EspError> {
        EspEncoder::get_value(self)
    }
}

