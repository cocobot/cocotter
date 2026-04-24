use std::time::Duration;

pub const ASSERV_PERIOD: Duration = Duration::from_millis(15);

#[cfg(target_os = "espidf")]
mod esp;
#[cfg(not(target_os = "espidf"))]
mod sim;

#[cfg(target_os = "espidf")]
pub use esp::PamiAsservHardware;
#[cfg(not(target_os = "espidf"))]
pub use sim::PamiAsservHardware;
