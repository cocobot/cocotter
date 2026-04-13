use embedded_can::{blocking::Can, Frame, Error};


/// CAN interface
///
/// Similar to `embedded_can::blocking::Can`, but without borrowing mutably.
pub trait CanInterface: Clone + Sync + Send {
    type Frame: Frame;
    type Error: Error;

    fn can_transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error>;
    fn can_receive(&self) -> Result<Self::Frame, Self::Error>;
}

#[cfg(feature = "std")]
impl<C: Can + Sync + Send> CanInterface for std::sync::Arc<std::sync::Mutex<C>> {
    type Frame = <C as Can>::Frame;
    type Error = <C as Can>::Error;

    fn can_transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error> {
        let mut can = self.lock().unwrap();
        can.transmit(frame)
    }

    fn can_receive(&self) -> Result<Self::Frame, Self::Error> {
        let mut can = self.lock().unwrap();
        can.receive()
    }
}

