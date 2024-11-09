use embedded_hal::digital::PinState;
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// I2C device address
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Address(u8);

impl From<u8> for Address {
    fn from(a: u8) -> Self {
        Address(a)
    }
}

impl Address {
    pub fn from_pin_state(addr: PinState) -> Self {
        match addr {
            PinState::Low => Address(0b010_0000),
            PinState::High  => Address(0b010_0001),
        }
    }
}

#[derive(Debug)]

pub struct Tca6408a<I2C> {
    i2c: I2C,
    address: u8,
}

#[derive(Debug)]
struct Register;
impl Register {
    const INPUT: u8 = 0x00;
    const OUTPUT: u8 = 0x01;
    const POLARITY: u8 = 0x02;
    const CONFIG: u8 = 0x03;
}

impl<I2C, E> Tca6408a<I2C>
where
    I2C: AsyncI2c<Error = E>,
{
    /// Create a new instance of the TCA6408A device.
    pub fn new(i2c: I2C, address: Address) -> Self {
        Tca6408a {
            i2c,
            address: address.0,
        }
    }

    /// Configure the pins as input or output.
    /// 0 = output, 1 = input
    pub async fn configure_output(&mut self, pins: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Register::CONFIG, !pins])
            .await
    }

    /// Set the output state of the pins.
    pub async fn set_output(&mut self, pins: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Register::OUTPUT, pins])
            .await
    }

    /// Get the input state of the pins.
    pub async fn get_input(&mut self) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[Register::INPUT], &mut data)
            .await?;
        Ok(data[0])
    }

    /// Set the polarity of the pins.
    pub async fn set_polarity(&mut self, polarity: u8) -> Result<(), E> {
        self.i2c
            .write(self.address, &[Register::POLARITY, polarity])
            .await
    }
}
