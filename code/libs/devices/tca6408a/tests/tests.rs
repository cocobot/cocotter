use embedded_hal::digital::PinState;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tca6408a::{Address, Tca6408a};
use tokio_test::block_on;

#[test]
fn read_input_pin() {
    let pin_state : u8 = 0b1100_0011;

    let expectations = [
        I2cTransaction::write_read(0xaa, vec![0x00], vec![pin_state]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from(0xaa));
    
    //check if the input pin state is the same as the one read from the I2C bus
    let result = block_on(tca6408a.get_input());
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), pin_state);

    //check if all expectations are met
    i2c.done();
}

#[test]
fn write_output_pin() {
    let pin_state : u8 = 0b0011_0011;

    let expectations = [
        I2cTransaction::write(0xaa, vec![0x01, pin_state]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from(0xaa));
    
    //check if the output pin state is the same as the one written to the I2C bus
    let result = block_on(tca6408a.set_output(pin_state));
    assert!(result.is_ok());

    //check if all expectations are met
    i2c.done();
}

#[test]
fn configure_polarity() {
    let pin_polarity : u8 = 0b1010_0011;

    let expectations = [
        I2cTransaction::write(0xaa, vec![0x02, pin_polarity]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from(0xaa));
    
    //run the configuration
    let result = block_on(tca6408a.set_polarity(pin_polarity));
    assert!(result.is_ok());

    //check if all expectations are met
    i2c.done();
}

#[test]
fn configure_pin_mode() {
    let pin_mode : u8 = 0b1111_0011;

    let expectations = [
        I2cTransaction::write(0xaa, vec![0x03, !pin_mode]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from(0xaa));
    
    //Run the configuration
    let result = block_on(tca6408a.configure_output(pin_mode));
    assert!(result.is_ok());

    //check if all expectations are met
    i2c.done();
}

#[test]
fn set_addr_low() {
    let pin_state : u8 = 0b1100_0011;

    let expectations = [
        I2cTransaction::write_read(0x20, vec![0x00], vec![pin_state]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from_pin_state(PinState::Low));
    
    //check if the input pin state is the same as the one read from the I2C bus
    let result = block_on(tca6408a.get_input());
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), pin_state);

    //check if all expectations are met
    i2c.done();
}

#[test]
fn set_addr_high() {
    let pin_state : u8 = 0b1100_0011;

    let expectations = [
        I2cTransaction::write_read(0x21, vec![0x00], vec![pin_state]),
    ];

    let mut i2c = I2cMock::new(&expectations);
 
    let mut tca6408a = Tca6408a::new(&mut i2c, Address::from_pin_state(PinState::High));
    
    //check if the input pin state is the same as the one read from the I2C bus
    let result = block_on(tca6408a.get_input());
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), pin_state);

    //check if all expectations are met
    i2c.done();
}