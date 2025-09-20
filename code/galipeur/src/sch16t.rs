use embedded_hal::spi::SpiDevice;
use std::{thread, time::Duration};

/// SCH16T SafeSPI v2.0 Register addresses
mod registers {
    // SafeSPI v2.0 uses 16-bit addresses
    // Status and control registers
    pub const SPI_STAT: u16 = 0x0000;  // SPI status register
    pub const RATE_X: u16 = 0x0002;    // Angular rate X-axis
    pub const RATE_Y: u16 = 0x0004;    // Angular rate Y-axis
    pub const RATE_Z: u16 = 0x0006;    // Angular rate Z-axis
    pub const ACC_X: u16 = 0x0008;     // Acceleration X-axis
    pub const ACC_Y: u16 = 0x000A;     // Acceleration Y-axis
    pub const ACC_Z: u16 = 0x000C;     // Acceleration Z-axis
    pub const TEMP: u16 = 0x000E;      // Temperature
    pub const STAT_SUM: u16 = 0x0010;  // Status summary
    pub const COM_STAT1: u16 = 0x0012; // Common status 1
    pub const COM_STAT2: u16 = 0x0014; // Common status 2
    pub const RATE_STAT1: u16 = 0x0016; // Rate sensor status 1
    pub const RATE_STAT2: u16 = 0x0018; // Rate sensor status 2
    pub const ACC_STAT1: u16 = 0x001A;  // Accelerometer status 1
    pub const ACC_STAT2: u16 = 0x001C;  // Accelerometer status 2

    // Configuration registers (write addresses have bit 15 set)
    pub const FILTER_CTRL: u16 = 0x8022; // Filter control
    pub const SPI_CTRL: u16 = 0x8024;    // SPI control
    pub const SENS_CTRL: u16 = 0x8026;   // Sensor control
}

/// SafeSPI v2.0 protocol specifics
mod safespi {
    // SafeSPI v2.0 frame format:
    // [16-bit address/command] [16-bit data]
    // MSB of address: 0=read, 1=write
    pub const READ_BIT: u16 = 0x0000;
    pub const WRITE_BIT: u16 = 0x8000;

    // CRC polynomial for SafeSPI v2.0
    pub const CRC_POLY: u8 = 0x1D; // x^8 + x^4 + x^3 + x^2 + 1

    pub fn calculate_crc(data: &[u8]) -> u8 {
        let mut crc = 0xFFu8;
        for byte in data {
            crc ^= *byte;
            for _ in 0..8 {
                if (crc & 0x80) != 0 {
                    crc = (crc << 1) ^ CRC_POLY;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}

/// Gyroscope full scale range
#[derive(Debug, Clone, Copy)]
pub enum GyroScale {
    /// ±245 degrees per second
    Dps245 = 0b00,
    /// ±500 degrees per second
    Dps500 = 0b01,
    /// ±1000 degrees per second
    Dps1000 = 0b10,
    /// ±2000 degrees per second
    Dps2000 = 0b11,
}

/// Accelerometer full scale range
#[derive(Debug, Clone, Copy)]
pub enum AccelScale {
    /// ±2g
    G2 = 0b00,
    /// ±4g
    G4 = 0b01,
    /// ±6g
    G6 = 0b10,
    /// ±8g
    G8 = 0b11,
    /// ±16g
    G16 = 0b111,
}

/// Output data rate
#[derive(Debug, Clone, Copy)]
pub enum OutputDataRate {
    /// Power down
    PowerDown = 0b0000,
    /// 12.5 Hz
    Hz12_5 = 0b0001,
    /// 25 Hz
    Hz25 = 0b0010,
    /// 50 Hz
    Hz50 = 0b0011,
    /// 100 Hz
    Hz100 = 0b0100,
    /// 200 Hz
    Hz200 = 0b0101,
    /// 400 Hz
    Hz400 = 0b0110,
    /// 800 Hz
    Hz800 = 0b0111,
    /// 1600 Hz
    Hz1600 = 0b1000,
}

/// 3-axis data structure
#[derive(Debug, Default, Clone, Copy)]
pub struct AxisData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// SCH16T 6-DOF IMU sensor driver
pub struct SCH16T<SPI> {
    spi: SPI,
    gyro_scale: GyroScale,
    accel_scale: AccelScale,
}

impl<SPI> SCH16T<SPI>
where
    SPI: SpiDevice,
{
    /// Create a new SCH16T driver instance
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            gyro_scale: GyroScale::Dps245,
            accel_scale: AccelScale::G2,
        }
    }

    /// Initialize the sensor with default settings
    pub fn init(&mut self) -> Result<(), SPI::Error> {
        // Wait for power-up
        thread::sleep(Duration::from_millis(50));

        // Read SPI status register to check communication
        let status = self.read_register(registers::SPI_STAT)?;
        log::info!("SCH16T SPI Status: 0x{:04X}", status);

        // Read status summary
        let stat_sum = self.read_register(registers::STAT_SUM)?;
        log::info!("SCH16T Status Summary: 0x{:04X}", stat_sum);

        // Configure sensor if needed
        // For now, use default configuration

        Ok(())
    }

    /// Read a 16-bit register using SafeSPI v2.0 protocol
    fn read_register(&mut self, addr: u16) -> Result<u16, SPI::Error> {
        // SafeSPI v2.0 frame: [16-bit address] [16-bit data]
        let addr_bytes = (addr | safespi::READ_BIT).to_be_bytes();
        let mut buffer = [addr_bytes[0], addr_bytes[1], 0x00, 0x00];

        log::debug!("SafeSPI Read: addr=0x{:04X}, sending: {:02X} {:02X} {:02X} {:02X}",
                   addr, buffer[0], buffer[1], buffer[2], buffer[3]);

        self.spi.transfer_in_place(&mut buffer)?;

        log::debug!("SafeSPI Read response: {:02X} {:02X} {:02X} {:02X}",
                   buffer[0], buffer[1], buffer[2], buffer[3]);

        let result = u16::from_be_bytes([buffer[2], buffer[3]]);
        Ok(result)
    }

    /// Write a 16-bit register using SafeSPI v2.0 protocol
    fn write_register(&mut self, addr: u16, value: u16) -> Result<(), SPI::Error> {
        // SafeSPI v2.0 frame: [16-bit address with write bit] [16-bit data]
        let addr_bytes = (addr | safespi::WRITE_BIT).to_be_bytes();
        let value_bytes = value.to_be_bytes();
        let buffer = [addr_bytes[0], addr_bytes[1], value_bytes[0], value_bytes[1]];

        log::debug!("SafeSPI Write: addr=0x{:04X}, value=0x{:04X}", addr, value);

        self.spi.write(&buffer)?;
        Ok(())
    }

    /// Configure sensor control register
    pub fn configure_sensor(&mut self, filter_enable: bool) -> Result<(), SPI::Error> {
        // Configure filter if needed
        if filter_enable {
            self.write_register(registers::FILTER_CTRL, 0x0001)?;
        }
        Ok(())
    }

    /// Read raw gyroscope data
    pub fn read_gyro_raw(&mut self) -> Result<AxisData, SPI::Error> {
        let x = self.read_register(registers::RATE_X)? as i16;
        let y = self.read_register(registers::RATE_Y)? as i16;
        let z = self.read_register(registers::RATE_Z)? as i16;

        Ok(AxisData { x, y, z })
    }

    /// Read raw accelerometer data
    pub fn read_accel_raw(&mut self) -> Result<AxisData, SPI::Error> {
        let x = self.read_register(registers::ACC_X)? as i16;
        let y = self.read_register(registers::ACC_Y)? as i16;
        let z = self.read_register(registers::ACC_Z)? as i16;

        Ok(AxisData { x, y, z })
    }

    /// Read gyroscope data in degrees per second
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), SPI::Error> {
        let raw = self.read_gyro_raw()?;
        let scale = match self.gyro_scale {
            GyroScale::Dps245 => 245.0 / 32768.0,
            GyroScale::Dps500 => 500.0 / 32768.0,
            GyroScale::Dps1000 => 1000.0 / 32768.0,
            GyroScale::Dps2000 => 2000.0 / 32768.0,
        };

        Ok((
            raw.x as f32 * scale,
            raw.y as f32 * scale,
            raw.z as f32 * scale,
        ))
    }

    /// Read accelerometer data in g
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), SPI::Error> {
        let raw = self.read_accel_raw()?;
        let scale = match self.accel_scale {
            AccelScale::G2 => 2.0 / 32768.0,
            AccelScale::G4 => 4.0 / 32768.0,
            AccelScale::G6 => 6.0 / 32768.0,
            AccelScale::G8 => 8.0 / 32768.0,
            AccelScale::G16 => 16.0 / 32768.0,
        };

        Ok((
            raw.x as f32 * scale,
            raw.y as f32 * scale,
            raw.z as f32 * scale,
        ))
    }

    /// Read temperature in Celsius
    pub fn read_temperature(&mut self) -> Result<f32, SPI::Error> {
        let raw = self.read_register(registers::TEMP)? as i16;
        // SCH16T temperature scaling: typically LSB = 0.01°C
        Ok(raw as f32 * 0.01)
    }

    /// Check if new data is available
    pub fn data_ready(&mut self) -> Result<bool, SPI::Error> {
        let status = self.read_register(registers::STAT_SUM)?;
        // Check if data ready bit is set (bit 0 typically)
        Ok((status & 0x0001) != 0)
    }

    /// Read status summary
    pub fn read_status(&mut self) -> Result<u16, SPI::Error> {
        self.read_register(registers::STAT_SUM)
    }

    /// Perform self-test by checking status registers
    pub fn self_test(&mut self) -> Result<bool, SPI::Error> {
        // Read all status registers
        let stat_sum = self.read_register(registers::STAT_SUM)?;
        let com_stat1 = self.read_register(registers::COM_STAT1)?;
        let com_stat2 = self.read_register(registers::COM_STAT2)?;
        let rate_stat1 = self.read_register(registers::RATE_STAT1)?;
        let acc_stat1 = self.read_register(registers::ACC_STAT1)?;

        log::info!("SCH16T Status: SUM={:04X}, COM1={:04X}, COM2={:04X}, RATE1={:04X}, ACC1={:04X}",
                  stat_sum, com_stat1, com_stat2, rate_stat1, acc_stat1);

        // Check if any error bits are set
        // Bit 15 is typically the error summary bit
        Ok((stat_sum & 0x8000) == 0)
    }

    /// Get raw sensor data in a single read
    pub fn read_all_raw(&mut self) -> Result<(AxisData, AxisData, i16), SPI::Error> {
        let gyro = self.read_gyro_raw()?;
        let accel = self.read_accel_raw()?;
        let temp = self.read_register(registers::TEMP)? as i16;

        Ok((gyro, accel, temp))
    }

    /// Test basic SPI communication
    pub fn test_communication(&mut self) -> Result<bool, SPI::Error> {
        log::info!("Testing SCH16T communication...");

        // First, try a simple SPI transaction to see if we get anything
        let mut test_buffer = [0x00, 0x00, 0x00, 0x00];
        self.spi.transfer_in_place(&mut test_buffer)?;
        log::info!("Raw SPI test: sent 00 00 00 00, received {:02X} {:02X} {:02X} {:02X}",
                  test_buffer[0], test_buffer[1], test_buffer[2], test_buffer[3]);

        // Try to read SPI status register (should not be 0xFFFF)
        let status = self.read_register(registers::SPI_STAT)?;
        log::info!("SPI_STAT register: 0x{:04X}", status);

        if status == 0xFFFF {
            log::error!("Getting 0xFFFF - MISO stuck high, no communication");

            // Try with different pattern to confirm
            let mut test = [0xAA, 0x55, 0xAA, 0x55];
            self.spi.transfer_in_place(&mut test)?;
            log::info!("Pattern test: sent AA 55 AA 55, received {:02X} {:02X} {:02X} {:02X}",
                      test[0], test[1], test[2], test[3]);

            return Ok(false);
        }

        if status == 0x0000 {
            log::error!("Getting 0x0000 - MISO stuck low or not connected");
            return Ok(false);
        }

        // Try to read all status registers
        let stat_sum = self.read_register(registers::STAT_SUM)?;
        log::info!("STAT_SUM register: 0x{:04X}", stat_sum);

        // Try reading sensor data
        let rate_x = self.read_register(registers::RATE_X)?;
        let acc_x = self.read_register(registers::ACC_X)?;
        log::info!("RATE_X: 0x{:04X}, ACC_X: 0x{:04X}", rate_x, acc_x);

        Ok(true)
    }

    /// Debug: dump all registers
    pub fn dump_registers(&mut self) -> Result<(), SPI::Error> {
        log::info!("SCH16T Register Dump:");
        log::info!("  SPI_STAT:   0x{:04X}", self.read_register(registers::SPI_STAT)?);
        log::info!("  RATE_X:     0x{:04X}", self.read_register(registers::RATE_X)?);
        log::info!("  RATE_Y:     0x{:04X}", self.read_register(registers::RATE_Y)?);
        log::info!("  RATE_Z:     0x{:04X}", self.read_register(registers::RATE_Z)?);
        log::info!("  ACC_X:      0x{:04X}", self.read_register(registers::ACC_X)?);
        log::info!("  ACC_Y:      0x{:04X}", self.read_register(registers::ACC_Y)?);
        log::info!("  ACC_Z:      0x{:04X}", self.read_register(registers::ACC_Z)?);
        log::info!("  TEMP:       0x{:04X}", self.read_register(registers::TEMP)?);
        log::info!("  STAT_SUM:   0x{:04X}", self.read_register(registers::STAT_SUM)?);
        log::info!("  COM_STAT1:  0x{:04X}", self.read_register(registers::COM_STAT1)?);
        log::info!("  COM_STAT2:  0x{:04X}", self.read_register(registers::COM_STAT2)?);
        Ok(())
    }
}