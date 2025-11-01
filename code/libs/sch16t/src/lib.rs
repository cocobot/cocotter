use embedded_hal::spi::{Operation, SpiDevice};
use num_traits::FromBytes;
use std::{
    thread,
    time::{Duration, Instant},
};

/// SCH16T
///
/// The SCH16T is a combined gyroscope and accelerometer.
#[allow(dead_code)]
pub struct Sch16t<SPI> {
    spi: SPI,
    address: u8,
    angle_scale: SCH16TAngleScale,
    acceleration_scale: SCH16TAccScale,
    angle: Sch16tAxis,
    acceleration: Sch16tAxis,
    last_measure_instant: Option<Instant>,
}

pub struct Sch16tAxis {
    x: f32,
    y: f32,
    z: f32,
}

/// SCH16T register addresses
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Register {
    RateX1 = 0x01,
    RateY1 = 0x02,
    RateZ1 = 0x03,
    AccX1 = 0x04,
    AccY1 = 0x05,
    AccZ1 = 0x06,
    AccX3 = 0x07,
    AccY3 = 0x08,
    AccZ3 = 0x09,
    RateX2 = 0x0A,
    RateY2 = 0x0B,
    RateZ2 = 0x0C,
    AccX2 = 0x0D,
    AccY2 = 0x0E,
    AccZ2 = 0x0F,
    Temp = 0x10,
    RateDCNT = 0x11,
    AccDCNT = 0x12,
    FreqCntr = 0x13,
    StatSum = 0x14,
    StatSumStat = 0x15,
    StatCom = 0x16,
    StatRateCom = 0x17,
    StatRateX = 0x18,
    StatRateY = 0x19,
    StatRateZ = 0x1A,
    StatAccX = 0x1B,
    StatAccY = 0x1C,
    StatAccZ = 0x1D,
    StatSyncActive = 0x1E,
    StatInfo = 0x1F,
    CtrlFiltRate = 0x25,
    CtrlFiltAcc12 = 0x26,
    CtrlFiltAcc3 = 0x27,
    CtrlRate = 0x28,
    CtrlAcc12 = 0x29,
    CtrlAcc3 = 0x2A,
    CtrlUserIf = 0x33,
    CtrlSt = 0x34,
    CtrlMode = 0x35,
    CtrlReset = 0x36,
    SysTest = 0x37,
    Spare1 = 0x38,
    Spare2 = 0x39,
    Spare3 = 0x3A,
    AsicId = 0x3B,
    CompId = 0x3C,
    SnId1 = 0x3D,
    SnId2 = 0x3E,
    SnId3 = 0x3F,
}

enum SCH16TFrameStatus {
    Normal = 0b00,
    Error = 0b01,
    Saturation = 0b10,
    Initialization = 0b11,
}

#[derive(Clone, Copy)]
pub enum SCH16TAngleScale {
    Scale300degs = 0b010,
    Scale125degs = 0b011,
    Scale62_5degs = 0b100,
}

#[derive(Clone, Copy)]
pub enum SCH16TAccScale {
    Scale80ms2 = 0b001,
    Scale60ms2 = 0b010,
    Scale30ms2 = 0b011,
    Scale15ms2 = 0b100,
}

#[allow(dead_code)]
enum SCH16TAngleFilter {
    Filter68Hz = 0b000,
    Filter30Hz = 0b001,
    Filter13Hz = 0b010,
    Filter280Hz = 0b011,
    Filter370Hz = 0b100,
    Filter235Hz = 0b101,
    Bypass = 0b111,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
enum SCH16TDecimationRatio {
    DecRatio1 = 0b000,
    DecRatio2 = 0b001,
    DecRatio4 = 0b010,
    DecRatio8 = 0b011,
    DecRatio16 = 0b100,
}

#[allow(dead_code)]
struct SPI48bRdFrame {
    address: u8,
    register: u8,
    data: u32,
    dcnt: Option<u8>,
    crc_ok: bool,
    status: SCH16TFrameStatus,
}

#[derive(Clone, Copy)]
struct SCH16TRateCtrl {
    xyz1_scale: SCH16TAngleScale,
    xyz2_scale: SCH16TAngleScale,
    z2_decim_ratio: SCH16TDecimationRatio,
    y2_decim_ratio: SCH16TDecimationRatio,
    x2_decim_ratio: SCH16TDecimationRatio,
}

struct SCH16TAcc12Ctrl {
    xyz1_scale: SCH16TAccScale,
    xyz2_scale: SCH16TAccScale,
    z2_decim_ratio: SCH16TDecimationRatio,
    y2_decim_ratio: SCH16TDecimationRatio,
    x2_decim_ratio: SCH16TDecimationRatio,
}

#[allow(dead_code)]
struct Sch16tStatRate {
    decimated_rate_saturation_ok: bool,
    interpolated_rate_saturation_ok: bool,
    digital_continuous_self_test_ok: bool,
    analog_continuous_self_test_ok: bool,
    signal_status_ok: bool,
    all_flags_ok: bool,
}

impl From<u16> for Sch16tStatRate {
    fn from(value: u16) -> Self {
        Sch16tStatRate {
            decimated_rate_saturation_ok: value & 1 << 9 != 0,
            interpolated_rate_saturation_ok: value & 1 << 8 != 0,
            digital_continuous_self_test_ok: value & 1 << 6 != 0,
            analog_continuous_self_test_ok: value & 1 << 5 != 0,
            signal_status_ok: value & 1 << 4 != 0,
            all_flags_ok: value & 0b0000_0011_0111_0000 != 0b0000_0011_0111_0000,
        }
    }
}

impl From<u8> for SCH16TFrameStatus {
    fn from(raw: u8) -> Self {
        match raw {
            0x00 => SCH16TFrameStatus::Normal,
            0x01 => SCH16TFrameStatus::Error,
            0x02 => SCH16TFrameStatus::Saturation,
            0x03 => SCH16TFrameStatus::Initialization,
            _ => SCH16TFrameStatus::Error,
        }
    }
}

impl From<SCH16TRateCtrl> for u32 {
    fn from(rate_ctrl: SCH16TRateCtrl) -> u32 {
        let mut reg: u32 = 0;

        reg |= rate_ctrl.x2_decim_ratio as u32 & 0x0007;
        reg |= (rate_ctrl.y2_decim_ratio as u32 & 0x0007) << 3;
        reg |= (rate_ctrl.z2_decim_ratio as u32 & 0x0007) << 6;
        reg |= (rate_ctrl.xyz2_scale as u32 & 0x0007) << 9;
        reg |= (rate_ctrl.xyz1_scale as u32 & 0x0007) << 12;

        reg
    }
}

impl From<SCH16TAcc12Ctrl> for u32 {
    fn from(rate_ctrl: SCH16TAcc12Ctrl) -> u32 {
        let mut reg: u32 = 0;

        reg |= rate_ctrl.x2_decim_ratio as u32 & 0x0007;
        reg |= (rate_ctrl.y2_decim_ratio as u32 & 0x0007) << 3;
        reg |= (rate_ctrl.z2_decim_ratio as u32 & 0x0007) << 6;
        reg |= (rate_ctrl.xyz2_scale as u32 & 0x0007) << 9;
        reg |= (rate_ctrl.xyz1_scale as u32 & 0x0007) << 12;

        reg
    }
}

pub enum Sch16tError<SPI> {
    Spi(SPI),
    // Add other errors for your driver here.
}

impl<SPI> Sch16t<SPI>
where
    SPI: SpiDevice, // Sch16tError<SPI>: From<Sch16tError<<SPI as embedded_hal::spi::ErrorType>::Error>>
{
    /// Create a new SCH16T instance
    ///
    /// # Arguments
    /// * `spi` - spi peripheral
    /// * `address` - TC address of the SCH16T (typically 0x00-0x03)
    pub fn new(spi: SPI, address: u8) -> Self {
        Self {
            spi,
            address: address & 0x03,
            acceleration_scale: SCH16TAccScale::Scale15ms2,
            angle_scale: SCH16TAngleScale::Scale300degs, //SCH16TAngleScale::Scale62_5degs,
            angle: Sch16tAxis {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            acceleration: Sch16tAxis {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            last_measure_instant: None,
        }
    }

    /// Initialize the TCA6408 with default configuration
    /// All pins are configured as inputs with normal polarity
    pub fn init(&mut self) -> Result<(), Sch16tError<SPI>> {
        // wait 50ms for NVM read and SPI start up
        thread::sleep(Duration::from_millis(50));
        self.write_register(Register::CtrlReset, 0x10_u32);
        thread::sleep(Duration::from_millis(50));
        println!("version {}", self.read_register(Register::AsicId)?);
        println!("comp id {}", self.read_register(Register::CompId)?);

        println!("test {}", self.read_register(Register::SysTest)?);
        self.write_register(Register::SysTest, 0x1234_5678);
        println!("test {}", self.read_register(Register::SysTest)?);
        // configure registers
        let ctrl_rate_val = SCH16TRateCtrl {
            xyz1_scale: self.angle_scale,
            xyz2_scale: SCH16TAngleScale::Scale300degs,
            x2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            y2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            z2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
        };
        self.write_register(Register::CtrlRate, u32::from(ctrl_rate_val));
        println!("u32 ctrl rate {:#010x}", u32::from(ctrl_rate_val));

        let ctrl_usr_if_val = 0b0010_0000_0000_1100;
        self.write_register(Register::CtrlUserIf, ctrl_usr_if_val);

        let ctrl_acc12_val = SCH16TAcc12Ctrl {
            xyz1_scale: self.acceleration_scale,
            xyz2_scale: SCH16TAccScale::Scale80ms2,
            x2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            y2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            z2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
        };
        self.write_register(Register::CtrlAcc12, u32::from(ctrl_acc12_val));

        // finally set ctrl mode
        self.write_register(Register::CtrlMode, 0x0001);

        // wait 215ms
        thread::sleep(Duration::from_millis(215));

        // read status bits
        let _ = self.read_register(Register::StatSum);
        let _ = self.read_register(Register::StatSumStat);
        let _ = self.read_register(Register::StatCom);
        let _ = self.read_register(Register::StatRateX);
        let _ = self.read_register(Register::StatRateY);
        let _ = self.read_register(Register::StatRateZ);
        let _ = self.read_register(Register::StatAccX);
        let _ = self.read_register(Register::StatAccY);
        let _ = self.read_register(Register::StatAccZ);
        let _ = self.read_register(Register::StatInfo);

        // finally set ctrl mode
        self.write_register(Register::CtrlMode, 0x0003);

        //wait 3ms
        thread::sleep(Duration::from_millis(3));

        // read all status registers twice
        for _i in 0..2 {
            let _ = self.read_register(Register::StatSum);
            let _ = self.read_register(Register::StatSumStat);
            let _ = self.read_register(Register::StatCom);
            let _ = self.read_register(Register::StatRateX);
            let _ = self.read_register(Register::StatRateY);
            let _ = self.read_register(Register::StatRateZ);
            let _ = self.read_register(Register::StatAccX);
            let _ = self.read_register(Register::StatAccY);
            let _ = self.read_register(Register::StatAccZ);
            let _ = self.read_register(Register::StatInfo);
        }

        println!(
            "ctrl rate {:#010x}",
            self.read_register(Register::CtrlRate)?
        );
        let _ = self.read_register(Register::CtrlUserIf);
        let _ = self.read_register(Register::CtrlAcc12);

        Ok(())
    }

    pub fn update_angle(&mut self) -> Result<(), Sch16tError<SPI>> {
        let x_stat = self.read_register(Register::StatRateX)?;
        let y_stat = self.read_register(Register::StatRateY)?;
        let z_stat = self.read_register(Register::StatRateZ)?;
        if self.last_measure_instant.is_none() {
            self.last_measure_instant = Some(Instant::now());
        } else {
            let elapsed_time_s = self.last_measure_instant.unwrap().elapsed().as_secs_f32();
            self.last_measure_instant = Some(Instant::now());
            if Sch16tStatRate::from(x_stat as u16).all_flags_ok {
                self.angle.x += self.angle_from_register(Register::RateX1)? * elapsed_time_s;
            } else {
                self.angle.x += self.angle_from_register(Register::RateX2)? * elapsed_time_s;
                println!("x meas ko");
            }
            if Sch16tStatRate::from(y_stat as u16).all_flags_ok {
                self.angle.y += self.angle_from_register(Register::RateY1)? * elapsed_time_s;
            } else {
                self.angle.x += self.angle_from_register(Register::RateY2)? * elapsed_time_s;
                println!("y meas ko");
            }
            if Sch16tStatRate::from(z_stat as u16).all_flags_ok {
                self.angle.z += self.angle_from_register(Register::RateZ1)? * elapsed_time_s;
            } else {
                self.angle.x += self.angle_from_register(Register::RateZ2)? * elapsed_time_s;
                println!("z meas ko");
            }
        }
        Ok(())
    }

    fn angle_from_register(&mut self, register: Register) -> Result<f32, Sch16tError<SPI>> {
        let mut raw_reg = self.read_register(register)?;
        raw_reg &= 0x000f_ffff;
        raw_reg <<= 12;
        let f32_reg = ((raw_reg as i32) >> 12) as f32;
        // angle is set to default 300deg/s for rate2 and is configured by self.angle_scale for rate1 data
        match register {
            Register::RateX1 | Register::RateY1 | Register::RateZ1 => match self.angle_scale {
                SCH16TAngleScale::Scale300degs => Ok(f32_reg / 1600.0),
                SCH16TAngleScale::Scale125degs => Ok(f32_reg / 3200.0),
                SCH16TAngleScale::Scale62_5degs => Ok(f32_reg / 6400.0),
            },
            Register::RateX2 | Register::RateY2 | Register::RateZ2 => Ok(f32_reg / 1600.0),
            _ => Ok(0.0), // TODO replace it by Err(tbd)
        }
    }

    pub fn read_angle(&mut self) -> [f32; 3] {
        [self.angle.x, self.angle.y, self.angle.z]
    }

    fn read_register(&mut self, register: Register) -> Result<u32, Sch16tError<SPI>> {
        let frame = self.create_spi48bf(register, true, 0x00000000);

        let mut answer: [u8; 6] = [0x00; 6];
        // `transaction` asserts and deasserts CS for us. No need to do it manually!
        let _ = self
            .spi
            .transaction(&mut [Operation::Write(&frame)])
            .map_err(Sch16tError::Spi);

        let _ = self
            .spi
            .transaction(&mut [Operation::Read(&mut answer)])
            .map_err(Sch16tError::Spi);

        let parsed_answer = Self::parse_spi48bf(answer);

        Ok(parsed_answer.data)
    }

    /// write a register to the SCH16T and return the write error status
    fn write_register(&mut self, register: Register, data: u32) -> bool {
        let frame = self.create_spi48bf(register, false, data);
        let mut answer: [u8; 6] = [0x00; 6];
        let _ = self
            .spi
            .transaction(&mut [Operation::Write(&frame)])
            .map_err(Sch16tError::Spi);
        let _ = self
            .spi
            .transaction(&mut [Operation::Read(&mut answer)])
            .map_err(Sch16tError::Spi);
        let parsed_answer = Self::parse_spi48bf(answer);
        matches!(parsed_answer.status, SCH16TFrameStatus::Error)
    }

    fn parse_spi48bf(raw: [u8; 6]) -> SPI48bRdFrame {
        let mut raw_u64 = [0x00; 8];
        for (i, &val) in raw.iter().enumerate() {
            raw_u64[i + 2] = val;
        }
        let full_frame: u64 = FromBytes::from_be_bytes(&raw_u64);
        // received SPI48B frame structure
        // D TA[9:0] IDS CE S[1:0] DCNT* RFU* DATA[19:0] CRC|7:0]
        let dcnt = if full_frame & (1 << 47) != 0 {
            Some((full_frame >> 29) as u8 & 0b0000_1111)
        } else {
            None
        };

        let address: u8 = (full_frame >> 45) as u8 & 0x03;
        let register: u8 = (full_frame >> 38) as u8;
        let status = if full_frame & (1 << 35) == 0 {
            SCH16TFrameStatus::from((full_frame >> 33) as u8)
        } else {
            SCH16TFrameStatus::Error
        };
        let data = (full_frame >> 8 & 0x000FFFFF) as u32; // 20 bits

        let crc_received = raw[5];
        //TODO bugged
        let frame_crc = compute_crc8(&raw[1..]);

        SPI48bRdFrame {
            address,
            register,
            data,
            dcnt,
            crc_ok: crc_received == frame_crc,
            status,
        }
    }

    /// create the spi48 bit frame
    fn create_spi48bf(&self, register: Register, read: bool, data: u32) -> [u8; 6] {
        // the SPI 48b frame definition :
        // TA[9:0] RW_bit 0 1 AE[6:0] data[19:0] CRC8[7:0]
        let mut full_frame: u64 = (1_u64) << 35; // set FT bit that indicates that the frame is 48 bits long 
        full_frame |= (self.address as u64) << 46; // address lsb contains the TA[9:8] bits     
        full_frame |= (register as u64) << 38; // TA[7:0] must contains the register address
        full_frame |= match read {
            true => 0,
            false => 1,
        } << 37;
        full_frame |= (data as u64 & 0x0000_0000_000f_ffff) << 8;

        let crc = compute_crc8(&full_frame.to_be_bytes()[2..7]);
        full_frame |= crc as u64;

        <[u8; 6]>::try_from(&full_frame.to_be_bytes()[2..8]).unwrap()
    }
}

fn compute_crc8(frame: &[u8]) -> u8 {
    let crc = crc::Crc::<u8>::new(&crc::CRC_8_OPENSAFETY);
    let mut digest = crc.digest();
    digest.update(&[0xff]);
    digest.update(&frame);
    digest.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    /*
    #[test]
    fn test_gpio_pins_bitfield() {
        let mut pins = GpioPins::new();

        // Test setting individual pins
        pins.set_p0(true);
        pins.set_p7(true);

        assert!(pins.p0());
        assert!(pins.p7());
        assert!(!pins.p1());
        assert_eq!(pins.as_u8(), 0b10000001);

        // Test setting via pin number
        pins.set_pin(3, true);
        assert!(pins.get_pin(3));
        assert_eq!(pins.as_u8(), 0b10001001);
    }
    */
    #[test]
    fn test_create_48b_frame() {
        let mut inst = sch16t::new(spi::spi2, 0x00);

        let frame = inst.create_spi48bf(Register::CtrlMode, false, 0x0000_0003);

        let expected_frame = [0x0D, 0x68, 0x00, 0x00, 0x03, 0x8D];

        assert_eq!(frame, expected_frame);
    }
    /*
    #[test]
    fn test_gpio_pins_default() {
        let pins = GpioPins::default();
        assert_eq!(pins.as_u8(), 0);

        for i in 0..8 {
            assert!(!pins.get_pin(i));
        }
    }

    #[test]
    fn test_gpio_pins_set_get() {
        let mut pins = GpioPins::new();

        // Test all pins
        for i in 0..8 {
            pins.set_pin(i, true);
            assert!(pins.get_pin(i));
        }

        assert_eq!(pins.as_u8(), 0xFF);

        // Test clearing pins
        for i in 0..8 {
            pins.set_pin(i, false);
            assert!(!pins.get_pin(i));
        }

        assert_eq!(pins.as_u8(), 0x00);
    }

    #[test]
    fn test_gpio_pins_invalid_pin() {
        let pins = GpioPins::new();

        // Test invalid pin numbers
        assert!(!pins.get_pin(8));
        assert!(!pins.get_pin(255));

        let mut pins = GpioPins::new();
        pins.set_pin(8, true);  // Should be ignored
        pins.set_pin(255, true); // Should be ignored
        assert_eq!(pins.as_u8(), 0);
    }*/
}
