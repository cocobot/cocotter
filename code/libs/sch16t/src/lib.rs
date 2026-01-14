use embedded_hal::spi::{Operation, SpiDevice};
use std::{
    thread,
    time::{Duration, Instant},
    fmt::Debug,
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

#[derive(Default)]
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
#[derive(Debug, Clone, Copy)]
pub struct Sch16tStatRate {
    decimated_rate_saturation_ok: bool,
    interpolated_rate_saturation_ok: bool,
    digital_continuous_self_test_ok: bool,
    analog_continuous_self_test_ok: bool,
    signal_status_ok: bool,
}

impl Sch16tStatRate {
    const fn all_flags_ok(&self) -> bool {
        self.decimated_rate_saturation_ok &&
            self.interpolated_rate_saturation_ok &&
            self.digital_continuous_self_test_ok &&
            self.analog_continuous_self_test_ok &&
            self.signal_status_ok
    }
}

impl From<u16> for Sch16tStatRate {
    fn from(value: u16) -> Self {
        Sch16tStatRate {
            decimated_rate_saturation_ok: value & 1 << 9 != 0,
            interpolated_rate_saturation_ok: value & 1 << 8 != 0,
            digital_continuous_self_test_ok: value & 1 << 6 != 0,
            analog_continuous_self_test_ok: value & 1 << 5 != 0,
            signal_status_ok: value & 1 << 4 != 0,
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
    XReading(Sch16tStatRate),
    YReading(Sch16tStatRate),
    ZReading(Sch16tStatRate),
}

impl<SPI: SpiDevice> Debug for Sch16tError<SPI> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Sch16tError::Spi(_) => write!(f, "SPI communication error"),
            Sch16tError::XReading(stat) => write!(f, "X error: {:?}", stat),
            Sch16tError::YReading(stat) => write!(f, "Y error: {:?}", stat),
            Sch16tError::ZReading(stat) => write!(f, "Z error: {:?}", stat),
        }
    }
}

impl<SPI: SpiDevice> Sch16t<SPI> {
    /// Create a new SCH16T instance
    ///
    /// # Arguments
    /// * `spi` - spi peripheral
    /// * `address` - TC address of the SCH16T (2 bits)
    pub fn new(spi: SPI, address: u8) -> Self {
        Self {
            spi,
            address: address & 0x03,
            acceleration_scale: SCH16TAccScale::Scale15ms2,
            angle_scale: SCH16TAngleScale::Scale300degs,
            angle: Sch16tAxis::default(),
            acceleration: Sch16tAxis::default(),
            last_measure_instant: None,
        }
    }

    /// Initialize the TCA6408 with default configuration
    /// All pins are configured as inputs with normal polarity
    pub fn init(&mut self) -> Result<(), Sch16tError<SPI>> {
        // Wait 32ms for NVM read and SPI start up
        thread::sleep(Duration::from_millis(50));
        self.write_register(Register::CtrlReset, 0x10);  //TODO Why 0x10?
        thread::sleep(Duration::from_millis(50));
        //XXX Communication is not allowed during 2ms after SPI SOFTRESET
        log::info!("sch16t: asic_id {:03x}, comp_id: {:04x}", self.read_register(Register::AsicId)?, self.read_register(Register::CompId)?);

        // Configure registers
        let ctrl_rate_val = SCH16TRateCtrl {
            xyz1_scale: self.angle_scale,
            xyz2_scale: SCH16TAngleScale::Scale300degs,
            x2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            y2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            z2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
        };
        self.write_register(Register::CtrlRate, ctrl_rate_val.into());
        self.write_register(Register::CtrlUserIf, 0b0010_0000_0000_1100);
        let ctrl_acc12_val = SCH16TAcc12Ctrl {
            xyz1_scale: self.acceleration_scale,
            xyz2_scale: SCH16TAccScale::Scale80ms2,
            x2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            y2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
            z2_decim_ratio: SCH16TDecimationRatio::DecRatio16,
        };
        self.write_register(Register::CtrlAcc12, ctrl_acc12_val.into());

        // Write EN_SENSOR=1 (CTRL_MODE = EN_SENSOR)
        self.write_register(Register::CtrlMode, 0b01);

        // Wait 215ms
        thread::sleep(Duration::from_millis(215));

        // Read status bits
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

        // Write EOI=1 (CTRL_MODE = EN_SENSOR | EOI_CTRL)
        self.write_register(Register::CtrlMode, 0b11);

        // Wait 3ms
        thread::sleep(Duration::from_millis(3));

        //TODO The exemple start-up sequence could probably be simplified
        // We don't check read values, so it's probably useless

        // Read all status registers twice
        for _ in 0..2 {
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

        let _ = self.read_register(Register::CtrlUserIf);
        let _ = self.read_register(Register::CtrlAcc12);

        Ok(())
    }

    pub fn update_angle(&mut self) -> Result<(), Sch16tError<SPI>> {
        let x_stat = self.read_register(Register::StatRateX)?;
        let y_stat = self.read_register(Register::StatRateY)?;
        let z_stat = self.read_register(Register::StatRateZ)?;
        let new_instant = Instant::now();
        if let Some(previous_instant) = self.last_measure_instant.replace(new_instant) {
            let elapsed = (new_instant - previous_instant).as_secs_f32();

            let stat = Sch16tStatRate::from(x_stat as u16);
            if stat.all_flags_ok() {
                self.angle.x += self.angle_from_register(Register::RateX1)? * elapsed;
            } else {
                self.angle.x += self.angle_from_register(Register::RateX2)? * elapsed;
                return Err(Sch16tError::XReading(stat));
            }


            let stat = Sch16tStatRate::from(y_stat as u16);
            if stat.all_flags_ok() {
                self.angle.y += self.angle_from_register(Register::RateY1)? * elapsed;
            } else {
                self.angle.x += self.angle_from_register(Register::RateY2)? * elapsed;
                return Err(Sch16tError::YReading(stat));
            }

            let stat = Sch16tStatRate::from(z_stat as u16);
            if stat.all_flags_ok() {
                self.angle.z += self.angle_from_register(Register::RateZ1)? * elapsed;
            } else {
                self.angle.x += self.angle_from_register(Register::RateZ2)? * elapsed;
                return Err(Sch16tError::ZReading(stat));
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
        let frame = create_spi48bf(self.address, register, true, 0);

        let mut answer: [u8; 6] = [0x00; 6];
        // `transaction` asserts and deasserts CS for us. No need to do it manually!
        let _ = self.spi.transaction(&mut [Operation::Write(&frame)]);
        let _ = self.spi.transaction(&mut [Operation::Read(&mut answer)]);
        let parsed_answer = parse_spi48bf(answer);
        Ok(parsed_answer.data)
    }

    /// write a register to the SCH16T and return the write error status
    fn write_register(&mut self, register: Register, data: u32) -> bool {
        let frame = create_spi48bf(self.address, register, false, data);
        let mut answer: [u8; 6] = [0; 6];
        let _ = self.spi.transaction(&mut [Operation::Write(&frame)]);
        let _ = self.spi.transaction(&mut [Operation::Read(&mut answer)]);
        let parsed_answer = parse_spi48bf(answer);
        matches!(parsed_answer.status, SCH16TFrameStatus::Error)
    }
}


fn parse_spi48bf(raw: [u8; 6]) -> SPI48bRdFrame {
    let mut raw_u64 = [0; 8];
    for (i, &val) in raw.iter().enumerate() {
        raw_u64[i + 2] = val;
    }
    let full_frame = u64::from_be_bytes(raw_u64);
    // Received SPI 48-bit frame structure:
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

/// Create an SPI 48-bit frame
fn create_spi48bf(address: u8, register: Register, read: bool, data: u32) -> [u8; 6] {
    // Sent SPI 48-bit frame structure:
    // TA[9:0] RW_bit 0 1 AE[6:0] data[19:0] CRC8[7:0]
    let mut frame_value: u64 = 1 << 35; // Frame Type bit: 1 for 48-bit frame
    frame_value |= (address as u64) << 46; // TA[9:8]: chip select (2-bit address)
    frame_value |= (register as u64) << 38; // TA[7:0]: register address
    frame_value |= if read { 0 } else { 1 } << 37;
    frame_value |= ((data & 0xfffff) as u64) << 8;

    let mut frame_bytes: [u8; 6] = frame_value.to_be_bytes()[2..8].try_into().unwrap();
    frame_bytes[5] = compute_crc8(&frame_bytes[..5]);
    frame_bytes
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

    #[test]
    fn test_create_48b_frame() {
        let frame = create_spi48bf(0x0, Register::CtrlMode, false, 0x0000_0003);
        let expected_frame = [0x0D, 0x68, 0x00, 0x00, 0x03, 0x8D];
        assert_eq!(frame, expected_frame);
    }
}
