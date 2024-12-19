#![no_std]

// a lire : https://github.com/inazarenko/ssd1331-async/blob/main/src/lib.rs

use esp_hal::spi::master::Spi;
use core::ops::AddAssign;
use embassy_time::{Duration, Instant, Timer};
use log::info;


#[allow(dead_code)]
enum RegistersAddr {
    RegFuncCfgAccess=0x01,  //R/W
    PinCtrl=0x02,           //R/W
    FifoCtrl1=0x07,         //R/W
    FifoCtrl2=0x08,         //R/W
    FifoCtrl3=0x09,         //R/W
    FifoCtrl4=0x0a,         //R/W
    CounterBdrReg1=0x0b,    //R/W
    CounterBdrReg2=0x0c,    //R/W
    Int1Ctrl=0x0d,          //R/W
    Int2Ctrl=0x0e,          //R/W
    WhoAmI=0x0f,            //R
    Ctrl1Xl=0x10,           //R/W
    Ctrl2G=0x11,            //R/W
    Ctrl3C=0x12,            //R/W
    Ctrl4C=0x13,            //R/W
    Ctrl5C=0x14,            //R/W
    Ctrl6C=0x15,            //R/W
    Ctrl7G=0x16,            //R/W
    Ctrl8Xl=0x17,           //R/W
    Ctrl9Xl=0x18,           //R/W
    Ctrl10C=0x19,           //R/W
    AllIntSrc=0x1a,         //R
    WakeUpSrc=0x1b,         //R
    TapSrc=0x1c,            //R
    D6dSrc=0x1d,            //R
    StatusReg=0x1e,         //R
    OutTempL=0x20,          //R
    OutTempH=0x21,          //R
    OutxLG=0x22,            //R
    OutxHG=0x23,            //R
    OutyLG=0x24,            //R
    OutyHG=0x25,            //R
    OutzLG=0x26,            //R
    OutzHG=0x27,            //R
    OutxLA=0x28,            //R
    OutxHA=0x29,            //R
    OutyLA=0x2a,            //R
    OutyHA=0x2b,            //R
    OutzLA=0x2c,            //R
    OutzHA=0x2d,            //R
    EmbFuncStatusMainpage=0x35, //R
    FsmStatusAMainpage=0x36,//R
    FsmStatusBMainpage=0x37,//R
    StatusMasterMainpage=0x39,  //R
    FifoStatus1=0x3a,   //R
    FifoStatus2=0x3b,   //R
    Timestamp0=0x40,    //R
    Timestamp1=0x41,    //R
    Timestamp2=0x42,    //R
    Timestamp3=0x43,    //R
    TapCfg0=0x56,//R/W
    TapCfg1=0x57,//R/W
    TapCfg2=0x58,//R/W
    TapThs6d=0x59,//R/W
    IntDur2=0x5a,//R/W
    WakeUpThs=0x5b,//R/W
    WakeUpDur=0x5c,//R/W
    FreeFall=0x5d,//R/W
    Md1Cfg=0x5e,//R/W
    Md2Cfg=0x5f,//R/W
    I3cBusAvb=0x62,//R/W
    InternalFreqFine=0x63,//R
    IntOis=0x6f,//R
    Ctrl1Ois=0x70,//R
    Ctrl2Ois=0x71,//R
    Ctrl3Ois=0x72,//R
    XOfsUsr=0x73,//R/W
    YOfsUsr=0x74,//R/W
    ZOfsUsr=0x75,//R/W
    FifoDataOutTag=0x78,//R
    FifoDataOutXL=0x79,//R
    FifoDataOutXH=0x7a,//R
    FifoDataOutYL=0x7b,//R
    FifoDataOutYH=0x7c,//R
    FifoDataOutZL=0x7d,//R
    FifoDataOutZH=0x7e//R
}

#[derive(Clone)]
// lsm6dso registers value definitions
pub enum ODR {
    OdrPowerDown  = 0,
    Odr12_5Hz  = 1,
    Odr26Hz    = 2,
    Odr52Hz    = 3,
    Odr104Hz   = 4,
    Odr208Hz   = 5,
    Odr416Hz   = 6,
    Odr833Hz   = 7,
    Odr1660Hz  = 8,
    Odr3330Hz  = 9,
    Odr6660Hz  = 10
}

#[derive(Clone)]
pub enum AccelerometerFullScale {
    Fs2g  = 0,
    Fs16g = 1,
    Fs4g  = 2,
    Fs8g  = 3,
}

impl AccelerometerFullScale{
    fn from(a : AccelerometerFullScale) -> f32{
        match a{
            Self::Fs2g => 2.0,
            Self::Fs4g => 4.0,
            Self::Fs8g => 8.0,
            Self::Fs16g=> 16.0,
        }
    }
}

#[derive(Clone)]
pub enum GyroFullScale {
    Fs125dps  = 1,
    Fs250dps  = 0,
    Fs500dps  = 2,
    Fs1000dps = 4,
    Fs2000dps = 6,
}

impl GyroFullScale{
    fn from(a : GyroFullScale) -> f32{
        match a{
            Self::Fs125dps => 125.0,
            Self::Fs250dps => 250.0,
            Self::Fs500dps => 500.0,
            Self::Fs1000dps=> 1000.0,
            Self::Fs2000dps=> 2000.0,
        }
    }
}

// end of lsm6dso registers value definitions

#[derive(Clone)]
pub struct Meas3d {
    pub x : f32,
    pub y : f32,
    pub z : f32,
}

impl Meas3d{
    fn from( a: [u8; 6], full_scale:f32) -> Meas3d{
        Meas3d{
            x: (((a[1] as i16)<<8)| a[0] as i16) as f32 * full_scale / (i16::MAX as f32) ,
            y: (((a[3] as i16)<<8)| a[2] as i16) as f32 * full_scale / (i16::MAX as f32),
            z: (((a[5] as i16)<<8)| a[4] as i16) as f32 * full_scale / (i16::MAX as f32),
        }
    }
}

impl AddAssign for Meas3d{
    fn add_assign(&mut self, other:Self){
        *self = Self{
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        };
    }
}

pub struct AccelerometerConfiguration {
    pub odr : ODR,
    pub full_scale : AccelerometerFullScale,
}

pub struct GyroscopeConfiguration {
    pub odr : ODR,
    pub full_scale : GyroFullScale,
}

pub struct Lsm6dso32xConfiguration{
    pub accelerometer : AccelerometerConfiguration,
    pub gyroscope : GyroscopeConfiguration,
}

pub struct Lsm6dso32x {
    //cs              : Output<'static>,
    spi             : Spi<'static, esp_hal::Blocking>,
    dev_conf        : Lsm6dso32xConfiguration,
    acceleration    : Meas3d,
    angular_rate    : Meas3d,
    angular_rate_offset: Meas3d,
    temperature     : f32,
    timestamp       : Instant,
}



//Specific implementation of a regular robot
impl Lsm6dso32x{
    
    const READ_OP_BV    : u8 = 0x80;
    const WRITE_OP_BV   : u8 = 0x00;
    const WHO_AM_I_DEF  : u8 = 0x6C;
    const ODR_BP        : u8 = 4;      // output data rate bit position in register CTRL1_XL
    const ODR_BM        : u8 = 0b1111_0000;   // output data rate bit mask  in register CTRL1_XL
    const FS_BP         : u8 = 2;
    const FS_BM         : u8 = 0b0000_1100;   
    //const STATUS_REG_GYRO_SETTLING_BM   : u8 = 0x04;   // temperature new data available bit in status reg
    const STATUS_REG_XLDA_BM    : u8 = 0b0000_0001;   // accelerometer new data available bit in status reg
    const STATUS_REG_GDA_BM     : u8 = 0b0000_0010;   // gyroscope new data available bit in status reg
    const STATUS_REG_TDA_BM     : u8 = 0b0000_0100;   // temperature new data available bit in status reg

    const CALIBRATION_PERIOD_MS : u64 = 20;

    //pub fn new(configuration: Lsm6dso32xConfiguration, spi_dev: Spi<'static, SpiMode>, _cs: Output<'static>) -> Self {
    pub fn new(configuration: Lsm6dso32xConfiguration, spi_dev: Spi<'static, esp_hal::Blocking>) -> Self {
        Lsm6dso32x { 
            spi         : spi_dev,
            //cs          : cs,
            dev_conf    : configuration, 
            acceleration: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            angular_rate: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            angular_rate_offset: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            temperature : 0.0, 
            timestamp : Instant::now(),
        }
    }

    pub async fn init(&mut self) {
        // check device identity
        let mut who_am_i_val : [u8;1] = [0xff];
        self.read_reg(RegistersAddr::WhoAmI, &mut who_am_i_val);
        if who_am_i_val[0] != Lsm6dso32x::WHO_AM_I_DEF
        {
            //todo : handle undetection of device
        }

        // reset device configuration
        let ctrl3_c_val : [u8 ;1] = [0b0100_0101];
        self.write_reg(RegistersAddr::Ctrl3C, &ctrl3_c_val);

        // reset device configuration
        let ctrl3_c_val : [u8 ;1] = [0b0100_0100];
        self.write_reg(RegistersAddr::Ctrl3C, &ctrl3_c_val);
        
        // configure accelerometer data rate and full scale
        let mut ctrl1_xl_val : [u8 ;1]  = [0x00];
        ctrl1_xl_val[0] |= ((self.dev_conf.accelerometer.odr.clone() as u8) << Lsm6dso32x::ODR_BP) & Lsm6dso32x::ODR_BM;
        ctrl1_xl_val[0] |= ((self.dev_conf.accelerometer.full_scale.clone() as u8)<< Lsm6dso32x::FS_BP) & Lsm6dso32x::FS_BM;
        self.write_reg(RegistersAddr::Ctrl1Xl, &ctrl1_xl_val);

        // configure gyroscope data rate and full scale
        let mut ctrl2_g_val : [u8 ;1]  = [0x00];
        ctrl2_g_val[0] |= ((self.dev_conf.gyroscope.odr.clone() as u8)<< Lsm6dso32x::ODR_BP) & Lsm6dso32x::ODR_BM;
        ctrl2_g_val[0] |= ((self.dev_conf.gyroscope.full_scale.clone() as u8) << Lsm6dso32x::FS_BP) & Lsm6dso32x::FS_BM;
        self.write_reg(RegistersAddr::Ctrl2G, &ctrl2_g_val);

        let mut angular_rate = Meas3d::from([0,0, 0,0, 0,0], 1.0);

        log::info!("calibrating lsm6dso32x");
        for _it in 0..20{
            self.update_measures(); // update but throw first measures
            Timer::after(Duration::from_millis(Self::CALIBRATION_PERIOD_MS)).await;
        }

        for _it in 0..200{
            log::info!("meas {}, {}, {}", angular_rate.x, angular_rate.y, angular_rate.z);
            self.update_measures();
            angular_rate += self.get_angular_rate();
            //run the asserv at 20Hz
            Timer::after(Duration::from_millis(Self::CALIBRATION_PERIOD_MS)).await;
            
        }
        log::info!("cal tot {}, {}, {}", angular_rate.x, angular_rate.y, angular_rate.z);
        angular_rate.x = -angular_rate.x/200.0;
        angular_rate.y = -angular_rate.y/200.0;
        angular_rate.z = -angular_rate.z/200.0;
        log::info!("cal div {}, {}, {}", angular_rate.x, angular_rate.y, angular_rate.z);
        self.angular_rate_offset = angular_rate;
        //todo correct it

    }

    fn read_reg(&mut self,_register_add : RegistersAddr, data_out: &mut [u8]){
        //first  : send reg and read bit and complete with 2 bytes (register )
        let mut reg_cmd : [u8 ; 7] =  [0xff; 7];
        reg_cmd[0] = Lsm6dso32x::READ_OP_BV | (_register_add as u8);
        data_out.copy_from_slice(self.spi.transfer(&mut reg_cmd).unwrap()[1..(data_out.len() + 1)].as_ref());
    }

    fn write_reg(&mut self,_register_add : RegistersAddr, data_in : &[u8]){
        let mut reg_cmd : [u8 ; 7] =  [0xff; 7];
        reg_cmd[0] = Lsm6dso32x::WRITE_OP_BV | (_register_add as u8);
        reg_cmd[1..(data_in.len() + 1)].copy_from_slice(data_in);
        self.spi.write_bytes(&reg_cmd[0..(data_in.len() + 1)]).unwrap();
    }

    pub fn update_measures(&mut self){
        let mut status_reg_val : [u8;1] = [0x00];
        self.read_reg(RegistersAddr::StatusReg, &mut status_reg_val);

        if status_reg_val[0] & (Lsm6dso32x::STATUS_REG_GDA_BM | Lsm6dso32x::STATUS_REG_XLDA_BM) == (Lsm6dso32x::STATUS_REG_GDA_BM | Lsm6dso32x::STATUS_REG_XLDA_BM)
        {
            let mut xl_val : [u8 ;6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            self.read_reg(RegistersAddr::OutxLG, &mut xl_val);
            self.angular_rate = Meas3d::from(xl_val, GyroFullScale::from(self.dev_conf.gyroscope.full_scale.clone()));
            self.angular_rate += self.angular_rate_offset.clone();

            let mut angle_rate_val : [u8 ;6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            self.read_reg(RegistersAddr::OutxLA, &mut angle_rate_val);
            self.acceleration = Meas3d::from(angle_rate_val, AccelerometerFullScale::from(self.dev_conf.accelerometer.full_scale.clone()));

            self.timestamp = Instant::now();
        }
        if status_reg_val[0] & Lsm6dso32x::STATUS_REG_TDA_BM == Lsm6dso32x::STATUS_REG_TDA_BM
        {
                        
            let mut temp_val : [u8 ;2] = [0x00, 0x00];
            self.read_reg(RegistersAddr::OutTempL, &mut temp_val);
            let raw_temp: i16 =  (((temp_val[1] as u16) <<8) | temp_val[0] as u16)as i16;
            self.temperature = 25.0 + ((raw_temp as f32) /256.0);
        }
    }

    pub fn get_acceleration(&self) -> Meas3d{
        self.acceleration.clone()
    }
    
    pub fn get_angular_rate(&self) -> Meas3d{
        self.angular_rate.clone()
    }

    pub fn get_measure_timestamp(&self) -> Instant{
        self.timestamp.clone()
    }

    pub fn get_temperature_degc(&self) -> f32{
        self.temperature.clone()
    }
}
