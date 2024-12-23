#![no_std]

// a lire : https://github.com/inazarenko/ssd1331-async/blob/main/src/lib.rs

use esp_hal::spi::master::Spi;
use core::ops::AddAssign;
use embassy_time::{Duration, Instant, Timer};

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

impl ODR{
    fn from(a : ODR) -> f32{
        match a{
            Self::OdrPowerDown  => 0.1,
            Self::Odr12_5Hz     => 12.5,
            Self::Odr26Hz       => 26.0,
            Self::Odr52Hz       => 52.0,
            Self::Odr104Hz      => 104.0,
            Self::Odr208Hz      => 208.0,
            Self::Odr416Hz      => 408.16,
            Self::Odr833Hz      => 833.0,
            Self::Odr1660Hz     => 1660.0,
            Self::Odr3330Hz     => 3300.0,
            Self::Odr6660Hz     => 6660.0,
        }
    }
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
        match a{ // see LA_So in datasheet
            Self::Fs2g => 0.000_061,
            Self::Fs4g => 0.000_122,
            Self::Fs8g => 0.000_244,
            Self::Fs16g=> 0.000_488,
        }
    }
}

#[derive(Clone)]
pub enum GyroFullScale {
    Fs250dps  = 0,
    Fs500dps  = 1,
    Fs1000dps = 2,
    Fs2000dps = 3,
}

impl GyroFullScale{
    fn from(a : GyroFullScale) -> f32{
        match a{ // see G_So  in datasheet
            Self::Fs250dps => 0.008_75,
            Self::Fs500dps => 0.017_50,
            Self::Fs1000dps=> 0.035_00,
            Self::Fs2000dps=> 0.070_00,
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
    fn from( a: [u8; 6], sensitivity:f32) -> Meas3d{
        Meas3d{
            x: (((a[1] as i16)<<8)| a[0] as i16) as f32 * sensitivity,
            y: (((a[3] as i16)<<8)| a[2] as i16) as f32 * sensitivity,
            z: (((a[5] as i16)<<8)| a[4] as i16) as f32 * sensitivity,
        }
    }

    fn divf32(a:Meas3d, other:f32)->Meas3d{
        Meas3d{
            x: a.x / other,
            y: a.y / other,
            z: a.z / other,
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
    angle           : Meas3d,
    temperature     : f32,
    timestamp       : Instant,
}

enum FifoMode {
    Disabled            = 0b000,
    FIFOMode            = 0b001,
    ContinuousToFIFO    = 0b011,
    BypassToContinuous  = 0b100,
    Continuous          = 0b110,
    BypassToFIFO        = 0b111,
}


//Specific implementation of a regular robot
impl Lsm6dso32x{
    
    const READ_OP_BV    : u8 = 0x80;
    const WRITE_OP_BV   : u8 = 0x00;
    const WHO_AM_I_DEF  : u8 = 0x6C;
    const ODR_BP        : u8 = 4;               // output data rate bit position in register CTRL1_XL
    const ODR_BM        : u8 = 0b1111_0000;     // output data rate bit mask in register CTRL1_XL
    const FIFO_GYRO_ODR_BP  : u8 =  4;          // gyro output data rate bit position in register FIFO_CTRL3
    const FIFO_GYRO_ODR_BM  : u8 = 0b1111_0000; // gyro output data rate bit mask in register FIFO_CTRL3
    const FS_BP         : u8 = 2;
    const FS_BM         : u8 = 0b0000_1100;
    const TAG_SENSOR_BP : u8 = 3;
    const TAG_SENSOR_BM : u8 = 0b1111_1000;
    //const STATUS_REG_GYRO_SETTLING_BM   : u8 = 0x04;   // temperature new data available bit in status reg
    const STATUS_REG_XLDA_BM    : u8 = 0b0000_0001;   // accelerometer new data available bit in status reg
    const STATUS_REG_GDA_BM     : u8 = 0b0000_0010;   // gyroscope new data available bit in status reg
    const STATUS_REG_TDA_BM     : u8 = 0b0000_0100;   // temperature new data available bit in status reg

    const CALIBRATION_PERIOD_MS : u64 = 20;

    const DEBUG_MODULE :bool= false;

    //pub fn new(configuration: Lsm6dso32xConfiguration, spi_dev: Spi<'static, SpiMode>, _cs: Output<'static>) -> Self {
    pub fn new(configuration: Lsm6dso32xConfiguration, spi_dev: Spi<'static, esp_hal::Blocking>) -> Self {
        Lsm6dso32x { 
            spi         : spi_dev,
            //cs          : cs,
            dev_conf    : configuration, 
            acceleration: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            angular_rate: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            angular_rate_offset: Meas3d::from([0,0, 0,0, 0,0], 1.0), 
            angle       : Meas3d::from([0,0, 0,0, 0,0], 1.0), 
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
        if Self::DEBUG_MODULE {log::info!("ctrl1 {:#02x}", ctrl1_xl_val[0]); };
        
        self.write_reg(RegistersAddr::Ctrl1Xl, &ctrl1_xl_val);

        // configure gyroscope data rate and full scale
        let mut ctrl2_g_val : [u8 ;1]  = [0x00];
        ctrl2_g_val[0] |= ((self.dev_conf.gyroscope.odr.clone() as u8)<< Lsm6dso32x::ODR_BP) & Lsm6dso32x::ODR_BM;
        ctrl2_g_val[0] |= ((self.dev_conf.gyroscope.full_scale.clone() as u8) << Lsm6dso32x::FS_BP) & Lsm6dso32x::FS_BM;
        if Self::DEBUG_MODULE {log::info!("ctrl2 {:#02x}", ctrl2_g_val[0]); };
        self.write_reg(RegistersAddr::Ctrl2G, &ctrl2_g_val);


        // configure fifo for gyro and accelerometer
        let mut fifo_ctrl_3_val : [u8 ;1] = [0x00];
        fifo_ctrl_3_val[0] |= (self.dev_conf.accelerometer.odr.clone() as u8) & !Lsm6dso32x::FIFO_GYRO_ODR_BM;
        fifo_ctrl_3_val[0] |= ((self.dev_conf.gyroscope.odr.clone() as u8) << Lsm6dso32x::FIFO_GYRO_ODR_BP) & Lsm6dso32x::FIFO_GYRO_ODR_BM;
        if Self::DEBUG_MODULE {log::info!("fifo ctrl3 {:#02x}", fifo_ctrl_3_val[0]); };
        self.write_reg(RegistersAddr::FifoCtrl3, &fifo_ctrl_3_val);

        let mut fifo_ctrl_4_val : [u8 ;1] = [0x00];
        fifo_ctrl_4_val[0] |= FifoMode::FIFOMode as u8; // timestamp and temperature not batched
        if Self::DEBUG_MODULE {log::info!("fifo ctrl4 {:#02x}", fifo_ctrl_4_val[0]); };
        self.write_reg(RegistersAddr::FifoCtrl4, &fifo_ctrl_4_val);

        let mut angular_rate = Meas3d::from([0,0, 0,0, 0,0], 1.0);

        if Self::DEBUG_MODULE {log::info!("calibrating lsm6dso32x"); };
        for _it in 0..20{
            self.update_measures(); // update but throw first measures
            Timer::after(Duration::from_millis(Self::CALIBRATION_PERIOD_MS)).await;
        }

        for _it in 0..200{
            //log::info!("meas {}, {}, {}", angular_rate.x, angular_rate.y, angular_rate.z);
            self.update_measures();
            angular_rate += self.get_angular_rate();
            //run the asserv at 20Hz
            Timer::after(Duration::from_millis(Self::CALIBRATION_PERIOD_MS)).await;
            
        }
        angular_rate = Meas3d::divf32(angular_rate, 200.0);
        
        if Self::DEBUG_MODULE {log::info!("cal div {}, {}, {}", angular_rate.x, angular_rate.y, angular_rate.z); };
        self.angular_rate_offset = angular_rate;
        //todo correct it
        
        self.angle =  Meas3d::from([0,0, 0,0, 0,0], 1.0);
    }

    fn read_reg(&mut self,_register_add : RegistersAddr, data_out: &mut [u8]){
        //first  : send reg and read bit and complete with 2 bytes (register )
        let mut reg_cmd : [u8 ; 8] =  [0xff; 8];
        reg_cmd[0] = Lsm6dso32x::READ_OP_BV | (_register_add as u8);
        data_out.copy_from_slice(self.spi.transfer(&mut reg_cmd).unwrap()[1..=data_out.len()].as_ref());
    }

    fn write_reg(&mut self,_register_add : RegistersAddr, data_in : &[u8]){
        let mut reg_cmd : [u8 ; 8] =  [0xff; 8];
        reg_cmd[0] = Lsm6dso32x::WRITE_OP_BV | (_register_add as u8);
        reg_cmd[1..(data_in.len() + 1)].copy_from_slice(data_in);
        self.spi.write_bytes(&reg_cmd[0..=data_in.len()]).unwrap();
    }

    pub fn update_measures(&mut self){
        let mut fifo_status_val : [u8;2] = [0x00;2];
        self.read_reg(RegistersAddr::FifoStatus1, &mut fifo_status_val);

        let diff_fifo :u16 = (((fifo_status_val[1] as u16) <<8) | (fifo_status_val[0] as u16)) & 0b0000_0011_1111_1111;
        self.acceleration = Meas3d::from([0;6], 1.0);
        self.angular_rate = Meas3d::from([0;6], 1.0);

        //log::info!("status 2 : {:#02x}, diff io {}", fifo_status_val[1], diff_fifo);
        if (fifo_status_val[1] & 0b1111_1000) != 0
        {
            log::info!("status {:#02x}", fifo_status_val[1]);
        }

        let mut gy_update_nb : u16 = 0;
        for _i in 0..diff_fifo
        {
            let mut val : [u8; 7] = [0x00; 7];
            self.read_reg(RegistersAddr::FifoDataOutTag, &mut val);
            //log::info!("tag : {:#02x}", val[0]);

            match (val[0] & Lsm6dso32x::TAG_SENSOR_BM) >> Lsm6dso32x::TAG_SENSOR_BP {
                0x02 =>{
                    let mut xl_val : [u8;6]= [0;6];
                    xl_val.copy_from_slice(&val[1..=6]);
                    self.acceleration += Meas3d::from(xl_val, AccelerometerFullScale::from(self.dev_conf.accelerometer.full_scale.clone()));                    
                }
                0x01 =>{
                    let mut gy_val : [u8;6] = [0;6];
                    gy_val.copy_from_slice(&val[1..=6]);
                    self.angular_rate += Meas3d::from(gy_val, GyroFullScale::from(self.dev_conf.gyroscope.full_scale.clone()));
                    gy_update_nb +=1;
                    self.angle+= Meas3d::divf32(Meas3d::from(gy_val, GyroFullScale::from(self.dev_conf.gyroscope.full_scale.clone())), ODR::from(self.dev_conf.gyroscope.odr.clone()));
                }
                0x04 => {
                    log::info!("{:#02x} {:#02x} {:#02x} {:#02x} {:#02x} {:#02x}",val[1], val[2], val[3], val[4], val[5], val[6])
                }
                key => log::info!("unknown {key}")
            };
            //log::info!("gy {} {} {}", self.angular_rate.x, self.angular_rate.y, self.angular_rate.z);
            self.timestamp = Instant::now();
        }
        if gy_update_nb >0
        {
            self.angular_rate = Meas3d::divf32(self.angular_rate.clone(), gy_update_nb as f32);
        }
        let status_reg_val : [u8;1] = [0x00; 1];
        self.read_reg(RegistersAddr::StatusReg, &mut fifo_status_val);
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

    pub fn get_angle(&self) -> Meas3d{
        self.angle.clone()
    }

}
