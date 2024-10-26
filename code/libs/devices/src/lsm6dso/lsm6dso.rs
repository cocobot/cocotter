use embedded_hal::spi;

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
enum ODR {
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
enum AccelerometerFullScale {
    Fs2g  = 0,
    Fs16g = 1,
    Fs4g  = 2,
    Fs8g  = 3
}

#[derive(Clone)]
enum GyroFullScale {
    Fs125dps  = 1,
    Fs250dps  = 0,
    Fs500dps  = 2,
    Fs1000dps = 4,
    Fs2000dps = 6
}

// end of lsm6dso registers value definitions

#[derive(Clone)]
pub struct Meas3d {
    x : i16,
    y : i16,
    z : i16,
}

impl Meas3d{
    fn from( a: [u8; 6]) -> Meas3d{
        Meas3d{
            x: ((a[1] as i16)<<8)| a[0] as i16,
            y: ((a[3] as i16)<<8)| a[2] as i16,
            z: ((a[5] as i16)<<8)| a[4] as i16,
        }
    }

}

struct AccelerometerConfiguration {
    odr : ODR,
    full_scale : AccelerometerFullScale,
}

struct GyroscopeConfiguration {
    odr : ODR,
    full_scale : GyroFullScale,
}

pub struct Lsm6dsoConfiguration{
    accelerometer : AccelerometerConfiguration,
    gyroscope : GyroscopeConfiguration,
}

pub struct Lsm6dso {
    spiDev : spi::SpiDriver,
    devConf : Lsm6dsoConfiguration,
    acceleration : Meas3d,
    angularRate : Meas3d,
    temperature : u16,
    previousTimestamp_us : u64
}



//Specific implementation of a regular robot
impl Lsm6dso {

    const READ_OP_bv    : u8 = 0x80;
    const WRITE_OP_bv   : u8 = 0x00;
    const WHO_AM_I_def  : u8 = 0x6C;
    const ODR_bp        : u8 = 4;      // output data rate bit position in register CTRL1_XL
    const ODR_bm        : u8 = 0x0f << 4;   // output data rate bit mask  in register CTRL1_XL
    const FS_bp         : u8 = 2;
    const FS_bm         : u8 = 0x03 << 2;   
    const STATUS_REG_GYRO_SETTLING_bm   : u8 = 0x04;   // temperature new data available bit in status reg
    const STATUS_REG_XLDA_bm    : u8 = 0x01;   // accelerometer new data available bit in status reg
    const STATUS_REG_GDA_bm     : u8 = 0x02;   // gyroscope new data available bit in status reg

    pub fn new(configuration: Lsm6dsoConfiguration, spi_dev: spi::SpiDriver) -> Lsm6dso {
        Lsm6dso { 
            spiDev : spi_dev,
            devConf : configuration, 
            acceleration: Meas3d::from([0,0, 0,0, 0,0]), 
            angularRate: Meas3d::from([0,0, 0,0, 0,0]), 
            temperature : u16::MAX, 
            previousTimestamp_us : u64::MAX
        }
    }

    pub fn init(&self) {
        // check device identity
        let mut who_am_i_val : [u8;1] = [0xff];
        Lsm6dso::read(RegistersAddr::WhoAmI, &mut who_am_i_val);
        if who_am_i_val[0] != Lsm6dso::WHO_AM_I_def
        {
            //todo : handle undetection of device
        }

        // reset device configuration
        let ctrl3_c_val : [u8 ;1] = [0x01];
        Lsm6dso::write(RegistersAddr::Ctrl3C, &ctrl3_c_val);

        // configure accelerometer data rate and full scale
        let mut ctrl1_xl_val : [u8 ;1]  = [0x00];
        ctrl1_xl_val[0] |= ((self.devConf.accelerometer.odr.clone() as u8) << Lsm6dso::ODR_bp) & Lsm6dso::ODR_bm;
        ctrl1_xl_val[0] |= ((self.devConf.accelerometer.full_scale.clone() as u8)<< Lsm6dso::FS_bp) & Lsm6dso::FS_bm;
        Lsm6dso::write(RegistersAddr::Ctrl1Xl, &ctrl1_xl_val);

        // configure gyroscope data rate and full scale
        let mut ctrl2_g_val : [u8 ;1]  = [0x00];
        ctrl2_g_val[0] |= ((self.devConf.gyroscope.odr.clone() as u8)<< Lsm6dso::ODR_bp) & Lsm6dso::ODR_bm;
        ctrl2_g_val[0] |= ((self.devConf.gyroscope.full_scale.clone() as u8) << Lsm6dso::FS_bp) & Lsm6dso::FS_bm;
        Lsm6dso::write(RegistersAddr::Ctrl2G, &ctrl2_g_val);

    }

    fn read_and_write(_register_add : RegistersAddr, _data_in : &[u8], _data_out: &mut [u8]){

    }


    fn read(_register_add : RegistersAddr, _data_out: &mut [u8]){

    }

    fn write(_register_add : RegistersAddr, _data_in : &[u8]){

    }

    pub fn update_measures(&mut self){
        let mut status_reg_val : [u8;1] = [0x00];
        Lsm6dso::read(RegistersAddr::StatusReg, &mut status_reg_val);

        if status_reg_val[0] & (Lsm6dso::STATUS_REG_GDA_bm | Lsm6dso::STATUS_REG_XLDA_bm) == (Lsm6dso::STATUS_REG_GDA_bm | Lsm6dso::STATUS_REG_XLDA_bm)
           {
                let mut temp_val : [u8 ;2] = [0x00, 0x00];
                Lsm6dso::read(RegistersAddr::OutTempL, &mut temp_val);

                let mut xl_val : [u8 ;6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                Lsm6dso::read(RegistersAddr::OutxLG, &mut xl_val);
                self.acceleration = Meas3d::from(xl_val);

                let mut angle_rate_val : [u8 ;6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                Lsm6dso::read(RegistersAddr::OutxLA, &mut angle_rate_val);
                self.angularRate = Meas3d::from(angle_rate_val);

                // todo : update timestamp
           }
    }

    pub fn get_acceleration(&self) -> Meas3d{
        self.acceleration.clone()
    }

    
    pub fn get_angular_rate(&self) -> Meas3d{
        self.angularRate.clone()
    }
}