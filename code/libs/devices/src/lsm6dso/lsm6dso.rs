use embedded_hal::spi

enum RegistersAddr {
    FUNC_CFG_ACCESS         = 0x01,   // R/W
    PIN_CTRL                = 0x02,   // R/W
    FIFO_CTRL1              = 0x07,   // R/W
    FIFO_CTRL2              = 0x08,   // R/W
    FIFO_CTRL3              = 0x09,   // R/W
    FIFO_CTRL4              = 0x0A,   // R/W
    COUNTER_BDR_REG1        = 0x0B,   // R/W
    COUNTER_BDR_REG2        = 0x0C,   // R/W
    INT1_CTRL               = 0x0D,   // R/W
    INT2_CTRL               = 0x0E,   // R/W
    WHO_AM_I                = 0x0F,   // R
    CTRL1_XL                = 0x10,   // R/W
    CTRL2_G                 = 0x11,   // R/W
    CTRL3_C                 = 0x12,   // R/W
    CTRL4_C                 = 0x13,   // R/W
    CTRL5_C                 = 0x14,   // R/W
    CTRL6_C                 = 0x15,   // R/W
    CTRL7_G                 = 0x16,   // R/W
    CTRL8_XL                = 0x17,   // R/W
    CTRL9_XL                = 0x18,   // R/W
    CTRL10_C                = 0x19,   // R/W
    ALL_INT_SRC             = 0x1A,   // R
    WAKE_UP_SRC             = 0x1B,   // R
    TAP_SRC                 = 0x1C,   // R
    D6D_SRC                 = 0x1D,   // R
    STATUS_REG              = 0x1E,   // /
    OUT_TEMP_L              = 0x20,   // R
    OUT_TEMP_H              = 0x21,   // R
    OUTX_L_G                = 0x22,   // R
    OUTX_H_G                = 0x23,   // R
    OUTY_L_G                = 0x24,   // R
    OUTY_H_G                = 0x25,   // R
    OUTZ_L_G                = 0x26,   // R
    OUTZ_H_G                = 0x27,   // R
    OUTX_L_A                = 0x28,   // R
    OUTX_H_A                = 0x29,   // R
    OUTY_L_A                = 0x2A,   // R
    OUTY_H_A                = 0x2B,   // R
    OUTZ_L_A                = 0x2C,   // R
    OUTZ_H_A                = 0x2D,   // R
    EMB_FUNC_STATUS_MAINPAGE= 0x35,   // R
    FSM_STATUS_A_MAINPAGE   = 0x36,   // R
    FSM_STATUS_B_MAINPAGE   = 0x37,   // R
    STATUS_MASTER_MAINPAGE  = 0x39,   // R
    FIFO_STATUS1            = 0x3A,   // R
    FIFO_STATUS2            = 0x3B,   // R
    TIMESTAMP0              = 0x40,   // R
    TIMESTAMP1              = 0x41,   // R
    TIMESTAMP2              = 0x42,   // R
    TIMESTAMP3              = 0x43,   // R
    TAP_CFG0                = 0x56,   // R/W
    TAP_CFG1                = 0x57,   // R/W
    TAP_CFG2                = 0x58,   // R/W
    TAP_THS_6D              = 0x59,   // R/W
    INT_DUR2                = 0x5A,   // R/W
    WAKE_UP_THS             = 0x5B,   // R/W
    WAKE_UP_DUR             = 0x5C,   // R/W
    FREE_FALL               = 0x5D,   // R/W
    MD1_CFG                 = 0x5E,   // R/W
    MD2_CFG                 = 0x5F,   // R/W
    I3C_BUS_AVB             = 0x62,   // R/W
    INTERNAL_FREQ_FINE      = 0x63,   // R
    INT_OIS                 = 0x6F,   // R
    CTRL1_OIS               = 0x70,   // R
    CTRL2_OIS               = 0x71,   // R
    CTRL3_OIS               = 0x72,   // R
    X_OFS_USR               = 0x73,   // R/W
    Y_OFS_USR               = 0x74,   // R/W
    Z_OFS_USR               = 0x75,   // R/W
    FIFO_DATA_OUT_TAG       = 0x78,   // R
    FIFO_DATA_OUT_X_L       = 0x79,   // R
    FIFO_DATA_OUT_X_H       = 0x7A,   // R
    FIFO_DATA_OUT_Y_L       = 0x7B,   // R
    FIFO_DATA_OUT_Y_H       = 0x7C,   // R
    FIFO_DATA_OUT_Z_L       = 0x7D,   // R
    FIFO_DATA_OUT_Z_H       = 0x7E    // R
}

// lsm6dso registers value definitions
enum ODR
{
    odr_power_down  = 0,
    odr_12_5Hz  = 1,
    odr_26Hz    = 2,
    odr_52Hz    = 3,
    odr_104Hz   = 4,
    odr_208Hz   = 5,
    odr_416Hz   = 6,
    odr_833Hz   = 7,
    odr_1660Hz  = 8,
    odr_3330Hz  = 9,
    odr_6660Hz  = 10
}

enum AccelerometerFullScale {
    
    fs_2g  = 0,
    fs_16g = 1,
    fs_4g  = 2,
    fs_8g  = 3
}

enum GyroFullScale {
    fs_125dps  = 1
    fs_250dps  = 0,
    fs_500dps  = 2,
    fs_1000dps = 4,
    fs_2000dps = 6
}

// end of lsm6dso registers value definitions

struct Meas3d{
    x : i16,
    y : i16,
    z : i16
}

struct AccelerometerConfiguration {
    odr : ODR,
    full_scale : AccelerometerFullScale,
}

struct GyroscopeConfiguration {
    odr : ODR,
    full_scale : GyroFullScale,
}

struct Lsm6dsoConfiguration{
    accelerometer : AccelerometerConfiguration,
    gyroscope : GyroscopeConfiguration,
}

struct Lsm6dso {
    accelleration : Meas3d,
    angular_rate : Meas3d,
    temperature : u16,
    previous_timestamp_us : u64
}