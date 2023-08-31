use embassy_stm32::peripherals::{PA0, PA1, PA2, PA8, PA9, PA10, PC6, PC7, PC8, USART3, PB14, PB12, PB13};

//led
pub type LedPin = PB14;

//inputs
pub type StepPin = PB12;
pub type DirPin = PB13;

//hall sensor definition
pub type UHallPin = PA0;
pub type VHallPin = PA1;
pub type WHallPin = PA2;

//motor drive definition
pub type UDrivePin = PA8;
pub type VDrivePin = PA9;
pub type WDrivePin = PA10;
pub type UEnablePin = PC6;
pub type VEnablePin = PC7;
pub type WEnablePin = PC8;

//uart
pub type DebugUart = USART3;