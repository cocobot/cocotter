#![no_std]

use embassy_stm32::{gpio::{Output, Input, Pull, Speed, Level}, exti::ExtiInput, usart::{UartTx, Config}, dma::NoDma};
use core::default::Default;

pub mod hardware;

pub struct Canon2019 {
    pub led: Option<Output<'static, hardware::LedPin>>,

    pub step_irq: Option<ExtiInput<'static, hardware::StepPin>>,
    pub dir: Option<Input<'static, hardware::DirPin>>,

    pub u_en: Option<Output<'static, hardware::UEnablePin>>,
    pub v_en: Option<Output<'static, hardware::VEnablePin>>,
    pub w_en: Option<Output<'static, hardware::WEnablePin>>,

    pub u_drive: Option<Output<'static, hardware::UDrivePin>>,
    pub v_drive: Option<Output<'static, hardware::VDrivePin>>,
    pub w_drive: Option<Output<'static, hardware::WDrivePin>>,

    pub uhall_irq: Option<ExtiInput<'static, hardware::UHallPin>>,
    pub vhall_irq: Option<ExtiInput<'static, hardware::VHallPin>>,
    pub whall_irq: Option<ExtiInput<'static, hardware::WHallPin>>,

    pub uart: Option<UartTx<'static, hardware::DebugUart, NoDma>>,
}

pub fn board_init() -> Canon2019 {
    let p = embassy_stm32::init(Default::default());

    Canon2019 { 
        led: Some(Output::new(p.PB14, Level::High, Speed::Low)),
        step_irq: Some(ExtiInput::new(Input::new(p.PB12, Pull::None), p.EXTI12)),
        dir: Some(Input::new(p.PB13, Pull::None)),
        u_en: Some(Output::new(p.PC6, Level::Low, Speed::VeryHigh)),
        v_en: Some(Output::new(p.PC7, Level::Low, Speed::VeryHigh)),
        w_en: Some(Output::new(p.PC8, Level::Low, Speed::VeryHigh)),
        u_drive: Some(Output::new(p.PA8, Level::Low, Speed::VeryHigh)),
        v_drive: Some(Output::new(p.PA9, Level::Low, Speed::VeryHigh)),
        w_drive: Some(Output::new(p.PA10, Level::Low, Speed::VeryHigh)),
        uhall_irq: Some(ExtiInput::new(Input::new(p.PA0, Pull::None), p.EXTI0)),
        vhall_irq: Some(ExtiInput::new(Input::new(p.PA1, Pull::None), p.EXTI1)),
        whall_irq: Some(ExtiInput::new(Input::new(p.PA2, Pull::None), p.EXTI2)),
        uart: Some(UartTx::new(p.USART3, p.PC10, NoDma, Config::default())),
    }
}