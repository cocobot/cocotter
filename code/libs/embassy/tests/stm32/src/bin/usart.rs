#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;
use defmt::assert_eq;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::interrupt;
use embassy_stm32::usart::{Config, Uart};
use example_common::*;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config());
    info!("Hello World!");

    // Arduino pins D0 and D1
    // They're connected together with a 1K resistor.
    #[cfg(feature = "stm32f103c8")]
    let (tx, rx, usart, irq) = (p.PA9, p.PA10, p.USART1, interrupt::take!(USART1));
    #[cfg(feature = "stm32g491re")]
    let (tx, rx, usart, irq) = (p.PC4, p.PC5, p.USART1, interrupt::take!(USART1));
    #[cfg(feature = "stm32g071rb")]
    let (tx, rx, usart, irq) = (p.PC4, p.PC5, p.USART1, interrupt::take!(USART1));
    #[cfg(feature = "stm32f429zi")]
    let (tx, rx, usart, irq) = (p.PG14, p.PG9, p.USART6, interrupt::take!(USART6));
    #[cfg(feature = "stm32wb55rg")]
    let (tx, rx, usart, irq) = (p.PA2, p.PA3, p.LPUART1, interrupt::take!(LPUART1));
    #[cfg(feature = "stm32h755zi")]
    let (tx, rx, usart, irq) = (p.PB6, p.PB7, p.USART1, interrupt::take!(USART1));
    #[cfg(feature = "stm32u585ai")]
    let (tx, rx, usart, irq) = (p.PD8, p.PD9, p.USART3, interrupt::take!(USART3));
    #[cfg(feature = "stm32h563zi")]
    let (tx, rx, usart, irq) = (p.PB6, p.PB7, p.LPUART1, interrupt::take!(LPUART1));
    #[cfg(feature = "stm32c031c6")]
    let (tx, rx, usart, irq) = (p.PB6, p.PB7, p.USART1, interrupt::take!(USART1));

    let config = Config::default();
    let mut usart = Uart::new(usart, rx, tx, irq, NoDma, NoDma, config);

    // We can't send too many bytes, they have to fit in the FIFO.
    // This is because we aren't sending+receiving at the same time.

    let data = [0xC0, 0xDE];
    usart.blocking_write(&data).unwrap();

    let mut buf = [0; 2];
    usart.blocking_read(&mut buf).unwrap();
    assert_eq!(buf, data);

    info!("Test OK");
    cortex_m::asm::bkpt();
}
