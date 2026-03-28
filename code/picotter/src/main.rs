#![no_std]
#![no_main]
#![allow(dead_code)]

mod arm;
mod can_handler;
mod can_logger;
mod can_protocol;
mod color_sensor;
mod ground_sensors;
mod i2c_devices;
mod lidar;
mod module;
mod ota_handler;
mod scs0009;

use embassy_executor::Spawner;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::can::OperatingMode;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{
    BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig, HalfDuplexReadback,
};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use log::{info, warn};
use panic_rtt_target as _;
use rtt_target::rprintln;
use rtt_target::rtt_init_print;

use can_handler::{cmd_receiver, log_sender, status_sender};
use can_protocol::{ArmTarget, CanMessage, Domain, ServoBus};
use i2c_devices::I2cDevices;
use module::Module;
use scs0009::Scs0009;

bind_interrupts!(struct Irqs {
    // I2C
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C2>;
    I2C3_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C3>;
    // UART for servos
    USART11 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART11>;
    UART5 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::UART5>;
    USART6 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART6>;
    USART1 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART1>;
    // Lidar UARTs (19200 baud, full-duplex)
    USART3 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART3>;
    USART2 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART2>;
    UART7 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::UART7>;
    UART4 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::UART4>;
    USART10 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART10>;
    UART9 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::UART9>;
    // CAN
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN1>;
});

// Servo IDs for each module (4 servos per module)
const MODULE0_SERVO_IDS: [u8; 4] = [10, 11, 12, 1];
const MODULE1_SERVO_IDS: [u8; 4] = [10, 11, 12, 13];
const MODULE2_SERVO_IDS: [u8; 4] = [10, 11, 12, 13];

// Translation servo IDs (one per module, on shared bus)
const TRANSLATION_SERVO_IDS: [u8; 3] = [10, 11, 12];

// Concrete types
type I2cType = I2c<'static, Async, i2c::Master>;
type ModuleType = Module<
    BufferedUartTx<'static>,
    BufferedUartRx<'static>,
    I2cType,
>;
type TranslationBusType = Scs0009<BufferedUartTx<'static>, BufferedUartRx<'static>>;

// Static mutexes for modules
static MODULE0: static_cell::StaticCell<Mutex<CriticalSectionRawMutex, ModuleType>> =
    static_cell::StaticCell::new();
static MODULE1: static_cell::StaticCell<Mutex<CriticalSectionRawMutex, ModuleType>> =
    static_cell::StaticCell::new();
static MODULE2: static_cell::StaticCell<Mutex<CriticalSectionRawMutex, ModuleType>> =
    static_cell::StaticCell::new();
static TRANSLATION_BUS: static_cell::StaticCell<Mutex<CriticalSectionRawMutex, TranslationBusType>> =
    static_cell::StaticCell::new();

/// LED blink task (5Hz) + periodic status report + ground sensor monitoring
#[embassy_executor::task]
async fn led_status_task(
    mut led: Output<'static>,
    module0: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
    module1: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
    module2: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
) {
    let status_tx = status_sender();
    let mut led_state = false;
    let mut cycle_count: u8 = 0;

    loop {
        led_state = !led_state;
        led.toggle();

        // Update ground sensors every cycle
        {
            let mut m0 = module0.lock().await;
            m0.update_ground_sensor().await.ok();
        }
        {
            //let mut m1 = module1.lock().await;
            //m1.update_ground_sensor().await.ok();
        }
        {
            //let mut m2 = module2.lock().await;
            //m2.update_ground_sensor().await.ok();
        }

        // Send ground status every 500ms (5 cycles)
        if cycle_count % 5 == 0 {
            let detection_mask = {
                let m0 = module0.lock().await;
                let m1 = module1.lock().await;
                let m2 = module2.lock().await;
                let mut mask = 0u8;
                if m0.ground_detected() {
                    mask |= 0x01;
                }
                if m1.ground_detected() {
                    mask |= 0x02;
                }
                if m2.ground_detected() {
                    mask |= 0x04;
                }
                mask
            };
            status_tx
                .try_send(CanMessage::GroundStatus {
                    sensor: 0,
                    detection_mask,
                })
                .ok();
        }

        // Every LED-on cycle, send periodic arm status for all modules
        if led_state {
            let target = ArmTarget::BROADCAST_ALL;

            {
                let mut m0 = module0.lock().await;
                m0.update_all_states().await.ok();
                for status in m0.get_status_messages(target) {
                    status_tx.try_send(status).ok();
                }
            }
            //{
            //    let mut m1 = module1.lock().await;
            //    m1.update_all_states().await.ok();
            //    for status in m1.get_status_messages(target) {
            //        status_tx.try_send(status).ok();
            //    }
            //}
            //{
            //    let mut m2 = module2.lock().await;
            //    m2.update_all_states().await.ok();
            //    for status in m2.get_status_messages(target) {
            //        status_tx.try_send(status).ok();
            //    }
            //}
        }

        cycle_count = cycle_count.wrapping_add(1);
        Timer::after_millis(100).await;
    }
}

/// Helper to set servo ID via broadcast on any servo bus
async fn set_servo_id_on_bus<TX: embedded_io_async::Write, RX: embedded_io_async::Read>(
    servo: &mut Scs0009<TX, RX>,
    new_id: u8,
) -> bool {
    if servo.unlock_eeprom(0xFE).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    if servo.set_id(new_id).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    if servo.lock_eeprom(new_id).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    match servo.ping(new_id).await {
        Ok(true) => {
            info!("Servo ID set to {} successfully", new_id);
            true
        }
        _ => {
            warn!("Failed to verify servo ID {}", new_id);
            false
        }
    }
}

/// CAN command receiver task
#[embassy_executor::task]
async fn cmd_task(
    module0: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
    module1: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
    module2: &'static Mutex<CriticalSectionRawMutex, ModuleType>,
    translation_bus: &'static Mutex<CriticalSectionRawMutex, TranslationBusType>,
) {
    let cmd_rx = cmd_receiver();
    let status_tx = status_sender();
    let mut ota_handler = ota_handler::OtaHandler::new();

    loop {
        let msg = cmd_rx.receive().await;

        // Handle OTA messages first
        if msg.domain() == Domain::Ota {
            if let Some(response) = ota_handler.handle_message(&msg) {
                status_tx.try_send(response).ok();
            }
            if matches!(msg, CanMessage::OtaReboot) {
                info!("OTA reboot requested");
                cortex_m::peripheral::SCB::sys_reset();
            }
            continue;
        }

        // Handle SetServoId
        if let CanMessage::SetServoId { bus, new_id } = &msg {
            let success = match bus {
                ServoBus::Module0 => {
                    let mut m0 = module0.lock().await;
                    set_servo_id_on_bus(m0.servo_bus_mut(), *new_id).await
                }
                ServoBus::Module1 => {
                    let mut m1 = module1.lock().await;
                    set_servo_id_on_bus(m1.servo_bus_mut(), *new_id).await
                }
                ServoBus::Module2 => {
                    let mut m2 = module2.lock().await;
                    set_servo_id_on_bus(m2.servo_bus_mut(), *new_id).await
                }
                ServoBus::Translation => {
                    let mut bus = translation_bus.lock().await;
                    set_servo_id_on_bus(&mut *bus, *new_id).await
                }
            };
            status_tx
                .try_send(CanMessage::SetServoIdResult {
                    bus: *bus,
                    new_id: *new_id,
                    success,
                })
                .ok();
            continue;
        }

        // Handle translation commands
        if let CanMessage::SetTranslation {
            module,
            position,
            time_ms,
        } = &msg
        {
            if (*module as usize) < 3 {
                let servo_id = TRANSLATION_SERVO_IDS[*module as usize];
                let mut bus = translation_bus.lock().await;
                if let Err(e) = bus.set_position(servo_id, *position, *time_ms, 0).await {
                    warn!("Translation servo {} error: {:?}", module, e);
                }
            }
            continue;
        }

        // Handle ground sensor commands
        match &msg {
            CanMessage::SetGroundThreshold { sensor, threshold } => {
                match *sensor {
                    0 => module0.lock().await.set_ground_threshold(*threshold),
                    1 => module1.lock().await.set_ground_threshold(*threshold),
                    2 => module2.lock().await.set_ground_threshold(*threshold),
                    _ => {}
                }
                continue;
            }
            CanMessage::RequestGroundValue { sensor } => {
                let response = match *sensor {
                    0 => {
                        let m = module0.lock().await;
                        Some(CanMessage::GroundValue {
                            sensor: 0,
                            value: m.ground_value(),
                            threshold: m.ground_threshold(),
                        })
                    }
                    1 => {
                        let m = module1.lock().await;
                        Some(CanMessage::GroundValue {
                            sensor: 1,
                            value: m.ground_value(),
                            threshold: m.ground_threshold(),
                        })
                    }
                    2 => {
                        let m = module2.lock().await;
                        Some(CanMessage::GroundValue {
                            sensor: 2,
                            value: m.ground_value(),
                            threshold: m.ground_threshold(),
                        })
                    }
                    _ => None,
                };
                if let Some(resp) = response {
                    status_tx.try_send(resp).ok();
                }
                continue;
            }
            _ => {}
        }

        // Handle color sensor raw request (needs response via status channel)
        if let CanMessage::RequestColorSensorRaw { target } = &msg {
            if target.match_module(0) || target.is_module_broadcast() {
                let mut m0 = module0.lock().await;
                if let Some(resp) = m0.read_color_sensor_raw(target.arm).await {
                    status_tx.try_send(resp).ok();
                }
            }
            if target.match_module(1) || target.is_module_broadcast() {
                let mut m1 = module1.lock().await;
                if let Some(resp) = m1.read_color_sensor_raw(target.arm).await {
                    status_tx.try_send(resp).ok();
                }
            }
            if target.match_module(2) || target.is_module_broadcast() {
                let mut m2 = module2.lock().await;
                if let Some(resp) = m2.read_color_sensor_raw(target.arm).await {
                    status_tx.try_send(resp).ok();
                }
            }
            continue;
        }

        // Handle arm-targeted messages
        if let Some(target) = msg.arm_target() {
            if target.match_module(0) || target.is_module_broadcast() {
                let mut m0 = module0.lock().await;
                if let Some(Err(e)) = m0.handle_message(&msg).await {
                    warn!("Module 0 error: {:?}", e);
                    rprintln!("Module 0 error: {:?}", e);
                }
            }
            if target.match_module(1) || target.is_module_broadcast() {
                let mut m1 = module1.lock().await;
                if let Some(Err(e)) = m1.handle_message(&msg).await {
                    warn!("Module 1 error: {:?}", e);
                }
            }
            if target.match_module(2) || target.is_module_broadcast() {
                let mut m2 = module2.lock().await;
                if let Some(Err(e)) = m2.handle_message(&msg).await {
                    warn!("Module 2 error: {:?}", e);
                }
            }

            // Send status after SetArm or RequestArmStatus
            if matches!(msg, CanMessage::SetArm { .. } | CanMessage::RequestArmStatus { .. }) {
                for module in [module0, module1, module2] {
                    let m = module.lock().await;
                    for status in m.get_status_messages(target) {
                        status_tx.try_send(status).ok();
                    }
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(16_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.mux.fdcan12sel = mux::Fdcansel::HSE;
    }
    let p = embassy_stm32::init(config);

    // =========================================================================
    // CAN Bus (FDCAN1: PA11 RX / PA12 TX @ 500kbps)
    // =========================================================================
    let mut can = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can.set_bitrate(500_000);
    let can = can.start(OperatingMode::NormalOperationMode);
    let (can_tx, can_rx, properties) = can.split();
    spawner.spawn(can_handler::can_rx_task(can_rx)).unwrap();
    spawner.spawn(can_handler::can_tx_task(can_tx)).unwrap();
    spawner.spawn(can_handler::can_monitor_task(properties)).unwrap();


    // Initialize CAN logger
    unsafe { can_logger::init(log_sender()) };
    rprintln!("CAN logger initialized");

    let mut led = Output::new(p.PD11, Level::Low, Speed::Low);

   
    // =========================================================================
    // Module 0: USART11 (PA6) + I2C1 (PB6/PB7)
    // =========================================================================
    let mut uart0_config = UartConfig::default();
    uart0_config.baudrate = 1_000_000;
    static UART0_TX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static UART0_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let uart0 = BufferedUart::new_half_duplex(
        p.USART11,
        p.PA6,
        Irqs,
        UART0_TX_BUF.init([0u8; 64]),
        UART0_RX_BUF.init([0u8; 64]),
        uart0_config,
        HalfDuplexReadback::Readback,
    )
    .unwrap();
    let (tx0, rx0) = uart0.split();

    let i2c1 = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        I2cConfig::default(),
    );

    let mut module0 = Module::new(0, Scs0009::new(tx0, rx0), I2cDevices::new(i2c1), MODULE0_SERVO_IDS);
    info!("Module 0 init: {:?}", module0.init().await);
    let module0 = MODULE0.init(Mutex::new(module0));

    // =========================================================================
    // Module 1: UART5 (PB13) + I2C2 (PB10/PB11)
    // =========================================================================
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = 1_000_000;
    static UART1_TX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static UART1_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let uart1 = BufferedUart::new_half_duplex(
        p.UART5,
        p.PB13,
        Irqs,
        UART1_TX_BUF.init([0u8; 64]),
        UART1_RX_BUF.init([0u8; 64]),
        uart1_config,
        HalfDuplexReadback::Readback,
    )
    .unwrap();
    let (tx1, rx1) = uart1.split();

    let i2c2 = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB12,
        Irqs,
        p.GPDMA1_CH2,
        p.GPDMA1_CH3,
        I2cConfig::default(),
    );

    let mut module1 = Module::new(1, Scs0009::new(tx1, rx1), I2cDevices::new(i2c2), MODULE1_SERVO_IDS);
    module1.init().await.ok();
    let module1 = MODULE1.init(Mutex::new(module1));

    // =========================================================================
    // Module 2: USART6 (PC6) + I2C3 (PA8/PC9)
    // =========================================================================
    let mut uart2_config = UartConfig::default();
    uart2_config.baudrate = 1_000_000;
    static UART2_TX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static UART2_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let uart2 = BufferedUart::new_half_duplex(
        p.USART6,
        p.PC6,
        Irqs,
        UART2_TX_BUF.init([0u8; 64]),
        UART2_RX_BUF.init([0u8; 64]),
        uart2_config,
        HalfDuplexReadback::Readback,
    )
    .unwrap();
    let (tx2, rx2) = uart2.split();

    let i2c3 = I2c::new(
        p.I2C3,
        p.PA8,
        p.PC9,
        Irqs,
        p.GPDMA1_CH4,
        p.GPDMA1_CH5,
        I2cConfig::default(),
    );

    let mut module2 = Module::new(2, Scs0009::new(tx2, rx2), I2cDevices::new(i2c3), MODULE2_SERVO_IDS);
    module2.init().await.ok();
    let module2 = MODULE2.init(Mutex::new(module2));

    // =========================================================================
    // Translation servo bus: USART1 (PB14) - shared for 3 translation servos
    // =========================================================================
    let mut uart_trans_config = UartConfig::default();
    uart_trans_config.baudrate = 1_000_000;
    static UART_TRANS_TX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    static UART_TRANS_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let uart_trans = BufferedUart::new_half_duplex(
        p.USART1,
        p.PB14,
        Irqs,
        UART_TRANS_TX_BUF.init([0u8; 64]),
        UART_TRANS_RX_BUF.init([0u8; 64]),
        uart_trans_config,
        HalfDuplexReadback::Readback,
    )
    .unwrap();
    let (tx_trans, rx_trans) = uart_trans.split();
    let translation_bus = TRANSLATION_BUS.init(Mutex::new(Scs0009::new(tx_trans, rx_trans)));

    // =========================================================================
    // Lidars M703A (6x, 19200 baud full-duplex, nCTRL=PA4, PWR_EN=PB1)
    // =========================================================================
    let lidar_nctrl = Output::new(p.PA4, Level::High, Speed::Low);
    let lidar_pwr_en = Output::new(p.PB1, Level::High, Speed::Low);

    let mut lidar_uart_config = UartConfig::default();
    lidar_uart_config.baudrate = 19200;

    // Lidar 0 (module 0.0): USART3 TX=PD8 RX=PD9
    static LIDAR0_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR0_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar0_uart = BufferedUart::new(
        p.USART3, p.PD9, p.PD8,
        LIDAR0_TX_BUF.init([0u8; 32]),
        LIDAR0_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar0_tx, lidar0_rx) = lidar0_uart.split();

    // Lidar 1 (module 0.1): USART2 TX=PA2 RX=PA3
    static LIDAR1_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR1_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar1_uart = BufferedUart::new(
        p.USART2, p.PA3, p.PA2,
        LIDAR1_TX_BUF.init([0u8; 32]),
        LIDAR1_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar1_tx, lidar1_rx) = lidar1_uart.split();

    // Lidar 2 (module 1.0): UART7 TX=PE8 RX=PE7
    static LIDAR2_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR2_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar2_uart = BufferedUart::new(
        p.UART7, p.PE7, p.PE8,
        LIDAR2_TX_BUF.init([0u8; 32]),
        LIDAR2_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar2_tx, lidar2_rx) = lidar2_uart.split();

    // Lidar 3 (module 1.1): UART4 TX=PA0 RX=PA1
    static LIDAR3_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR3_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar3_uart = BufferedUart::new(
        p.UART4, p.PA1, p.PA0,
        LIDAR3_TX_BUF.init([0u8; 32]),
        LIDAR3_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar3_tx, lidar3_rx) = lidar3_uart.split();

    // Lidar 4 (module 2.0): USART10 TX=PE3 RX=PE2
    static LIDAR4_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR4_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar4_uart = BufferedUart::new(
        p.USART10, p.PE2, p.PE3,
        LIDAR4_TX_BUF.init([0u8; 32]),
        LIDAR4_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar4_tx, lidar4_rx) = lidar4_uart.split();

    // Lidar 5 (module 2.1): UART9 TX=PD15 RX=PD14
    static LIDAR5_TX_BUF: static_cell::StaticCell<[u8; 32]> = static_cell::StaticCell::new();
    static LIDAR5_RX_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();
    let lidar5_uart = BufferedUart::new(
        p.UART9, p.PD14, p.PD15,
        LIDAR5_TX_BUF.init([0u8; 32]),
        LIDAR5_RX_BUF.init([0u8; 64]),
        Irqs, lidar_uart_config,
    ).unwrap();
    let (lidar5_tx, lidar5_rx) = lidar5_uart.split();

    lidar::init_and_spawn(
        &spawner,
        lidar_nctrl,
        lidar_pwr_en,
        [
            lidar::m703a::M703a::new(0, lidar0_tx, lidar0_rx),
            lidar::m703a::M703a::new(1, lidar1_tx, lidar1_rx),
            lidar::m703a::M703a::new(2, lidar2_tx, lidar2_rx),
            lidar::m703a::M703a::new(3, lidar3_tx, lidar3_rx),
            lidar::m703a::M703a::new(4, lidar4_tx, lidar4_rx),
            lidar::m703a::M703a::new(5, lidar5_tx, lidar5_rx),
        ],
    );

    // =========================================================================
    // Startup
    // =========================================================================
    for _ in 0..5 {
        led.toggle();
        Timer::after_millis(100).await;
    }

    rprintln!("Picotter ready");

    spawner
        .spawn(led_status_task(led, module0, module1, module2))
        .unwrap();
    spawner
        .spawn(cmd_task(module0, module1, module2, translation_bus))
        .unwrap();

    // Main task idles
    loop {
        Timer::after_millis(1000).await;
        info!("Picotter alive !");
    }
}
