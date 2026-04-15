#![no_std]
#![no_main]
#![allow(dead_code)]

mod arm;
mod can_handler;
mod can_logger;
mod can_protocol;
mod ground_sensors;
mod i2c_devices;
mod lidar;
mod module;
mod scs0009;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::join::{join, join3, join_array};
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::can::OperatingMode;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::timer::low_level::{CountingMode, OutputPolarity};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{
    BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig, HalfDuplexReadback,
};
use embassy_stm32::interrupt;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use log::{info, warn};
use panic_rtt_target as _;
use rtt_target::rprintln;

static COLOR_LED_DUTY: AtomicU8 = AtomicU8::new(0);
static COLOR_THRESHOLD: AtomicU16 = AtomicU16::new(2000);
use rtt_target::rtt_init_print;

// Shared no-init region at end of RAM (256 bytes, matches bootloader)
// Layout: [panic_magic:4][panic_len:4][panic_msg:240][bootloader_magic:4][pad:4]
const SHARED_BASE: u32 = 0x2009_FF00;
const PANIC_MAGIC_ADDR: *mut u32 = SHARED_BASE as *mut u32;
const PANIC_LEN_ADDR: *mut u32 = (SHARED_BASE + 4) as *mut u32;
const PANIC_MSG_ADDR: *mut u8 = (SHARED_BASE + 8) as *mut u8;
const PANIC_MSG_MAX: usize = 240;
const BOOTLOADER_MAGIC_ADDR: *mut u32 = (SHARED_BASE + 248) as *mut u32;
const PANIC_MAGIC: u32 = 0xDEAD_BEEF;
const BOOTLOADER_MAGIC: u32 = 0xB007_10AD;

/// Fixed-size buffer writer for formatting panic messages without alloc
struct PanicBuf {
    pos: usize,
}

impl PanicBuf {
    const fn new() -> Self {
        Self { pos: 0 }
    }
}

impl core::fmt::Write for PanicBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = PANIC_MSG_MAX - self.pos;
        let to_write = bytes.len().min(remaining);
        if to_write > 0 {
            unsafe {
                core::ptr::copy_nonoverlapping(
                    bytes.as_ptr(),
                    PANIC_MSG_ADDR.add(self.pos),
                    to_write,
                );
            }
            self.pos += to_write;
        }
        Ok(())
    }
}

////// Panic handler: log to RTT, store message in shared RAM for bootloader, and reset
///#[panic_handler]
///fn panic(info: &core::panic::PanicInfo) -> ! {
///    // Print to RTT (best effort)
///    rprintln!("PANIC: {}", info);
///
///    // Write panic message to shared no-init RAM
///    let mut buf = PanicBuf::new();
///    let _ = core::fmt::write(&mut buf, format_args!("{}", info));
///
///    unsafe {
///        core::ptr::write_volatile(PANIC_LEN_ADDR, buf.pos as u32);
///        core::ptr::write_volatile(PANIC_MAGIC_ADDR, PANIC_MAGIC);
///        core::ptr::write_volatile(BOOTLOADER_MAGIC_ADDR, BOOTLOADER_MAGIC);
///    }
///
///    cortex_m::peripheral::SCB::sys_reset();
///}



use can_handler::{cmd_receiver, log_sender, status_sender};
use can_protocol::{ArmFlags, ArmTarget, CanMessage, Domain, ServoBus, Stage2Target};
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
    // DMA for I2C
    GPDMA1_CHANNEL0 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH1>;
    GPDMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH2>;
    GPDMA1_CHANNEL3 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH3>;
    GPDMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH4>;
    GPDMA1_CHANNEL5 => embassy_stm32::dma::InterruptHandler<peripherals::GPDMA1_CH5>;
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
const MODULE0_SERVO_IDS: [u8; 4] = [10, 11, 12, 13];
const MODULE1_SERVO_IDS: [u8; 4] = [10, 11, 12, 13];
const MODULE2_SERVO_IDS: [u8; 4] = [10, 11, 12, 13];

// Stage2 servo IDs (2 per module, on same bus as arm servos)
const MODULE0_STAGE2_IDS: [u8; 2] = [20, 21];
const MODULE1_STAGE2_IDS: [u8; 2] = [20, 21];
const MODULE2_STAGE2_IDS: [u8; 2] = [20, 21];

// Translation servo IDs (one per module, on shared bus)
const TRANSLATION_SERVO_IDS: [u8; 3] = [30, 31, 32];

// Translation target positions (set by SetTranslation, read by periodic update)
static TRANSLATION_TARGETS: [AtomicU16; 3] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
static TRANSLATION_MOVING: [AtomicU8; 3] = [AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0)];

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
    translation_bus: &'static Mutex<CriticalSectionRawMutex, TranslationBusType>,
    mut color_led_pwm: SimplePwmChannel<'static, peripherals::TIM8>,
) {
    let status_tx = status_sender();
    let mut led_state = false;
    let mut cycle_count: u8 = 0;
    let mut prev_transl_moving = [false; 3];
    let mut prev_ground_mask: u8 = 0;
    let mut last_ground_send = Instant::now();
    let mut aru = true;

    loop {
        let t0 = Instant::now();
        led_state = !led_state;

        // Update ground sensors every cycle
        let ground = {
            let mut m0 = module0.lock().await;
            let mut m1 = module1.lock().await;
            let mut m2 = module2.lock().await;
            let (g0, g1, g2) = join3(
                m0.update_ground_sensor(),
                m1.update_ground_sensor(),
                m2.update_ground_sensor(),
            ).await;
            (g0.unwrap_or(false), g1.unwrap_or(false), g2.unwrap_or(false))
        };

        if !ground.0 || !ground.1 || !ground.2 || aru {
            lidar::power_off();
        }
        else {
            lidar::power_on();
        }

        // Build detection mask, send on change or every 2s
        let detection_mask =
            (ground.0 as u8) | ((ground.1 as u8) << 1) | ((ground.2 as u8) << 2);
        if detection_mask != prev_ground_mask || t0.duration_since(last_ground_send) >= Duration::from_secs(2) {
            prev_ground_mask = detection_mask;
            last_ground_send = t0;
            status_tx
                .try_send(CanMessage::GroundStatus { detection_mask })
                .ok();
        }

        let threshold = COLOR_THRESHOLD.load(Ordering::Relaxed);
        let periodic_status = cycle_count % 5 == 0; // 250ms

        // Update and conditionally send status for module 0
        let module_update = |m: &'static Mutex<CriticalSectionRawMutex, ModuleType>| {
            async move {
                let mut m = m.lock().await;
                m.update_all_states().await.ok();
                m.update_all_stage2_states().await.ok();
                m.update_color(led_state, threshold).await;

                let send_status = periodic_status || m.has_moving_changed();
                if send_status {
                    for status in m.get_status_messages(ArmTarget::BROADCAST_ALL) {
                        status_tx.try_send(status).ok();
                    }
                    for status in m.get_stage2_status_messages(Stage2Target::BROADCAST_ALL) {
                        status_tx.try_send(status).ok();
                    }
                }
            }
        };

        let translation_update = async {
            for module in 0..3u8 {
                let servo_id = TRANSLATION_SERVO_IDS[module as usize];
                let (position, error) = match translation_bus.lock().await.read_position(servo_id).await {
                    Ok((pos, err)) => (pos, err),
                    Err(_) => (0, 0xFF),
                };
                let target = TRANSLATION_TARGETS[module as usize].load(Ordering::Relaxed);
                let diff = if position > target { position - target } else { target - position };
                let position_reached = diff <= 10;
                if position_reached {
                    TRANSLATION_MOVING[module as usize].store(0, Ordering::Relaxed);
                }
                let moving = TRANSLATION_MOVING[module as usize].load(Ordering::Relaxed) != 0;
                let moving_changed = moving != prev_transl_moving[module as usize];
                prev_transl_moving[module as usize] = moving;

                if periodic_status || moving_changed {
                    let flags = ArmFlags { torque_enabled: false, moving, position_reached };
                    status_tx
                        .try_send(CanMessage::TranslationStatus {
                            module,
                            position,
                            error,
                            flags,
                        })
                        .ok();
                }

                break; //remove after servo 1 and 2 are cabled
            }
        };

        join(
            join_array([
                module_update(module0),
                //module_update(module1),
                //module_update(module2),
            ]),
            translation_update,
        ).await;

        // Set LED for next cycle (next cycle led_state will be flipped)
        if led_state {
            // Next cycle will be LED off
            color_led_pwm.set_duty_cycle(0);
            led.set_low();
        } else {
            // Next cycle will be LED on
            let duty = COLOR_LED_DUTY.load(Ordering::Relaxed);
            color_led_pwm.set_duty_cycle_fraction(duty as u32, 255);
            led.set_high();
        }

        // Send lidar status every ~200ms (4 cycles @ 50ms)
        // Module 0 → lidar 0,3 / Module 1 → lidar 1,4 / Module 2 → lidar 2,5
        if cycle_count % 4 == 0 {
            const LIDAR_MAP: [[usize; 2]; 3] = [[0, 3], [1, 4], [2, 5]];
            for (module, lidars) in LIDAR_MAP.iter().enumerate() {
                let m0 = lidar::get_measurement(lidars[0]);
                let m1 = lidar::get_measurement(lidars[1]);
                status_tx
                    .try_send(CanMessage::LidarStatus {
                        module: module as u8,
                        distance_0: m0.distance_mm.min(u16::MAX as u32) as u16,
                        sq_0: m0.signal_quality,
                        distance_1: m1.distance_mm.min(u16::MAX as u32) as u16,
                        sq_1: m1.signal_quality,
                    })
                    .ok();
            }
        }

        // Send battery status every ~1s (20 cycles @ 50ms)
        if cycle_count % 20 == 0 {
            let mut sum_mv: u32 = 0;
            let mut count: u32 = 0;
            let mut modules_mask: u8 = 0;

            // Voltage divider: 10k series + 2k2 parallel
            // V_batt = V_adc * (10.0 + 2.2) / 2.2
            // V_adc_mv = raw * 3300 / 4095
            // Combined: V_batt_mv = raw * 3300 * 122 / (4095 * 22)
            const NUM: u32 = 3300 * 122;
            const DEN: u32 = 4095 * 22;

            {
                let m0 = module0.lock().await;
                if let Some(raw) = m0.battery_voltage_raw() {
                    sum_mv += (raw as u32) * NUM / DEN;
                    count += 1;
                    modules_mask |= 0x01;
                }
            }
            {
                let m1 = module1.lock().await;
                if let Some(raw) = m1.battery_voltage_raw() {
                    sum_mv += (raw as u32) * NUM / DEN;
                    count += 1;
                    modules_mask |= 0x02;
                }
            }
            {
                let m2 = module2.lock().await;
                if let Some(raw) = m2.battery_voltage_raw() {
                    sum_mv += (raw as u32) * NUM / DEN;
                    count += 1;
                    modules_mask |= 0x04;
                }
            }

            if count > 0 {
                let voltage_mv = (sum_mv / count) as u16;
                if voltage_mv < 1000 {
                    aru = true;
                }
                else {
                    aru = false;
                }
                status_tx
                    .try_send(CanMessage::BatteryStatus {
                        voltage_mv,
                        modules_mask,
                    })
                    .ok();
            }
        }

        cycle_count = cycle_count.wrapping_add(1);
        let elapsed_ms = (Instant::now() - t0).as_millis();
        let delay = if elapsed_ms < 40 { 50 - elapsed_ms } else { 10 };
        Timer::after_millis(delay).await;
    }
}

/// Helper to set servo ID on any servo bus
async fn set_servo_id_on_bus<TX: embedded_io_async::Write, RX: embedded_io_async::Read>(
    servo: &mut Scs0009<TX, RX>,
    origin_id: u8,
    new_id: u8,
) -> bool {
    if servo.unlock_eeprom(origin_id).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    if servo.change_id(origin_id, new_id).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    if servo.lock_eeprom(new_id).await.is_err() {
        return false;
    }
    Timer::after_millis(50).await;

    match servo.ping(new_id).await {
        Ok(true) => {
            info!("Servo ID changed {} -> {} successfully", origin_id, new_id);
            true
        }
        _ => {
            warn!("Failed to verify servo ID {} -> {}", origin_id, new_id);
            false
        }
    }
}

/// Helper to scan a servo bus
async fn scan_servo_bus<TX: embedded_io_async::Write, RX: embedded_io_async::Read>(
    servo: &mut Scs0009<TX, RX>,
    bus_name: &str,
) {
    info!("Scanning bus {}...", bus_name);
    let (found, count) = servo.scan::<32>(1, 253).await;
    if count == 0 {
        info!("Bus {}: no servos found", bus_name);
    } else {
        for i in 0..count {
            info!("Bus {}: found servo ID {}", bus_name, found[i]);
        }
        info!("Bus {}: {} servo(s) found", bus_name, count);
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

    loop {
        let msg = cmd_rx.receive().await;

        // Track ping reception in cmd_task
        if let CanMessage::Ping { value } = &msg {
            status_tx
                .try_send(CanMessage::Ping { value: *value + 1 })
                .ok();
            continue;
        }

        // Handle Reboot command
        if let CanMessage::Reboot { mode } = &msg {
            match mode {
                cancaner::RebootMode::Bootloader => {
                    info!("Rebooting to bootloader...");
                    unsafe { core::ptr::write_volatile(BOOTLOADER_MAGIC_ADDR, BOOTLOADER_MAGIC) };
                    cortex_m::peripheral::SCB::sys_reset();
                }
                cancaner::RebootMode::Normal => {
                    info!("Rebooting...");
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }
        }

        // OTA messages are handled by the bootloader, ignore here
        if msg.domain() == Domain::Ota {
            continue;
        }

        // Handle SetLidarEnable
        if let CanMessage::SetLidarEnable { enable } = &msg {
            if *enable {
                lidar::power_on();
                info!("Lidars enabled");
            } else {
                lidar::power_off();
                info!("Lidars disabled");
            }
            continue;
        }

        // Handle SetServoId
        if let CanMessage::SetServoId { bus, origin_id, new_id } = &msg {
            let success = match bus {
                ServoBus::Module0 => {
                    let mut m0 = module0.lock().await;
                    set_servo_id_on_bus(m0.servo_bus_mut(), *origin_id, *new_id).await
                }
                ServoBus::Module1 => {
                    let mut m1 = module1.lock().await;
                    set_servo_id_on_bus(m1.servo_bus_mut(), *origin_id, *new_id).await
                }
                ServoBus::Module2 => {
                    let mut m2 = module2.lock().await;
                    set_servo_id_on_bus(m2.servo_bus_mut(), *origin_id, *new_id).await
                }
                ServoBus::Translation => {
                    let mut bus = translation_bus.lock().await;
                    set_servo_id_on_bus(&mut *bus, *origin_id, *new_id).await
                }
            };
            status_tx
                .try_send(CanMessage::SetServoIdResult {
                    bus: *bus,
                    origin_id: *origin_id,
                    new_id: *new_id,
                    success,
                })
                .ok();
            continue;
        }

        // Handle ScanBus
        if let CanMessage::ScanBus { bus } = &msg {
            match bus {
                ServoBus::Module0 => {
                    let mut m0 = module0.lock().await;
                    scan_servo_bus(m0.servo_bus_mut(), "module0").await;
                }
                ServoBus::Module1 => {
                    let mut m1 = module1.lock().await;
                    scan_servo_bus(m1.servo_bus_mut(), "module1").await;
                }
                ServoBus::Module2 => {
                    let mut m2 = module2.lock().await;
                    scan_servo_bus(m2.servo_bus_mut(), "module2").await;
                }
                ServoBus::Translation => {
                    let mut bus = translation_bus.lock().await;
                    scan_servo_bus(&mut *bus, "translation").await;
                }
            }
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
                TRANSLATION_TARGETS[*module as usize].store(*position, Ordering::Relaxed);
                TRANSLATION_MOVING[*module as usize].store(1, Ordering::Relaxed);
                let mut bus = translation_bus.lock().await;
                if let Err(e) = bus.set_position(servo_id, *position, *time_ms, 0).await {
                    warn!("Translation servo {} error: {:?}", module, e);
                }
            }
            continue;
        }

        // Handle RequestTranslationStatus
        if let CanMessage::RequestTranslationStatus { module } = &msg {
            if (*module as usize) < 3 {
                let servo_id = TRANSLATION_SERVO_IDS[*module as usize];
                let mut bus = translation_bus.lock().await;
                let (position, error) = match bus.read_position(servo_id).await {
                    Ok((pos, err)) => (pos, err),
                    Err(e) => {
                        warn!("Translation servo {} read error: {:?}", module, e);
                        (0, 0xFF)
                    }
                };
                let target = TRANSLATION_TARGETS[*module as usize].load(Ordering::Relaxed);
                let diff = if position > target { position - target } else { target - position };
                let position_reached = diff <= 10;
                if position_reached {
                    TRANSLATION_MOVING[*module as usize].store(0, Ordering::Relaxed);
                }
                let moving = TRANSLATION_MOVING[*module as usize].load(Ordering::Relaxed) != 0;
                let flags = ArmFlags { torque_enabled: false, moving, position_reached };
                status_tx
                    .try_send(CanMessage::TranslationStatus {
                        module: *module,
                        position,
                        error,
                        flags,
                    })
                    .ok();
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

        // Handle color LED PWM
        if let CanMessage::SetColorLedPwm { duty } = &msg {
            COLOR_LED_DUTY.store(*duty, Ordering::Relaxed);
            continue;
        }

        // Handle color threshold
        if let CanMessage::SetColorThreshold { threshold } = &msg {
            COLOR_THRESHOLD.store(*threshold, Ordering::Relaxed);
            continue;
        }

        // Handle color sensor raw request — return last stored delta
        if let CanMessage::RequestColorSensorRaw { target } = &msg {
            if target.match_module(0) || target.is_module_broadcast() {
                let m0 = module0.lock().await;
                if let Some(resp) = m0.get_color_delta(target.arm) {
                    status_tx.try_send(resp).ok();
                }
            }
            if target.match_module(1) || target.is_module_broadcast() {
                let m1 = module1.lock().await;
                if let Some(resp) = m1.get_color_delta(target.arm) {
                    status_tx.try_send(resp).ok();
                }
            }
            if target.match_module(2) || target.is_module_broadcast() {
                let m2 = module2.lock().await;
                if let Some(resp) = m2.get_color_delta(target.arm) {
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
            continue;
        }

        // Handle stage2-targeted messages
        if let Some(target) = msg.stage2_target() {
            if target.match_module(0) || target.is_module_broadcast() {
                let mut m0 = module0.lock().await;
                if let Some(Err(e)) = m0.handle_stage2_message(&msg).await {
                    warn!("Module 0 stage2 error: {:?}", e);
                }
            }
            if target.match_module(1) || target.is_module_broadcast() {
                let mut m1 = module1.lock().await;
                if let Some(Err(e)) = m1.handle_stage2_message(&msg).await {
                    warn!("Module 1 stage2 error: {:?}", e);
                }
            }
            if target.match_module(2) || target.is_module_broadcast() {
                let mut m2 = module2.lock().await;
                if let Some(Err(e)) = m2.handle_stage2_message(&msg).await {
                    warn!("Module 2 stage2 error: {:?}", e);
                }
            }

            // Send status after SetStage2 or RequestStage2Status
            if matches!(msg, CanMessage::SetStage2 { .. } | CanMessage::RequestStage2Status { .. }) {
                for module in [module0, module1, module2] {
                    let m = module.lock().await;
                    for status in m.get_stage2_status_messages(target) {
                        status_tx.try_send(status).ok();
                    }
                }
            }
        }
    }
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn TIM7() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();

    // Clear interrupt masks that bootloader may have left set
    unsafe {
        core::arch::asm!("cpsie i");
        core::arch::asm!("cpsie f");
        core::arch::asm!("msr BASEPRI, {}", in(reg) 0u32);
    }

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(16_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV4,     // 16MHz / 4 = 4MHz
            mul: PllMul::MUL125,         // 4MHz × 125 = 500MHz VCO
            divp: Some(PllDiv::DIV2),    // 500MHz / 2 = 250MHz
            divq: Some(PllDiv::DIV10),   // 500MHz / 10 = 50MHz for FDCAN
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.fdcan12sel = mux::Fdcansel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    // =========================================================================
    // CAN Bus (FDCAN1: PA11 RX / PA12 TX @ 500kbps)
    // =========================================================================
    let mut can = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can.set_bitrate(500_000);
    // Use FIFO mode for TX: never replace pending frames, just wait
    let mut can_config = can.config();
    can_config.tx_buffer_mode = embassy_stm32::can::config::TxBufferMode::Fifo;
    can.set_config(can_config);
    let can = can.start(OperatingMode::NormalOperationMode);
    let (can_tx, can_rx, _properties) = can.split();

    // Start high-priority executor on TIM7 for CAN task
    let high_spawner = EXECUTOR_HIGH.start(interrupt::TIM7);
    high_spawner.spawn(can_handler::can_task(can_rx, can_tx).unwrap());


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

    let mut i2c1_config = I2cConfig::default();
    i2c1_config.timeout = embassy_time::Duration::from_millis(100);
    let i2c1 = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        Irqs,
        i2c1_config,
    );

    let mut module0 = Module::new(0, Scs0009::new(tx0, rx0), I2cDevices::new(i2c1), MODULE0_SERVO_IDS, MODULE0_STAGE2_IDS);
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

    let mut i2c2_config = I2cConfig::default();
    i2c2_config.timeout = embassy_time::Duration::from_millis(100);
    let i2c2 = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB12,
        p.GPDMA1_CH2,
        p.GPDMA1_CH3,
        Irqs,
        i2c2_config,
    );

    let mut module1 = Module::new(1, Scs0009::new(tx1, rx1), I2cDevices::new(i2c2), MODULE1_SERVO_IDS, MODULE1_STAGE2_IDS);
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

    let mut i2c3_config = I2cConfig::default();
    i2c3_config.timeout = embassy_time::Duration::from_millis(100);
    let i2c3 = I2c::new(
        p.I2C3,
        p.PA8,
        p.PC9,
        p.GPDMA1_CH4,
        p.GPDMA1_CH5,
        Irqs,
        i2c3_config,
    );

    let mut module2 = Module::new(2, Scs0009::new(tx2, rx2), I2cDevices::new(i2c3), MODULE2_SERVO_IDS, MODULE2_STAGE2_IDS);
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
    // Color sensor LED PWM (TIM8_CH3 on PC8)
    // =========================================================================
    let color_led_pwm = SimplePwm::new(
        p.TIM8,
        None,
        None,
        Some(PwmPin::new(p.PC8, OutputType::PushPull)),
        None,
        Hertz(1_000),
        CountingMode::EdgeAlignedUp,
    );
    let mut color_led_ch3 = color_led_pwm.split().ch3;
    color_led_ch3.set_polarity(OutputPolarity::ActiveLow);
    color_led_ch3.set_duty_cycle(0);
    color_led_ch3.enable();


    // =========================================================================
    // Startup
    // =========================================================================
    for _ in 0..5 {
        led.toggle();
        Timer::after_millis(100).await;
    }

    spawner.spawn(led_status_task(led, module0, module1, module2, translation_bus, color_led_ch3).unwrap());
    spawner.spawn(cmd_task(module0, module1, module2, translation_bus).unwrap());

    // Main task idles
    loop {
        Timer::after_millis(5000).await;
        info!("Picotter is alive !");
    }
}
