//! Lidar subsystem: 6x M703A laser distance modules
//!
//! Hardware: 6 lidars (2 per arm module), shared nCTRL pin (PA4) and power enable (PB1).
//! Each lidar has its own UART at 19200 baud full-duplex.
//!
//! Power lifecycle:
//! - Starts powered OFF after `init_and_spawn`
//! - `power_on()` enables power, tasks poll and start init sequence
//! - `power_off()` cuts power, tasks poll and go back to waiting

pub mod m703a;

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::pac;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Timer};
use rtt_target::rprintln;
use log::info;

use m703a::{LidarMeasurement, M703a, M703aError};

pub const NUM_LIDARS: usize = 6;
pub const LIDARS_PER_MODULE: usize = 2;

const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(3);
const POLL_INTERVAL: Duration = Duration::from_millis(25);

// Shared measurement storage
static LIDAR_DATA: [BlockingMutex<CriticalSectionRawMutex, Cell<LidarMeasurement>>; NUM_LIDARS] = [
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
];

// Power state: true = powered on, false = powered off
static POWER_STATE: AtomicBool = AtomicBool::new(false);

// Barrier 1: tasks that have disabled TX and are ready for power-on
static POWER_READY: AtomicU8 = AtomicU8::new(0);
// Barrier 2: tasks that have completed init (success or failure)
static INIT_REPORTED: AtomicU8 = AtomicU8::new(0);

pub struct LidarManager {
    nctrl: Output<'static>,
    power_en: Output<'static>,
}

// Manager stored in a blocking mutex so tasks can access it (enable/disable nCTRL)
static MANAGER: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<LidarManager>>> =
    BlockingMutex::new(RefCell::new(None));

impl LidarManager {
    pub fn new(nctrl: Output<'static>, power_en: Output<'static>) -> Self {
        Self { nctrl, power_en }
    }

    /// Pull nCTRL LOW to enable continuous measurement mode
    fn enable_nctrl(&mut self) {
        self.nctrl.set_low();
    }

    /// Pull nCTRL HIGH to stop continuous measurements
    fn disable_nctrl(&mut self) {
        self.nctrl.set_high();
    }
}

/// Cut power to all lidars
pub fn power_off() {
    if !POWER_STATE.swap(false, Ordering::Relaxed) {
        return; // already off
    }
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.disable_nctrl();
            mgr.power_en.set_high();
        }
    });
    rprintln!("Lidar: power OFF");
}

/// Signal tasks to start power-on sequence (does NOT touch hardware)
pub fn power_on() {
    if POWER_STATE.swap(true, Ordering::Relaxed) {
        return; // already on
    }
    POWER_READY.store(0, Ordering::SeqCst);
    INIT_REPORTED.store(0, Ordering::SeqCst);
    rprintln!("Lidar: power ON requested");
}

/// Actually enable power hardware — called from tasks after TX/nCTRL are safe
fn hw_power_enable() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.power_en.set_low();
        }
    });
}

/// Read measurement for a specific lidar
pub fn get_measurement(id: usize) -> LidarMeasurement {
    if id < NUM_LIDARS {
        LIDAR_DATA[id].lock(|cell| cell.get())
    } else {
        LidarMeasurement::new()
    }
}

/// Read all 6 measurements
pub fn get_all_measurements() -> [LidarMeasurement; NUM_LIDARS] {
    let mut out = [LidarMeasurement::new(); NUM_LIDARS];
    for (i, slot) in LIDAR_DATA.iter().enumerate() {
        out[i] = slot.lock(|cell| cell.get());
    }
    out
}

/// Clear stored measurement for a lidar (called on power off)
fn clear_measurement(id: usize) {
    if id < NUM_LIDARS {
        LIDAR_DATA[id].lock(|cell| cell.set(LidarMeasurement::new()));
    }
}

type LidarConcrete = M703a<BufferedUartTx<'static>, BufferedUartRx<'static>>;

/// Get USART register block for a lidar by ID
fn usart_regs(id: u8) -> pac::usart::Usart {
    match id {
        0 => pac::USART3,
        1 => pac::USART2,
        2 => pac::UART7,
        3 => pac::UART4,
        4 => pac::USART10,
        5 => pac::UART9,
        _ => unreachable!(),
    }
}

/// Disable USART transmitter (TE=0) — TX pin stops driving high
fn disable_usart_tx(id: u8) {
    usart_regs(id).cr1().modify(|w| w.set_te(false));
}

/// Re-enable USART transmitter (TE=1)
fn enable_usart_tx(id: u8) {
    usart_regs(id).cr1().modify(|w| w.set_te(true));
}

/// Enable nCTRL LOW (continuous mode) — called from tasks after boot
fn enable_nctrl() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.enable_nctrl();
        }
    });
}

fn disable_nctrl() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.disable_nctrl();
        }
    });
}

/// Check if power is still on
fn is_powered() -> bool {
    POWER_STATE.load(Ordering::Relaxed)
}

/// Wait until power is ON, polling every 25ms
async fn wait_power_on() {
    while !is_powered() {
        Timer::after(POLL_INTERVAL).await;
    }
}

#[embassy_executor::task(pool_size = 6)]
async fn lidar_task(mut lidar: LidarConcrete, id: u8) {
    rprintln!("Lidar {} task started, waiting for power", id);

    loop {
        wait_power_on().await;
        info!("Test on {}", id);

        // --- Phase 1: Cut parasitic power, then apply real power ---
        // Disable TX (stops driving high → no parasitic power via UART)
        disable_usart_tx(id);
        enable_nctrl();
        clear_measurement(id as usize);

        // Barrier 1: wait for all tasks to have disabled TX
        POWER_READY.fetch_add(1, Ordering::SeqCst);
        while POWER_READY.load(Ordering::SeqCst) < NUM_LIDARS as u8 {
            if !is_powered() {
                break;
            }
            Timer::after(POLL_INTERVAL).await;
        }
        if !is_powered() {
            enable_usart_tx(id);
            continue;
        }

        // All TX disabled — wait for parasitic power to drain, then apply real power
        Timer::after_millis(250).await;
        hw_power_enable(); // idempotent, all tasks call it
        enable_usart_tx(id);
        disable_nctrl();

        // Wait for module to boot
        Timer::after_millis(750).await;

        if !is_powered() {
            continue;
        }

        // --- Phase 2: Init sequence ---
        // Read version (informational, not fatal)
        match lidar.read_version().await {
            Ok(ver) => {
                let len = ver.iter().position(|&b| b == 0).unwrap_or(ver.len());
                info!(
                    "Lidar {}: version: {}",
                    id,
                    core::str::from_utf8(&ver[..len]).unwrap_or("?")
                );
            }
            Err(e) => info!("Lidar {}: version read failed: {:?}", id, e),
        }

        if !is_powered() {
            continue;
        }

        // Laser ON must be called before nCTRL goes low
        let init_ok = match lidar.laser_on().await {
            Ok(()) => true,
            Err(e) => {
                rprintln!("Lidar {}: laser_on failed: {:?}, sleeping until next power cycle", id, e);
                false
            }
        };

        // Barrier 2: wait for all tasks to finish init
        let count = INIT_REPORTED.fetch_add(1, Ordering::SeqCst) + 1;
        if count == NUM_LIDARS as u8 {
            rprintln!("Lidar {}: all {} reported, enabling nCTRL", id, NUM_LIDARS);
            enable_nctrl();
        }
        while INIT_REPORTED.load(Ordering::SeqCst) < NUM_LIDARS as u8 {
            if !is_powered() {
                break;
            }
            Timer::after(POLL_INTERVAL).await;
        }

        if !is_powered() {
            continue;
        }

        // Failed lidars sleep until next power cycle
        if !init_ok {
            while is_powered() {
                Timer::after(POLL_INTERVAL).await;
            }
            continue;
        }

        Timer::after_millis(50).await;

        // Start fast continuous measurement
        if let Err(e) = lidar.start_continuous().await {
            rprintln!("Lidar {}: start_continuous failed: {:?}", id, e);
        }

        // Continuous read loop — exits when power is cut
        while is_powered() {
            match lidar.read_measurement().await {
                Ok(m) => {
                    LIDAR_DATA[id as usize].lock(|cell| cell.set(m));
                    //info!("L{}: {}mm q={}", id, m.distance_mm, m.signal_quality);
                }
                Err(M703aError::Timeout) => {
                    if lidar.is_stale(WATCHDOG_TIMEOUT) {
                        rprintln!("Lidar {}: watchdog timeout, resetting", id);
                        lidar.reset().await;
                    }
                }
                Err(M703aError::ModuleError(code)) => {
                    rprintln!("Lidar {}: module error Er.{:02}", id, code);
                    Timer::after_millis(100).await;
                }
                Err(e) => {
                    rprintln!("Lidar {}: error {:?}", id, e);
                    Timer::after_millis(100).await;
                }
            }
        }

        rprintln!("Lidar {}: power cut, stopping", id);
        clear_measurement(id as usize);
    }
}

/// Initialize lidar subsystem and spawn all 6 tasks.
/// Call this after UART setup. nCTRL and power_en GPIOs are consumed.
/// Lidars start powered OFF — call `power_on()` to start measurements.
pub fn init_and_spawn(
    spawner: &Spawner,
    nctrl: Output<'static>,
    power_en: Output<'static>,
    lidars: [LidarConcrete; NUM_LIDARS],
) {
    let mut mgr = LidarManager::new(nctrl, power_en);
    mgr.disable_nctrl();
    mgr.power_en.set_high(); // Start powered off

    MANAGER.lock(|cell| {
        cell.replace(Some(mgr));
    });

    rprintln!("Lidar manager initialized, power OFF");

    for (id, lidar) in lidars.into_iter().enumerate() {
        spawner.spawn(lidar_task(lidar, id as u8).unwrap());
    }
}
