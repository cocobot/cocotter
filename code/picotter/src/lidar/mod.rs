//! Lidar subsystem: 6x M703A laser distance modules
//!
//! Hardware: 6 lidars (2 per arm module), shared nCTRL pin (PA4) and power enable (PB1).
//! Each lidar has its own UART at 19200 baud full-duplex.

pub mod m703a;

use core::cell::{Cell, RefCell};
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Timer};
use rtt_target::rprintln;

use m703a::{LidarMeasurement, M703a, M703aError};

pub const NUM_LIDARS: usize = 6;
pub const LIDARS_PER_MODULE: usize = 2;

const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(3);
const STARTUP_RETRIES: usize = 3;

// Shared measurement storage
static LIDAR_DATA: [BlockingMutex<CriticalSectionRawMutex, Cell<LidarMeasurement>>; NUM_LIDARS] = [
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
    BlockingMutex::new(Cell::new(LidarMeasurement::new())),
];

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
    pub fn enable(&mut self) {
        self.nctrl.set_low();
    }

    /// Pull nCTRL HIGH to stop continuous measurements
    pub fn disable(&mut self) {
        self.nctrl.set_high();
    }

    /// Cut power to all lidars
    pub fn power_off(&mut self) {
        self.nctrl.set_high();
        self.power_en.set_high();
    }

    /// Enable power to all lidars
    pub fn power_on(&mut self) {
        self.power_en.set_low();
    }
}

/// Pull nCTRL LOW (idempotent, safe to call from multiple tasks)
pub fn enable_continuous() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.enable();
        }
    });
}

/// Pull nCTRL HIGH to stop all continuous measurements
pub fn disable_continuous() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.disable();
        }
    });
}

/// Cut power to all lidars
pub fn power_off_all() {
    MANAGER.lock(|cell| {
        if let Some(mgr) = cell.borrow_mut().as_mut() {
            mgr.power_off();
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

type LidarConcrete = M703a<BufferedUartTx<'static>, BufferedUartRx<'static>>;

#[embassy_executor::task(pool_size = 6)]
async fn lidar_task(mut lidar: LidarConcrete, id: u8) {
    rprintln!("Lidar {} task started", id);

    // Wait for module to boot after power on
    Timer::after_millis(500).await;

    // Laser ON with retries
    //let mut started: bool = false;
    //for attempt in 0..STARTUP_RETRIES {
    //    match lidar.laser_on().await {
    //        Ok(()) => {
    //            rprintln!("Lidar {}: laser ON", id);
    //            started = true;
    //            break;
    //        }
    //        Err(e) => {
    //            rprintln!(
    //                "Lidar {}: laser_on attempt {}/{} failed: {:?}",
    //                id,
    //                attempt + 1,
    //                STARTUP_RETRIES,
    //                e
    //            );
    //            Timer::after_secs(1).await;
    //        }
    //    }
    //}

    // Read and print version
     match lidar.read_version().await {
            Ok(ver) => {
                let len = ver.iter().position(|&b| b == 0).unwrap_or(ver.len());
                rprintln!("Lidar {}: version: {}", id, core::str::from_utf8(&ver[..len]).unwrap_or("?"));
            }
            Err(e) => rprintln!("Lidar {}: version read failed: {:?}", id, e),
        }

    // Enable continuous mode (nCTRL LOW) — idempotent, all tasks call it
    enable_continuous();
    Timer::after_millis(50).await;

    // Start fast continuous measurement
    if let Err(e) = lidar.start_continuous().await {
        rprintln!("Lidar {}: start_continuous failed: {:?}", id, e);
    }

    // Continuous read loop
    loop {
        match lidar.read_measurement().await {
            Ok(m) => {
                LIDAR_DATA[id as usize].lock(|cell| cell.set(m));
                rprintln!("L{}: {}mm q={}", id, m.distance_mm, m.signal_quality);
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
}

/// Initialize lidar subsystem and spawn all 6 tasks.
/// Call this after UART setup. nCTRL and power_en GPIOs are consumed.
pub fn init_and_spawn(
    spawner: &Spawner,
    nctrl: Output<'static>,
    power_en: Output<'static>,
    lidars: [LidarConcrete; NUM_LIDARS],
) {
    let mut mgr = LidarManager::new(nctrl, power_en);
    mgr.power_on();
    // nCTRL stays HIGH — tasks will pull it low after laser_on + version read

    MANAGER.lock(|cell| {
        cell.replace(Some(mgr));
    });
    rprintln!("Lidar manager initialized, power ON, nCTRL HIGH");

    for (id, lidar) in lidars.into_iter().enumerate() {
        spawner.spawn(lidar_task(lidar, id as u8)).unwrap();
    }
}
