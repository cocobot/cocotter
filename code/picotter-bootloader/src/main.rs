#![no_std]
#![no_main]

use cancaner::ota::{OtaReceiver, OtaStorage};
use cancaner::{CanMessage, LogEncoder, LogLevel, OtaReadyStatus, OtaResultStatus};
use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::can::frame::Header;
use embassy_stm32::can::{CanConfigurator, OperatingMode};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, flash, peripherals};
use embassy_time::Timer;
use embedded_can::Id;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN1>;
});

// ============================================================================
// Constants
// ============================================================================

// Shared no-init region at end of RAM (256 bytes, matches picotter app)
// Layout: [panic_magic:4][panic_len:4][panic_msg:240][bootloader_magic:4][pad:4]
const SHARED_BASE: u32 = 0x2009_FF00;
const PANIC_MAGIC_ADDR: *mut u32 = SHARED_BASE as *mut u32;
const PANIC_LEN_ADDR: *mut u32 = (SHARED_BASE + 4) as *mut u32;
const PANIC_MSG_ADDR: *const u8 = (SHARED_BASE + 8) as *const u8;
const PANIC_MSG_MAX: usize = 240;
const BOOTLOADER_MAGIC_ADDR: *mut u32 = (SHARED_BASE + 248) as *mut u32;
const PANIC_MAGIC: u32 = 0xDEAD_BEEF;

/// Magic value written to RAM by the application to request bootloader mode
const BOOTLOADER_MAGIC: u32 = 0xB007_10AD;

/// Base address of the application in flash
const APP_BASE: u32 = 0x0801_0000;

/// Maximum application size (960KB)
const APP_MAX_SIZE: u32 = 960 * 1024;

/// Bootloader flash size (64KB = 8 sectors of 8KB)
const BOOTLOADER_SIZE: u32 = 64 * 1024;

/// Flash sector size on STM32H562
const FLASH_SECTOR_SIZE: u32 = 8 * 1024;

/// Flash write alignment on STM32H5 (128-bit = 16 bytes)
const FLASH_WRITE_ALIGN: usize = 16;

/// Timeout before booting the app if no OTA activity (seconds)
const OTA_TIMEOUT_SECS: u64 = 5;

// ============================================================================
// CRC32 (software, same polynomial as standard CRC-32/ISO-3309)
// ============================================================================

const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0u32;
    while i < 256 {
        let mut crc = i;
        let mut j = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
            j += 1;
        }
        table[i as usize] = crc;
        i += 1;
    }
    table
};

struct Crc32 {
    state: u32,
}

impl Crc32 {
    const fn new() -> Self {
        Self { state: 0xFFFF_FFFF }
    }

    fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let index = ((self.state ^ byte as u32) & 0xFF) as usize;
            self.state = (self.state >> 8) ^ CRC32_TABLE[index];
        }
    }

    fn finalize(&self) -> u32 {
        self.state ^ 0xFFFF_FFFF
    }
}

// ============================================================================
// OTA Flash Storage
// ============================================================================

struct OtaFlashStorage {
    flash: flash::Flash<'static, flash::Blocking>,
    expected_size: u32,
    received_size: u32,
    crc: Crc32,
    /// Buffer for accumulating data until we have FLASH_WRITE_ALIGN bytes
    write_buf: [u8; FLASH_WRITE_ALIGN],
    /// How many bytes are in the write buffer
    buf_len: usize,
    /// Current write offset in flash (aligned)
    flash_offset: u32,
}

impl OtaFlashStorage {
    fn new(flash: flash::Flash<'static, flash::Blocking>) -> Self {
        Self {
            flash,
            expected_size: 0,
            received_size: 0,
            crc: Crc32::new(),
            write_buf: [0xFF; FLASH_WRITE_ALIGN],
            buf_len: 0,
            flash_offset: 0,
        }
    }

    /// Flush the write buffer to flash, padding with 0xFF if needed
    fn flush_buffer(&mut self) -> bool {
        if self.buf_len == 0 {
            return true;
        }
        // Pad remaining bytes with 0xFF
        for i in self.buf_len..FLASH_WRITE_ALIGN {
            self.write_buf[i] = 0xFF;
        }
        let offset = BOOTLOADER_SIZE + self.flash_offset;
        match self.flash.blocking_write(offset, &self.write_buf) {
            Ok(()) => {
                self.flash_offset += FLASH_WRITE_ALIGN as u32;
                self.buf_len = 0;
                self.write_buf = [0xFF; FLASH_WRITE_ALIGN];
                true
            }
            Err(e) => {
                rprintln!("Flash write error at 0x{:08X}: {:?}", offset, e);
                false
            }
        }
    }
}

impl OtaStorage for OtaFlashStorage {
    fn prepare(&mut self, fw_size: u32) -> OtaReadyStatus {
        if fw_size > APP_MAX_SIZE {
            rprintln!("OTA: firmware too large ({} > {})", fw_size, APP_MAX_SIZE);
            return OtaReadyStatus::NoSpace;
        }

        rprintln!("OTA: preparing for {} bytes, erasing flash...", fw_size);

        // Erase application sectors
        let num_sectors = (fw_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
        let first_sector = (BOOTLOADER_SIZE / FLASH_SECTOR_SIZE) as u32;

        for i in 0..num_sectors {
            let sector = first_sector + i;
            let offset = sector * FLASH_SECTOR_SIZE;
            if let Err(e) = self.flash.blocking_erase(offset, offset + FLASH_SECTOR_SIZE) {
                rprintln!("Flash erase error sector {}: {:?}", sector, e);
                return OtaReadyStatus::NoSpace;
            }
        }

        rprintln!("OTA: erased {} sectors", num_sectors);

        self.expected_size = fw_size;
        self.received_size = 0;
        self.crc = Crc32::new();
        self.write_buf = [0xFF; FLASH_WRITE_ALIGN];
        self.buf_len = 0;
        self.flash_offset = 0;

        OtaReadyStatus::Ok
    }

    fn write_chunk(&mut self, _offset: u32, data: &[u8]) -> bool {
        // Update CRC with the raw data
        self.crc.update(data);
        self.received_size += data.len() as u32;

        // Buffer data and write in FLASH_WRITE_ALIGN-byte blocks
        let mut pos = 0;
        while pos < data.len() {
            let space = FLASH_WRITE_ALIGN - self.buf_len;
            let to_copy = space.min(data.len() - pos);
            self.write_buf[self.buf_len..self.buf_len + to_copy]
                .copy_from_slice(&data[pos..pos + to_copy]);
            self.buf_len += to_copy;
            pos += to_copy;

            if self.buf_len == FLASH_WRITE_ALIGN {
                if !self.flush_buffer() {
                    return false;
                }
            }
        }

        true
    }

    fn finalize(&mut self, expected_size: u32, expected_crc: u32) -> (OtaResultStatus, u32) {
        // Flush any remaining buffered data
        if !self.flush_buffer() {
            return (OtaResultStatus::CrcError, 0);
        }

        if self.received_size != expected_size {
            rprintln!(
                "OTA: size mismatch: received {} expected {}",
                self.received_size,
                expected_size
            );
            return (OtaResultStatus::SizeMismatch, 0);
        }

        let computed_crc = self.crc.finalize();
        if computed_crc != expected_crc {
            rprintln!(
                "OTA: CRC mismatch: computed 0x{:08X} expected 0x{:08X}",
                computed_crc,
                expected_crc
            );
            return (OtaResultStatus::CrcError, computed_crc);
        }

        rprintln!("OTA: verified OK ({} bytes, CRC 0x{:08X})", self.received_size, computed_crc);
        (OtaResultStatus::Ok, computed_crc)
    }

    fn apply(&mut self) {
        // Nothing to do — firmware is already written to the app region
        rprintln!("OTA: firmware applied, ready to boot");
    }

    fn abort(&mut self) {
        rprintln!("OTA: aborted");
        self.expected_size = 0;
        self.received_size = 0;
    }
}

// ============================================================================
// CAN message helpers
// ============================================================================

/// Parse a CanMessage from a CAN frame
fn parse_frame(frame: &embassy_stm32::can::Frame) -> Option<CanMessage> {
    let id = match frame.header().id() {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(_) => return None,
    };
    CanMessage::parse(id, frame.data())
}

/// Encode a CanMessage into a CAN frame
fn to_frame(msg: &CanMessage) -> embassy_stm32::can::Frame {
    let encoded = msg.encode();
    embassy_stm32::can::Frame::new(
        Header::new(Id::Standard(encoded.id), encoded.len as u8, false),
        &encoded.data[..encoded.len],
    )
    .unwrap()
}

// ============================================================================
// Application validation & jump
// ============================================================================

/// Check if the application at APP_BASE looks valid
///
/// Validates that the initial stack pointer points to RAM
fn app_is_valid() -> bool {
    let sp = unsafe { core::ptr::read_volatile(APP_BASE as *const u32) };
    // SP must point to RAM region (0x20000000 - 0x200A0000)
    sp >= 0x2000_0000 && sp <= 0x200A_0000
}

/// Jump to the application
///
/// # Safety
/// Must only be called when app_is_valid() returns true.
/// This function never returns.
unsafe fn jump_to_app() -> ! {
    let vtor = APP_BASE;
    let initial_sp = core::ptr::read_volatile(vtor as *const u32);
    let reset_handler = core::ptr::read_volatile((vtor + 4) as *const u32);

    rprintln!("Jumping to app at 0x{:08X} (SP=0x{:08X}, Reset=0x{:08X})", vtor, initial_sp, reset_handler);

    // Disable all interrupts
    cortex_m::interrupt::disable();

    // Set VTOR to application vector table
    core::ptr::write_volatile(0xE000_ED08 as *mut u32, vtor);

    // Set MSP and jump to application reset handler
    core::arch::asm!(
        "msr MSP, {sp}",
        "bx {reset}",
        sp = in(reg) initial_sp,
        reset = in(reg) reset_handler,
        options(noreturn),
    )
}

// ============================================================================
// Main
// ============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    rtt_init_print!();
    rprintln!("=== Picotter Bootloader ===");

    // Read shared region
    let boot_magic = unsafe { core::ptr::read_volatile(BOOTLOADER_MAGIC_ADDR) };
    let panic_magic = unsafe { core::ptr::read_volatile(PANIC_MAGIC_ADDR) };

    // Read panic info before clearing (we'll send it via CAN later)
    let mut panic_buf = [0u8; PANIC_MSG_MAX];
    let mut panic_len = 0usize;
    let has_panic = panic_magic == PANIC_MAGIC;
    if has_panic {
        panic_len = unsafe { core::ptr::read_volatile(PANIC_LEN_ADDR) } as usize;
        panic_len = panic_len.min(PANIC_MSG_MAX);
        unsafe {
            core::ptr::copy_nonoverlapping(PANIC_MSG_ADDR, panic_buf.as_mut_ptr(), panic_len);
        }
        rprintln!("App panic detected ({} bytes)", panic_len);
    }

    // Always clear shared region
    unsafe {
        core::ptr::write_volatile(BOOTLOADER_MAGIC_ADDR, 0);
        core::ptr::write_volatile(PANIC_MAGIC_ADDR, 0);
    }

    let stay_in_bootloader = boot_magic == BOOTLOADER_MAGIC;
    let valid_app = app_is_valid();

    rprintln!(
        "Boot magic: 0x{:08X} (stay={}), App valid: {}, Panic: {}",
        boot_magic,
        stay_in_bootloader,
        valid_app,
        has_panic,
    );

    // If no bootloader request and app is valid, jump directly
    if !stay_in_bootloader && valid_app {
        rprintln!("Booting application...");
        unsafe { jump_to_app() };
    }

    if !valid_app {
        rprintln!("No valid application found, staying in bootloader");
    } else {
        rprintln!("Bootloader mode requested via magic word");
    }

    // ========================================================================
    // Init clocks and peripherals
    // ========================================================================
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

    // Init CAN (FDCAN1: PA11 RX / PA12 TX @ 500kbps)
    let mut can_config = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_config.set_bitrate(500_000);
    let mut can = can_config.start(OperatingMode::NormalOperationMode);

    // Init flash
    let flash = flash::Flash::new_blocking(p.FLASH);
    let mut ota = OtaReceiver::new(OtaFlashStorage::new(flash));

    rprintln!("CAN and Flash initialized, waiting for OTA...");

    // Send panic message via CAN log if we have one
    if has_panic && panic_len > 0 {
        rprintln!("Sending panic message via CAN ({} bytes)...", panic_len);
        let mut log_enc = LogEncoder::new();
        for frame in log_enc.encode(LogLevel::Error, &panic_buf[..panic_len]) {
            can.write(&to_frame(&frame)).await;
        }
        rprintln!("Panic message sent");
    }

    // ========================================================================
    // Main loop: handle CAN messages
    // ========================================================================
    loop {
        let result = select(can.read(), Timer::after_secs(OTA_TIMEOUT_SECS)).await;

        match result {
            Either::First(Ok(envelope)) => {
                if let Some(msg) = parse_frame(&envelope.frame) {
                    match msg {
                        // Respond to Ping so sabotter can detect bootloader mode
                        CanMessage::Ping { value } => {
                            rprintln!("Ping {}", value);
                            let response = cancaner::ping_response(value);
                            can.write(&to_frame(&response)).await;
                        }

                        // OTA protocol messages
                        CanMessage::OtaStart { fw_size, fw_crc32 } => {
                            rprintln!("OTA Start: {} bytes, CRC 0x{:08X}", fw_size, fw_crc32);
                            let response = ota.handle_start(fw_size, fw_crc32);
                            can.write(&to_frame(&response)).await;
                        }
                        CanMessage::OtaData {
                            chunk_idx,
                            data,
                            data_len,
                        } => {
                            let response = ota.handle_data(chunk_idx, &data, data_len);
                            can.write(&to_frame(&response)).await;
                            // Log progress every 100 chunks
                            if chunk_idx % 100 == 0 {
                                let (rx, total) = ota.progress();
                                rprintln!("OTA progress: {}/{}", rx, total);
                            }
                        }
                        CanMessage::OtaFinish => {
                            rprintln!("OTA Finish");
                            let response = ota.handle_finish();
                            can.write(&to_frame(&response)).await;
                        }
                        CanMessage::OtaReboot => {
                            rprintln!("OTA Reboot requested");
                            if ota.handle_reboot() {
                                rprintln!("Rebooting to application...");
                                Timer::after_millis(10).await; // Let CAN flush
                                SCB::sys_reset();
                            }
                        }
                        CanMessage::OtaAbort => {
                            rprintln!("OTA Abort");
                            ota.handle_abort();
                        }

                        _ => {} // Ignore non-OTA, non-Ping messages
                    }
                }
            }
            Either::First(Err(_)) => {
                // CAN error, ignore
            }
            Either::Second(_) => {
                // Timeout — if app is valid, boot it
                if valid_app {
                    rprintln!("OTA timeout, booting existing application...");
                    unsafe { jump_to_app() };
                }
                // Otherwise keep waiting
            }
        }
    }
}
