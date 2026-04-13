//! CAN Logger implementation
//!
//! Implements the `log::Log` trait to send log messages over CAN bus.
//! Replaces rprintln! calls and allows remote debugging via sabotter.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use cortex_m::interrupt::Mutex;

use crate::can_protocol::{CanMessage, LogEncoder, LogLevel};

/// Maximum log message length
const MAX_LOG_LEN: usize = 128;

/// Log channel capacity
pub const LOG_CHANNEL_CAPACITY: usize = 64;

/// Global log level filter (can be changed at runtime via CAN)
static LOG_LEVEL_FILTER: AtomicU8 = AtomicU8::new(LogLevel::Info as u8);

/// Static log encoder with sequence counter (protected by critical section)
static LOG_ENCODER: Mutex<UnsafeCell<LogEncoder>> =
    Mutex::new(UnsafeCell::new(LogEncoder::new()));

/// Set the log level filter
pub fn set_log_level(level: LogLevel) {
    LOG_LEVEL_FILTER.store(level as u8, Ordering::Relaxed);
}

/// Get current log level filter
pub fn get_log_level() -> LogLevel {
    LogLevel::from_u8(LOG_LEVEL_FILTER.load(Ordering::Relaxed))
}

/// Convert log::Level to our LogLevel
fn log_level_from_std(level: log::Level) -> LogLevel {
    match level {
        log::Level::Error => LogLevel::Error,
        log::Level::Warn => LogLevel::Warn,
        log::Level::Info => LogLevel::Info,
        log::Level::Debug => LogLevel::Debug,
        log::Level::Trace => LogLevel::Trace,
    }
}

/// Convert our LogLevel to log::LevelFilter
fn level_filter_to_std(level: LogLevel) -> log::LevelFilter {
    match level {
        LogLevel::Off => log::LevelFilter::Off,
        LogLevel::Error => log::LevelFilter::Error,
        LogLevel::Warn => log::LevelFilter::Warn,
        LogLevel::Info => log::LevelFilter::Info,
        LogLevel::Debug => log::LevelFilter::Debug,
        LogLevel::Trace => log::LevelFilter::Trace,
    }
}

/// Global logger instance (requires init)
static GLOBAL_LOGGER: Mutex<UnsafeCell<Option<CanLoggerGlobal>>> =
    Mutex::new(UnsafeCell::new(None));

/// Static reference to logger for log crate
static LOGGER_REF: StaticLogger = StaticLogger;

struct StaticLogger;

struct CanLoggerGlobal {
    sender: Sender<'static, CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY>,
}

// Safety: CanLoggerGlobal contains a Sender which is Send+Sync
unsafe impl Send for CanLoggerGlobal {}
unsafe impl Sync for CanLoggerGlobal {}

impl log::Log for StaticLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        let filter = get_log_level();
        if filter == LogLevel::Off {
            return false;
        }
        let level = log_level_from_std(metadata.level());
        (level as u8) <= (filter as u8)
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let level = log_level_from_std(record.level());

        let mut buffer = [0u8; MAX_LOG_LEN];
        let len = format_log_record(record, &mut buffer);

        if len > 0 {
            cortex_m::interrupt::free(|cs| {
                let logger_cell = GLOBAL_LOGGER.borrow(cs);
                // Safety: we're in a critical section
                let logger_opt = unsafe { &*logger_cell.get() };
                if let Some(logger) = logger_opt {
                    let encoder_cell = LOG_ENCODER.borrow(cs);
                    // Safety: we're in a critical section
                    let encoder = unsafe { &mut *encoder_cell.get() };
                    let frames = encoder.encode(level, &buffer[..len]);
                    for frame in frames {
                        if logger.sender.try_send(frame).is_err() {
                            break;
                        }
                    }
                }
            });
        }
    }

    fn flush(&self) {}
}

/// Format a log record into the buffer, returns bytes written
fn format_log_record(record: &log::Record, buffer: &mut [u8]) -> usize {
    use core::fmt::Write;

    struct BufferWriter<'a> {
        buffer: &'a mut [u8],
        pos: usize,
    }

    impl<'a> Write for BufferWriter<'a> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let space = self.buffer.len() - self.pos;
            let to_copy = bytes.len().min(space);
            self.buffer[self.pos..self.pos + to_copy].copy_from_slice(&bytes[..to_copy]);
            self.pos += to_copy;
            Ok(())
        }
    }

    let mut writer = BufferWriter { buffer, pos: 0 };

    let target = record.target();
    if !target.is_empty() && target.len() < 20 {
        let _ = write!(writer, "{}: ", target);
    }
    let _ = write!(writer, "{}", record.args());

    writer.pos
}

/// Initialize the CAN logger as the global logger
///
/// # Safety
/// Must only be called once, at the start of the program.
pub unsafe fn init(
    sender: Sender<'static, CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY>,
) {
    cortex_m::interrupt::free(|cs| {
        let logger_cell = GLOBAL_LOGGER.borrow(cs);
        // Safety: we're in a critical section
        *logger_cell.get() = Some(CanLoggerGlobal { sender });
    });

    let _ = log::set_logger(&LOGGER_REF);
    log::set_max_level(level_filter_to_std(LogLevel::Trace));
}
