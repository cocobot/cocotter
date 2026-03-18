//! CAN message handler module
//!
//! Split RX/TX architecture:
//! - `can_rx_task`: blocks on CAN receive, dispatches to CMD_CHANNEL
//! - `can_tx_task`: awaits on STATUS_CHANNEL and LOG_CHANNEL, sends frames
//! - `can_monitor_task`: periodic error counter / bus state monitoring via RTT

use embassy_stm32::can::{CanRx, CanTx, Properties};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::Timer;
use embedded_can::Id;
use rtt_target::rprintln;
use cortex_m::peripheral::SCB;

use crate::can_logger::{self, LOG_CHANNEL_CAPACITY};
use crate::can_protocol::{CanMessage, CanMessageFrameExt};

/// Channel capacity for CAN messages (increased from 8 to 32)
/// Buffers messages during bus errors/BusWarning and prevents message loss
const CHANNEL_CAPACITY: usize = 32;

/// Threshold of consecutive errors before forcing CAN controller reset
/// After this many errors, we force a system reset to clear the CAN state immediately
const ERROR_THRESHOLD_FOR_RESET: u32 = 10000;

/// Channel for incoming CAN commands (to be processed by cmd_task)
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> =
    Channel::new();

/// Channel for outgoing CAN status messages
pub static STATUS_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> =
    Channel::new();

/// Channel for outgoing log messages (higher capacity)
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY> = Channel::new();

/// Get command receiver (for cmd_task)
pub fn cmd_receiver(
) -> Receiver<'static, CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> {
    CMD_CHANNEL.receiver()
}

/// Get status sender (for modules to send status)
pub fn status_sender(
) -> Sender<'static, CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> {
    STATUS_CHANNEL.sender()
}

/// Get log sender (for CanLogger)
pub fn log_sender() -> Sender<'static, CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY> {
    LOG_CHANNEL.sender()
}

/// CAN RX task - receives frames and dispatches to command channel
#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) {
    rprintln!("CAN RX task started");
    let mut consecutive_errors: u32 = 0;
    let mut dropped_count: u32 = 0;

    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                if consecutive_errors > 0 {
                    rprintln!("CAN: Recovered after {} errors (dropped {} msgs)", consecutive_errors, dropped_count);
                    dropped_count = 0;
                }
                consecutive_errors = 0;
                let frame = &envelope.frame;
                let id = match frame.header().id() {
                    Id::Standard(id) => id.as_raw(),
                    Id::Extended(_) => continue,
                };

                if let Some(msg) = CanMessage::from_frame(frame) {
                    match &msg {
                        CanMessage::Ping { value } => {
                            rprintln!("CAN RX ping {}", value);
                            let response = cancaner::ping_response(*value);
                            STATUS_CHANNEL.try_send(response).ok();
                        }
                        CanMessage::LogConfig { level } => {
                            can_logger::set_log_level(*level);
                            rprintln!("Log level set to {:?}", level);
                        }
                        _ => {
                            if CMD_CHANNEL.try_send(msg).is_err() {
                                dropped_count += 1;
                                // Only warn on first drop or once per 10 drops to avoid flooding
                                if dropped_count == 1 || dropped_count % 10 == 0 {
                                    rprintln!("CAN: cmd channel full, dropped {} msgs (last: 0x{:03X})", dropped_count, id);
                                }
                            }
                        }
                    }
                }
            }
            Err(e) => {
                consecutive_errors += 1;

                // Log first 5 errors, then every 100th to avoid console flooding
                if consecutive_errors <= 5 || consecutive_errors % 100 == 0 {
                    rprintln!("CAN RX err #{}: {:?}", consecutive_errors, e);
                }

                // Rate-limit error loop: without this, BusWarning errors drain instantly
                // and generate thousands of errors per second in a tight busy-loop
                Timer::after_millis(10).await;

                // Force hard reset after too many consecutive errors to recover immediately
                // This doesn't wait for automatic bus recovery
                if consecutive_errors >= ERROR_THRESHOLD_FOR_RESET {
                    rprintln!("CAN: HARD RESET - {} consecutive errors exceeded threshold", consecutive_errors);
                    SCB::sys_reset();
                }
            }
        }
    }
}

/// CAN TX task - sends status and log messages
#[embassy_executor::task]
pub async fn can_tx_task(mut can_tx: CanTx<'static>) {
    rprintln!("CAN TX task started");

    loop {
        // Drain any pending status messages first (priority)
        while let Ok(msg) = STATUS_CHANNEL.try_receive() {
            let frame = msg.to_frame();
            can_tx.write(&frame).await;
        }

        // Drain any pending log messages
        while let Ok(msg) = LOG_CHANNEL.try_receive() {
            let frame = msg.to_frame();
            can_tx.write(&frame).await;
        }

        // Wait for either channel to have a message
        use embassy_futures::select::{select, Either};
        match select(STATUS_CHANNEL.receive(), LOG_CHANNEL.receive()).await {
            Either::First(msg) => {
                let frame = msg.to_frame();
                can_tx.write(&frame).await;
            }
            Either::Second(msg) => {
                let frame = msg.to_frame();
                can_tx.write(&frame).await;
            }
        }
    }
}

/// CAN bus monitor task - monitors error counters and triggers aggressive recovery
///
/// Watches the REC (Rx Error Counter) which indicates bus health:
/// - REC < 96: Normal operation
/// - REC >= 96: Error warning state
/// - REC >= 128: Error passive state (cannot receive)
/// - REC >= 256: Bus-off state (stops all activity)
///
/// If REC stays >192 for 5+ seconds, forces a hard reset to recover quickly
/// without waiting for automatic bus recovery.
#[embassy_executor::task]
pub async fn can_monitor_task(properties: Properties) {
    rprintln!("CAN monitor task started");
    let mut last_tec: u8 = 0;
    let mut last_rec: u8 = 0;
    let mut high_error_count: u32 = 0;

    loop {
        Timer::after_millis(500).await;

        let tec = properties.tx_error_count();
        let rec = properties.rx_error_count();
        let mode = properties.bus_error_mode();

        // Log when error counters change or are non-zero
        if tec != last_tec || rec != last_rec || tec > 0 || rec > 0 {
            rprintln!("CAN: TEC={} REC={} mode={:?}", tec, rec, mode);
            last_tec = tec;
            last_rec = rec;
        }

        // If REC stays critically high (>192 = very close to bus-off), count consecutive cycles
        if rec > 192 {
            high_error_count += 1;
            if high_error_count == 1 || high_error_count % 2 == 0 {
                rprintln!("CAN: WARNING - REC={} (critical, {} x 0.5s)", rec, high_error_count);
            }
            // After 10 cycles of high REC (5 seconds), force a hard reset
            if high_error_count >= 10 {
                rprintln!("CAN: CRITICAL - REC stayed >192 for 5s, forcing HARD RESET");
                SCB::sys_reset();
            }
        } else {
            high_error_count = 0;
        }
    }
}
