//! CAN message handler module
//!
//! can_task runs on a high-priority InterruptExecutor (TIM7), so all channels
//! must use CriticalSectionRawMutex (not ThreadModeRawMutex).

use embassy_futures::select::{select, Either};
use embassy_stm32::can::{CanRx, CanTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};
use rtt_target::rprintln;

use crate::can_logger::LOG_CHANNEL_CAPACITY;
use crate::can_protocol::{CanMessage, CanMessageFrameExt};

/// Channel capacity for CAN messages
const CHANNEL_CAPACITY: usize = 256;

/// After receiving an RX frame, suppress TX for this duration to avoid
/// interfering with incoming bursts.
const RX_QUIET_MS: u64 = 5;

/// Channel for incoming CAN commands (to be processed by cmd_task)
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> =
    Channel::new();

/// Channel for outgoing CAN status messages
pub static STATUS_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> =
    Channel::new();

/// Channel for outgoing log messages
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY> =
    Channel::new();

/// Get command receiver (for cmd_task)
pub fn cmd_receiver() -> Receiver<'static, CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> {
    CMD_CHANNEL.receiver()
}

/// Get status sender (for modules to send status)
pub fn status_sender() -> Sender<'static, CriticalSectionRawMutex, CanMessage, CHANNEL_CAPACITY> {
    STATUS_CHANNEL.sender()
}

/// Get log sender (for CanLogger)
pub fn log_sender(
) -> Sender<'static, CriticalSectionRawMutex, CanMessage, LOG_CHANNEL_CAPACITY> {
    LOG_CHANNEL.sender()
}

/// Process one RX envelope
fn process_rx(
    envelope: &embassy_stm32::can::frame::Envelope,
    count: &mut u32,
    missed: &mut u32,
    last_ping: &mut i32,
    tx_total: &mut u32,
    tx_during_burst: &mut u32,
) {
    if let Some(msg) = CanMessage::from_frame(&envelope.frame) {        
        CMD_CHANNEL.try_send(msg).ok();
    }
}

/// CAN task: RX dispatch + TX from STATUS_CHANNEL and LOG_CHANNEL
/// Runs on InterruptExecutor (TIM7) for high priority.
/// TX is suppressed while RX is active (5ms quiet period after last RX).
#[embassy_executor::task]
pub async fn can_task(mut can_rx: CanRx<'static>, mut can_tx: CanTx<'static>) {
    rprintln!("CAN task started (rx+tx)");
    let mut count: u32 = 0;
    let mut missed: u32 = 0;
    let mut last_ping: i32 = -1;
    let mut tx_total: u32 = 0;
    let mut tx_during_burst: u32 = 0;

    loop {
        // Wait for first event: RX frame or TX message
        match select(
            can_rx.read(),
            select(STATUS_CHANNEL.receive(), LOG_CHANNEL.receive()),
        )
        .await
        {
            Either::First(rx_result) => {
                // Got RX — process it, then drain RX until 5ms silence
                if let Ok(envelope) = rx_result {
                    process_rx(
                        &envelope,
                        &mut count,
                        &mut missed,
                        &mut last_ping,
                        &mut tx_total,
                        &mut tx_during_burst,
                    );
                }

                // Keep draining RX until no frame for RX_QUIET_MS
                loop {
                    match select(can_rx.read(), Timer::after(Duration::from_millis(RX_QUIET_MS)))
                        .await
                    {
                        Either::First(rx_result) => {
                            if let Ok(envelope) = rx_result {
                                process_rx(
                                    &envelope,
                                    &mut count,
                                    &mut missed,
                                    &mut last_ping,
                                    &mut tx_total,
                                    &mut tx_during_burst,
                                );
                            }
                        }
                        Either::Second(_timeout) => {
                            // 5ms without RX — safe to TX now
                            break;
                        }
                    }
                }


            }
            Either::Second(tx_either) => {
                // No RX pending, safe to TX immediately
                let msg = match tx_either {
                    Either::First(m) | Either::Second(m) => m,
                };
                let frame = msg.to_frame();
                let tx = can_tx.write(&frame).await; 
                if tx.is_some() {
                    rprintln!("Error sending CAN frame: {:?}/ tx{:?}", frame, tx);
                }
                tx_total += 1;
                if last_ping >= 0 {
                    tx_during_burst += 1;
                }
            }
        }
    }
}
