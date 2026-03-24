use std::sync::{Arc, Mutex};
use esp_idf_svc::{
    hal::{
        can::CanDriver,
        delay::BLOCK,
    },
    sys::{
        twai_get_status_info, twai_initiate_recovery, twai_message_t, twai_read_alerts,
        twai_receive, twai_reconfigure_alerts, twai_start, twai_status_info_t, twai_transmit,
        TWAI_ALERT_ABOVE_ERR_WARN, TWAI_ALERT_ARB_LOST, TWAI_ALERT_BUS_ERROR, TWAI_ALERT_BUS_OFF,
        TWAI_ALERT_BUS_RECOVERED, TWAI_ALERT_ERR_PASS, TWAI_ALERT_RX_FIFO_OVERRUN,
        TWAI_ALERT_RX_QUEUE_FULL, TWAI_ALERT_TX_FAILED,
    },
};
use crate::{CanMessage, LogDecoder};
use crate::types::LogLevel;


const MAX_TX_RETRIES: u32 = 3;

type Callback = Box<dyn FnMut(&CanMessage) + Send + 'static>;

#[derive(Clone)]
pub struct CanInterface {
    callbacks: Arc<Mutex<Vec<Callback>>>,
}

impl CanInterface {
    pub fn new(mut driver: CanDriver<'static>) -> Self {
        driver.start().expect("TWAI start failed");
        // Leak to keep TWAI controller alive (Drop would call twai_stop + twai_driver_uninstall)
        Box::leak(Box::new(driver));

        // Enable error alerts for monitoring + auto-recovery
        let alerts_to_enable = TWAI_ALERT_BUS_ERROR
            | TWAI_ALERT_TX_FAILED
            | TWAI_ALERT_RX_QUEUE_FULL
            | TWAI_ALERT_BUS_OFF
            | TWAI_ALERT_ABOVE_ERR_WARN
            | TWAI_ALERT_ERR_PASS
            | TWAI_ALERT_ARB_LOST
            | TWAI_ALERT_BUS_RECOVERED
            | TWAI_ALERT_RX_FIFO_OVERRUN;

        let ret = unsafe { twai_reconfigure_alerts(alerts_to_enable, core::ptr::null_mut()) };
        if ret != 0 {
            log::error!("CAN: failed to configure alerts (err={ret})");
        }

        // Monitor thread: watches for CAN alerts, logs errors, auto-recovers on bus-off
        std::thread::Builder::new()
            .name("can-mon".into())
            .stack_size(4096)
            .spawn(move || loop {
                let mut alerts: u32 = 0;
                if unsafe { twai_read_alerts(&mut alerts, BLOCK) } != 0 {
                    continue;
                }

                if alerts & TWAI_ALERT_BUS_ERROR != 0 {
                    log::warn!("CAN: bus error detected");
                }
                if alerts & TWAI_ALERT_TX_FAILED != 0 {
                    log::warn!("CAN: TX failed");
                }
                if alerts & TWAI_ALERT_ARB_LOST != 0 {
                    log::warn!("CAN: arbitration lost");
                }
                if alerts & TWAI_ALERT_RX_QUEUE_FULL != 0 {
                    log::warn!("CAN: RX queue full, messages lost");
                }
                if alerts & TWAI_ALERT_RX_FIFO_OVERRUN != 0 {
                    log::warn!("CAN: RX FIFO overrun");
                }
                if alerts & TWAI_ALERT_ABOVE_ERR_WARN != 0 {
                    log::warn!("CAN: error counter above warning threshold");
                }
                if alerts & TWAI_ALERT_ERR_PASS != 0 {
                    log::error!("CAN: entered error passive state");
                }

                // Auto-recovery: bus-off → initiate recovery
                if alerts & TWAI_ALERT_BUS_OFF != 0 {
                    log::error!("CAN: bus off! initiating recovery...");
                    let ret = unsafe { twai_initiate_recovery() };
                    if ret != 0 {
                        log::error!("CAN: recovery initiation failed (err={ret})");
                    }
                }

                // After recovery completes, driver is in STOPPED state → restart
                if alerts & TWAI_ALERT_BUS_RECOVERED != 0 {
                    log::info!("CAN: bus recovered, restarting...");
                    let ret = unsafe { twai_start() };
                    if ret != 0 {
                        log::error!("CAN: restart after recovery failed (err={ret})");
                    } else {
                        log::info!("CAN: restarted successfully");
                    }
                }

                // Log detailed status on serious errors
                if alerts
                    & (TWAI_ALERT_BUS_ERROR
                        | TWAI_ALERT_ERR_PASS
                        | TWAI_ALERT_BUS_OFF
                        | TWAI_ALERT_BUS_RECOVERED)
                    != 0
                {
                    log_can_status();
                }
            })
            .expect("spawn can-mon");

        let callbacks: Arc<Mutex<Vec<Callback>>> = Default::default();

        // RX thread: blocks on twai_receive, zero polling
        let callbacks_clone = callbacks.clone();
        std::thread::Builder::new()
            .name("can-rx".into())
            .stack_size(4096)
            .spawn(move || loop {
                let mut raw: twai_message_t = unsafe { core::mem::zeroed() };
                let ret = unsafe { twai_receive(&mut raw, BLOCK) };
                if ret == 0 {
                    let len = raw.data_length_code as usize;
                    if let Some(msg) = CanMessage::parse(raw.identifier as u16, &raw.data[..len]) {                    
                        for callback in callbacks_clone.lock().unwrap().iter_mut() {
                            callback(&msg);
                        }
                    } else {
                        log::warn!("CAN: unknown frame id=0x{:03x} len={len}", raw.identifier);
                    }
                } else {
                    log::warn!("CAN: twai_receive error (err={ret})");
                }
            })
            .expect("spawn can-rx");

        Self { callbacks }
    }

    /// Send a CAN message with automatic retry on failure
    pub fn send(&self, msg: &CanMessage) {
        let encoded = msg.encode();
        let mut raw: twai_message_t = unsafe { core::mem::zeroed() };
        raw.identifier = encoded.id.as_raw() as u32;
        raw.data_length_code = encoded.len as u8;
        raw.data[..encoded.len].copy_from_slice(&encoded.data[..encoded.len]);

        for attempt in 0..MAX_TX_RETRIES {
            let ret = unsafe { twai_transmit(&raw, BLOCK) };
            if ret == 0 {
                return;
            }
            log::warn!("CAN: twai_transmit failed (err={ret}, attempt {}/{})", attempt + 1, MAX_TX_RETRIES);
        }
        log::error!("CAN: TX failed after {MAX_TX_RETRIES} retries");
    }

    pub fn add_callback<F: FnMut(&CanMessage) + Send + 'static>(&self, callback: F) {
        self.callbacks.lock().unwrap().push(Box::new(callback));
    }

    /// Register a callback that proxies CAN log frames to the Rust log crate
    pub fn add_log_callback(&self, target: &'static str) {
        let mut dec = LogDecoder::new();
        self.add_callback(move |msg| match msg {
            CanMessage::LogMsg { seq, level, payload, payload_len } => {
                dec.process_msg(*seq, *level, &payload[..*payload_len as usize]);
            }
            CanMessage::LogCont { seq, payload, payload_len } => {
                dec.process_cont(*seq, &payload[..*payload_len as usize]);
            }
            CanMessage::LogEnd { seq, total_len, payload, payload_len } => {
                if let Some((level, data)) = dec.process_end(*seq, *total_len, &payload[..*payload_len as usize]) {
                    let s = str::from_utf8(data).unwrap_or("<utf8 error>");
                    match level {
                        LogLevel::Error => log::error!(target: target, "{s}"),
                        LogLevel::Warn => log::warn!(target: target, "{s}"),
                        LogLevel::Info => log::info!(target: target, "{s}"),
                        LogLevel::Debug => log::debug!(target: target, "{s}"),
                        LogLevel::Trace => log::trace!(target: target, "{s}"),
                        LogLevel::Off => {}
                    }
                }
            }
            _ => {}
        });
    }
}

fn log_can_status() {
    let mut status: twai_status_info_t = unsafe { core::mem::zeroed() };
    if unsafe { twai_get_status_info(&mut status) } == 0 {
        let state_str = match status.state {
            0 => &"STOPPED",
            1 => &"RUNNING",
            2 => &"BUS_OFF",
            3 => &"RECOVERING",
            _ => &"UNKNOWN",
        };
        log::warn!(
            "CAN status: state={state_str}, tx_err={}, rx_err={}, tx_failed={}, rx_missed={}, bus_err={}",
            status.tx_error_counter,
            status.rx_error_counter,
            status.tx_failed_count,
            status.rx_missed_count,
            status.bus_error_count,
        );
    }
}

