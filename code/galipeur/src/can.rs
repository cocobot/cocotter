use std::sync::{Arc, Mutex};
use std::thread;

use board_sabotter::CanBus;
use cancaner::{CanMessage, LogDecoder, LogLevel};
use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::sys::{twai_message_t, twai_receive, twai_transmit};

type Callback = Box<dyn FnMut(&CanMessage) + Send + 'static>;

#[derive(Clone)]
pub struct CanInterface {
    callbacks: Arc<Mutex<Vec<Callback>>>,
}

impl CanInterface {
    pub fn new(mut driver: CanBus) -> Self {
        let callbacks: Arc<Mutex<Vec<Callback>>> = Arc::new(Mutex::new(Vec::new()));

        driver.start().expect("TWAI start failed");
        // Leak to keep TWAI controller alive (Drop would call twai_stop + twai_driver_uninstall)
        Box::leak(Box::new(driver));

        // RX thread: blocks on twai_receive, zero polling
        let cb = callbacks.clone();
        thread::Builder::new()
            .name("can-rx".into())
            .stack_size(4096)
            .spawn(move || loop {
                let mut raw: twai_message_t = unsafe { core::mem::zeroed() };
                if unsafe { twai_receive(&mut raw, BLOCK) } == 0 {
                    // ESP_OK = 0
                    let len = raw.data_length_code as usize;
                    if let Some(msg) = CanMessage::parse(raw.identifier as u16, &raw.data[..len]) {
                        let mut cbs = cb.lock().unwrap();
                        for callback in cbs.iter_mut() {
                            callback(&msg);
                        }
                    }
                }
            })
            .expect("spawn can-rx");

        Self { callbacks }
    }

    /// Send a CAN message — calls twai_transmit directly (FreeRTOS queue, thread-safe)
    pub fn send(&self, msg: &CanMessage) {
        let encoded = msg.encode();
        let mut raw: twai_message_t = unsafe { core::mem::zeroed() };
        raw.identifier = encoded.id.as_raw() as u32;
        raw.data_length_code = encoded.len as u8;
        raw.data[..encoded.len].copy_from_slice(&encoded.data[..encoded.len]);
        log::info!("Sending CAN message: id=0x{:03x} data={:02x?}", raw.identifier, &raw.data[..raw.data_length_code as usize]);
        unsafe { twai_transmit(&raw, BLOCK) };
    }

    pub fn add_callback<F: FnMut(&CanMessage) + Send + 'static>(&self, callback: F) {
        self.callbacks.lock().unwrap().push(Box::new(callback));
    }
}

/// Register a callback that proxies CAN log frames to the Rust log crate
pub fn setup_can_log(can: &CanInterface) {
    let mut dec = LogDecoder::new();

    can.add_callback(move |msg| match msg {
        CanMessage::LogMsg {
            seq,
            level,
            payload,
            payload_len,
        } => {
            dec.process_msg(*seq, *level, &payload[..*payload_len as usize]);
        }
        CanMessage::LogCont {
            seq,
            payload,
            payload_len,
        } => {
            dec.process_cont(*seq, &payload[..*payload_len as usize]);
        }
        CanMessage::LogEnd {
            seq,
            total_len,
            payload,
            payload_len,
        } => {
            if let Some((level, data)) =
                dec.process_end(*seq, *total_len, &payload[..*payload_len as usize])
            {
                let s = core::str::from_utf8(data).unwrap_or("<utf8 error>");
                match level {
                    LogLevel::Error => log::error!(target: "picotter", "{s}"),
                    LogLevel::Warn => log::warn!(target: "picotter", "{s}"),
                    LogLevel::Info => log::info!(target: "picotter", "{s}"),
                    LogLevel::Debug => log::debug!(target: "picotter", "{s}"),
                    LogLevel::Trace => log::trace!(target: "picotter", "{s}"),
                    LogLevel::Off => {}
                }
            }
        }
        _ => {}
    });
}
