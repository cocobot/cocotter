use std::ffi::c_void;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::OnceLock;
use std::time::Duration;

use esp_idf_svc::sys;
use flume::{Receiver, Sender};

use crate::{uuid128_le, BleServer};

// ---------------------------------------------------------------------------
// Rome service UUIDs (little-endian for NimBLE)
// ---------------------------------------------------------------------------

const SERVICE_UUID_BYTES: [u8; 16] = uuid128_le(0x8187_0000_ffa549699ab4e777ca411f95);
const CHAR_TELEMETRY_UUID_BYTES: [u8; 16] = uuid128_le(0x8187_0001_ffa549699ab4e777ca411f95);
const CHAR_ORDERS_UUID_BYTES: [u8; 16] = uuid128_le(0x8187_0002_ffa549699ab4e777ca411f95);
const CHAR_LOGS_UUID_BYTES: [u8; 16] = uuid128_le(0x8187_0003_ffa549699ab4e777ca411f95);

static SVC_UUID: sys::ble_uuid128_t = sys::ble_uuid128_t {
    u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
    value: SERVICE_UUID_BYTES,
};
static CHR_TELEMETRY_UUID: sys::ble_uuid128_t = sys::ble_uuid128_t {
    u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
    value: CHAR_TELEMETRY_UUID_BYTES,
};
static CHR_ORDERS_UUID: sys::ble_uuid128_t = sys::ble_uuid128_t {
    u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
    value: CHAR_ORDERS_UUID_BYTES,
};
static CHR_LOGS_UUID: sys::ble_uuid128_t = sys::ble_uuid128_t {
    u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
    value: CHAR_LOGS_UUID_BYTES,
};

// ---------------------------------------------------------------------------
// GATT handle storage (written by ble_gatts_add_svcs, read later)
// ---------------------------------------------------------------------------

static TELEMETRY_HANDLE: AtomicU16 = AtomicU16::new(0);
static ORDERS_HANDLE: AtomicU16 = AtomicU16::new(0);
static LOGS_HANDLE: AtomicU16 = AtomicU16::new(0);

pub(crate) fn telemetry_attr_handle() -> u16 {
    TELEMETRY_HANDLE.load(Ordering::Relaxed)
}

pub(crate) fn logs_attr_handle() -> u16 {
    LOGS_HANDLE.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Channel for incoming orders (GATT write -> flume)
// ---------------------------------------------------------------------------

static ROME_RX_SENDER: OnceLock<Sender<Box<[u8]>>> = OnceLock::new();

// ---------------------------------------------------------------------------
// GATT service table builder
// ---------------------------------------------------------------------------

/// Leaked GATT service table (must live for 'static)
struct GattServiceTable {
    _chars: Box<[sys::ble_gatt_chr_def]>,
    svcs: Box<[sys::ble_gatt_svc_def]>,
}

fn build_gatt_svcs() -> &'static [sys::ble_gatt_svc_def] {
    // We need mutable pointers for val_handle. Use leaked boxes.
    let telemetry_handle_ptr = Box::leak(Box::new(0u16)) as *mut u16;
    let orders_handle_ptr = Box::leak(Box::new(0u16)) as *mut u16;
    let logs_handle_ptr = Box::leak(Box::new(0u16)) as *mut u16;

    // Store the pointers so we can read them after ble_gatts_add_svcs fills them
    HANDLE_PTRS
        .set(HandlePtrs {
            telemetry: telemetry_handle_ptr,
            orders: orders_handle_ptr,
            logs: logs_handle_ptr,
        })
        .ok()
        .expect("HANDLE_PTRS already set");

    let chars: Vec<sys::ble_gatt_chr_def> = vec![
        // Orders characteristic (write, encrypted)
        sys::ble_gatt_chr_def {
            uuid: &CHR_ORDERS_UUID.u as *const _,
            access_cb: Some(gatt_access_cb),
            arg: std::ptr::null_mut(),
            descriptors: std::ptr::null_mut(),
            flags: (sys::BLE_GATT_CHR_F_WRITE | sys::BLE_GATT_CHR_F_WRITE_ENC) as sys::ble_gatt_chr_flags,
            min_key_size: 0,
            val_handle: orders_handle_ptr,
            cpfd: std::ptr::null_mut(),
        },
        // Telemetry characteristic (notify)
        sys::ble_gatt_chr_def {
            uuid: &CHR_TELEMETRY_UUID.u as *const _,
            access_cb: Some(gatt_access_cb),
            arg: std::ptr::null_mut(),
            descriptors: std::ptr::null_mut(),
            flags: sys::BLE_GATT_CHR_F_NOTIFY as sys::ble_gatt_chr_flags,
            min_key_size: 0,
            val_handle: telemetry_handle_ptr,
            cpfd: std::ptr::null_mut(),
        },
        // Logs characteristic (notify)
        sys::ble_gatt_chr_def {
            uuid: &CHR_LOGS_UUID.u as *const _,
            access_cb: Some(gatt_access_cb),
            arg: std::ptr::null_mut(),
            descriptors: std::ptr::null_mut(),
            flags: sys::BLE_GATT_CHR_F_NOTIFY as sys::ble_gatt_chr_flags,
            min_key_size: 0,
            val_handle: logs_handle_ptr,
            cpfd: std::ptr::null_mut(),
        },
        // Terminator
        unsafe { std::mem::zeroed() },
    ];
    let chars = chars.into_boxed_slice();
    let chars_ptr = chars.as_ptr();

    let svcs: Vec<sys::ble_gatt_svc_def> = vec![
        sys::ble_gatt_svc_def {
            type_: sys::BLE_GATT_SVC_TYPE_PRIMARY as u8,
            uuid: &SVC_UUID.u as *const _,
            includes: std::ptr::null_mut(),
            characteristics: chars_ptr,
        },
        // Terminator
        unsafe { std::mem::zeroed() },
    ];
    let svcs = svcs.into_boxed_slice();

    // Leak both to get 'static lifetime
    let table = Box::leak(Box::new(GattServiceTable {
        _chars: chars,
        svcs,
    }));

    &table.svcs
}

// Storage for val_handle pointers (filled by NimBLE during ble_gatts_add_svcs)
struct HandlePtrs {
    telemetry: *mut u16,
    orders: *mut u16,
    logs: *mut u16,
}
unsafe impl Send for HandlePtrs {}
unsafe impl Sync for HandlePtrs {}

static HANDLE_PTRS: OnceLock<HandlePtrs> = OnceLock::new();

/// Copy handles from the leaked pointers into our AtomicU16s
fn copy_handles() {
    if let Some(ptrs) = HANDLE_PTRS.get() {
        TELEMETRY_HANDLE.store(unsafe { *ptrs.telemetry }, Ordering::SeqCst);
        ORDERS_HANDLE.store(unsafe { *ptrs.orders }, Ordering::SeqCst);
        LOGS_HANDLE.store(unsafe { *ptrs.logs }, Ordering::SeqCst);
        log::info!(
            "GATT handles: telemetry={}, orders={}, logs={}",
            TELEMETRY_HANDLE.load(Ordering::Relaxed),
            ORDERS_HANDLE.load(Ordering::Relaxed),
            LOGS_HANDLE.load(Ordering::Relaxed),
        );
    }
}

// ---------------------------------------------------------------------------
// GATT access callback
// ---------------------------------------------------------------------------

extern "C" fn gatt_access_cb(
    _conn_handle: u16,
    attr_handle: u16,
    ctxt: *mut sys::ble_gatt_access_ctxt,
    _arg: *mut c_void,
) -> i32 {
    let ctxt = unsafe { &*ctxt };

    #[allow(non_upper_case_globals)]
    match ctxt.op as u32 {
        sys::BLE_GATT_ACCESS_OP_WRITE_CHR => {
            if attr_handle == ORDERS_HANDLE.load(Ordering::Relaxed) {
                // Read data from mbuf
                let om = ctxt.om;
                if !om.is_null() {
                    let len = unsafe { (*om).om_len } as usize;
                    let mut buf = vec![0u8; len];
                    let rc = unsafe {
                        sys::os_mbuf_copydata(om, 0, len as i32, buf.as_mut_ptr() as *mut _)
                    };
                    if rc == 0 {
                        if let Some(sender) = ROME_RX_SENDER.get() {
                            if let Err(e) = sender.send(buf.into_boxed_slice()) {
                                log::error!("Failed to push RX data: {e:?}");
                            }
                        }
                    }
                }
            }
        }
        sys::BLE_GATT_ACCESS_OP_READ_CHR => {
            // Not used in Rome protocol
        }
        _ => {}
    }

    0
}

// ---------------------------------------------------------------------------
// RomePeripheral
// ---------------------------------------------------------------------------

pub struct RomePeripheral {
    pub sender: Sender<Box<[u8]>>,
    pub receiver: Receiver<Box<[u8]>>,
}

impl RomePeripheral {
    pub fn run(server: BleServer, name: String) -> Self {
        let (rome_rx_sender, rome_receiver) = flume::unbounded();
        let (rome_sender, rome_tx_receiver) = flume::unbounded::<Box<[u8]>>();

        // Store the orders channel sender globally for the GATT callback
        ROME_RX_SENDER
            .set(rome_rx_sender)
            .ok()
            .expect("ROME_RX_SENDER already set");

        // Build and register GATT service table
        let svcs = build_gatt_svcs();
        unsafe {
            let rc = sys::ble_gatts_count_cfg(svcs.as_ptr());
            if rc != 0 {
                panic!("ble_gatts_count_cfg failed: {rc}");
            }
            let rc = sys::ble_gatts_add_svcs(svcs.as_ptr());
            if rc != 0 {
                panic!("ble_gatts_add_svcs failed: {rc}");
            }
        }

        // Start NimBLE host task (GATT is now registered)
        server.start_host();

        // Copy handles now that the host has started and assigned them
        copy_handles();

        // Setup and start advertising
        server.setup_advertising(&name, &SERVICE_UUID_BYTES).unwrap();
        server.start_advertising().unwrap();

        // Spawn notification thread
        std::thread::spawn(move || {
            loop {
                if let Ok(data) = rome_tx_receiver.recv() {
                    let telemetry_handle = TELEMETRY_HANDLE.load(Ordering::Relaxed);
                    if telemetry_handle == 0 {
                        continue;
                    }

                    crate::with_connections(|conns| {
                        for conn in conns {
                            if conn.subscribed_telemetry {
                                // Build mbuf for notification
                                let om = unsafe {
                                    sys::os_msys_get_pkthdr(data.len() as u16, 0)
                                };
                                if om.is_null() {
                                    log::warn!("Failed to get mbuf for notification");
                                    continue;
                                }
                                let rc = unsafe {
                                    sys::os_mbuf_append(
                                        om,
                                        data.as_ptr() as *const _,
                                        data.len() as u16,
                                    )
                                };
                                if rc != 0 {
                                    unsafe { sys::os_mbuf_free_chain(om) };
                                    continue;
                                }
                                let rc = unsafe {
                                    sys::ble_gatts_notify_custom(
                                        conn.conn_handle,
                                        telemetry_handle,
                                        om,
                                    )
                                };
                                if rc != 0 {
                                    log::warn!("Notify failed for conn {}: {rc}", conn.conn_handle);
                                }
                            }
                        }
                    });
                } else {
                    log::warn!("No data recv on rome TX");
                    std::thread::sleep(Duration::from_millis(1000));
                }
            }
        });

        log::info!("ROME peripheral running");

        Self {
            sender: rome_sender,
            receiver: rome_receiver,
        }
    }
}
