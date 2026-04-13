use std::ffi::c_void;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::OnceLock;

use esp_idf_svc::sys;
use flume::{Receiver, Sender};

use crate::uuid128_le;

// ---------------------------------------------------------------------------
// OTA protocol constants
// ---------------------------------------------------------------------------

const OTA_CMD_START: u8 = 0x01;
const OTA_CMD_DATA: u8 = 0x02;
const OTA_CMD_FINISH: u8 = 0x03;
const OTA_CMD_REBOOT: u8 = 0x04;
const OTA_CMD_ABORT: u8 = 0x05;

const OTA_RSP_READY: u8 = 0x81;
const OTA_RSP_ACK: u8 = 0x82;
const OTA_RSP_RESULT: u8 = 0x83;

// ---------------------------------------------------------------------------
// OTA service UUIDs (2000 range, 1000 reserved for Rome expansion)
// ---------------------------------------------------------------------------

const OTA_SERVICE_UUID_BYTES: [u8; 16] = uuid128_le(0x8187_2000_ffa549699ab4e777ca411f95);

/// Generate characteristic UUID for a given target index (0-based)
const fn ota_char_uuid(target: u8) -> [u8; 16] {
    // 0x8187_2001 + target
    let base: u128 = 0x8187_2001_ffa549699ab4e777ca411f95;
    uuid128_le(base + (target as u128) * (1u128 << 96))
}

// Pre-generate UUIDs for up to 4 targets (can be extended)
const MAX_OTA_TARGETS: usize = 4;

static OTA_SVC_UUID: sys::ble_uuid128_t = sys::ble_uuid128_t {
    u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
    value: OTA_SERVICE_UUID_BYTES,
};

pub use board_common::hal::OtaHandler;

// ---------------------------------------------------------------------------
// OTA command/response types for channels
// ---------------------------------------------------------------------------

enum OtaCommand {
    Start { size: u32, crc: u32 },
    Data { offset: u32, data: Box<[u8]> },
    Finish,
    Reboot,
    Abort,
}

// ---------------------------------------------------------------------------
// Global state for GATT callbacks
// ---------------------------------------------------------------------------

struct OtaGlobalState {
    /// Per-target: (attr_handle, command_sender)
    targets: Vec<OtaTargetState>,
}

struct OtaTargetState {
    handle: AtomicU16,
    cmd_tx: Sender<OtaCommand>,
}

static OTA_STATE: OnceLock<OtaGlobalState> = OnceLock::new();

// Storage for val_handle pointers (filled by NimBLE during ble_gatts_add_svcs)
struct OtaHandlePtrs {
    ptrs: Vec<*mut u16>,
}
unsafe impl Send for OtaHandlePtrs {}
unsafe impl Sync for OtaHandlePtrs {}

static OTA_HANDLE_PTRS: OnceLock<OtaHandlePtrs> = OnceLock::new();

// Static UUIDs for characteristics (leaked, need 'static lifetime for NimBLE)
static OTA_CHR_UUIDS: OnceLock<Vec<&'static sys::ble_uuid128_t>> = OnceLock::new();

// ---------------------------------------------------------------------------
// GATT service table builder
// ---------------------------------------------------------------------------

struct OtaGattTable {
    _chr_uuids: Vec<Box<sys::ble_uuid128_t>>,
    _chars: Box<[sys::ble_gatt_chr_def]>,
    svcs: Box<[sys::ble_gatt_svc_def]>,
}

fn build_ota_gatt_svcs(num_targets: usize) -> &'static [sys::ble_gatt_svc_def] {
    assert!(num_targets > 0 && num_targets <= MAX_OTA_TARGETS);

    // Create and leak UUID structs for each characteristic
    let mut chr_uuids: Vec<Box<sys::ble_uuid128_t>> = Vec::with_capacity(num_targets);
    let mut chr_uuid_ptrs: Vec<&'static sys::ble_uuid128_t> = Vec::with_capacity(num_targets);
    let mut handle_ptrs: Vec<*mut u16> = Vec::with_capacity(num_targets);

    for i in 0..num_targets {
        let uuid_bytes = ota_char_uuid(i as u8);
        let uuid = Box::new(sys::ble_uuid128_t {
            u: sys::ble_uuid_t { type_: sys::BLE_UUID_TYPE_128 as u8 },
            value: uuid_bytes,
        });
        let uuid_static: &'static sys::ble_uuid128_t = Box::leak(uuid.clone());
        chr_uuid_ptrs.push(uuid_static);
        chr_uuids.push(uuid);

        let handle_ptr = Box::leak(Box::new(0u16)) as *mut u16;
        handle_ptrs.push(handle_ptr);
    }

    OTA_HANDLE_PTRS
        .set(OtaHandlePtrs { ptrs: handle_ptrs.clone() })
        .ok()
        .expect("OTA_HANDLE_PTRS already set");

    OTA_CHR_UUIDS
        .set(chr_uuid_ptrs)
        .ok()
        .expect("OTA_CHR_UUIDS already set");

    // Build characteristic definitions
    let mut chars: Vec<sys::ble_gatt_chr_def> = Vec::with_capacity(num_targets + 1);
    for i in 0..num_targets {
        let uuid_ref = OTA_CHR_UUIDS.get().unwrap();
        chars.push(sys::ble_gatt_chr_def {
            uuid: &uuid_ref[i].u as *const _,
            access_cb: Some(ota_gatt_access_cb),
            arg: std::ptr::null_mut(),
            descriptors: std::ptr::null_mut(),
            flags: (sys::BLE_GATT_CHR_F_WRITE_NO_RSP | sys::BLE_GATT_CHR_F_NOTIFY)
                as sys::ble_gatt_chr_flags,
            min_key_size: 0,
            val_handle: handle_ptrs[i],
            cpfd: std::ptr::null_mut(),
        });
    }
    // Terminator
    chars.push(unsafe { std::mem::zeroed() });

    let chars = chars.into_boxed_slice();
    let chars_ptr = chars.as_ptr();

    let svcs: Vec<sys::ble_gatt_svc_def> = vec![
        sys::ble_gatt_svc_def {
            type_: sys::BLE_GATT_SVC_TYPE_PRIMARY as u8,
            uuid: &OTA_SVC_UUID.u as *const _,
            includes: std::ptr::null_mut(),
            characteristics: chars_ptr,
        },
        // Terminator
        unsafe { std::mem::zeroed() },
    ];
    let svcs = svcs.into_boxed_slice();

    let table = Box::leak(Box::new(OtaGattTable {
        _chr_uuids: chr_uuids,
        _chars: chars,
        svcs,
    }));

    &table.svcs
}

// ---------------------------------------------------------------------------
// GATT access callback
// ---------------------------------------------------------------------------

extern "C" fn ota_gatt_access_cb(
    _conn_handle: u16,
    attr_handle: u16,
    ctxt: *mut sys::ble_gatt_access_ctxt,
    _arg: *mut c_void,
) -> i32 {
    let ctxt = unsafe { &*ctxt };

    #[allow(non_upper_case_globals)]
    match ctxt.op as u32 {
        sys::BLE_GATT_ACCESS_OP_WRITE_CHR => {
            let state = match OTA_STATE.get() {
                Some(s) => s,
                None => return sys::BLE_ATT_ERR_UNLIKELY as i32,
            };

            // Find which target this handle belongs to
            let target_idx = state
                .targets
                .iter()
                .position(|t| t.handle.load(Ordering::Relaxed) == attr_handle);

            let target_idx = match target_idx {
                Some(i) => i,
                None => return sys::BLE_ATT_ERR_UNLIKELY as i32,
            };

            // Read data from mbuf
            let om = ctxt.om;
            if om.is_null() {
                return sys::BLE_ATT_ERR_UNLIKELY as i32;
            }
            let len = unsafe { (*om).om_len } as usize;
            if len == 0 {
                return sys::BLE_ATT_ERR_UNLIKELY as i32;
            }
            let mut buf = vec![0u8; len];
            let rc = unsafe {
                sys::os_mbuf_copydata(om, 0, len as i32, buf.as_mut_ptr() as *mut _)
            };
            if rc != 0 {
                return sys::BLE_ATT_ERR_UNLIKELY as i32;
            }

            // Parse OTA command
            let cmd = match buf[0] {
                OTA_CMD_START if len >= 9 => {
                    let size = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    let crc = u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]);
                    OtaCommand::Start { size, crc }
                }
                OTA_CMD_DATA if len >= 5 => {
                    let offset = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    let data = buf[5..].to_vec().into_boxed_slice();
                    OtaCommand::Data { offset, data }
                }
                OTA_CMD_FINISH => OtaCommand::Finish,
                OTA_CMD_REBOOT => OtaCommand::Reboot,
                OTA_CMD_ABORT => OtaCommand::Abort,
                _ => return 0, // Unknown command, ignore
            };

            let _ = state.targets[target_idx].cmd_tx.try_send(cmd);
        }
        _ => {}
    }

    0
}

// ---------------------------------------------------------------------------
// Notification helper
// ---------------------------------------------------------------------------

fn send_notification(conn_handle: u16, attr_handle: u16, data: &[u8]) {
    let om = unsafe { sys::os_msys_get_pkthdr(data.len() as u16, 0) };
    if om.is_null() {
        log::warn!("OTA: failed to get mbuf for notification");
        return;
    }
    let rc = unsafe { sys::os_mbuf_append(om, data.as_ptr() as *const _, data.len() as u16) };
    if rc != 0 {
        unsafe { sys::os_mbuf_free_chain(om) };
        return;
    }
    let rc = unsafe { sys::ble_gatts_notify_custom(conn_handle, attr_handle, om) };
    if rc != 0 {
        log::warn!("OTA: notify failed for conn {conn_handle}: {rc}");
    }
}

fn notify_all_subscribers(attr_handle: u16, data: &[u8]) {
    crate::with_connections(|conns| {
        for conn in conns {
            // OTA notifications go to all connected clients (no subscription tracking needed
            // beyond what NimBLE's CCCD provides, but we send to all for simplicity)
            send_notification(conn.conn_handle, attr_handle, data);
        }
    });
}

// ---------------------------------------------------------------------------
// OtaRegistration (pre-host-start)
// ---------------------------------------------------------------------------

pub struct OtaRegistration {
    num_targets: usize,
    cmd_receivers: Vec<Receiver<OtaCommand>>,
}

/// Register OTA GATT service. Must be called before `server.start_host()`.
pub fn register_gatt(num_targets: usize) -> OtaRegistration {
    let mut cmd_senders = Vec::with_capacity(num_targets);
    let mut cmd_receivers = Vec::with_capacity(num_targets);
    let mut targets = Vec::with_capacity(num_targets);

    for _ in 0..num_targets {
        let (tx, rx) = flume::bounded(4);
        cmd_senders.push(tx.clone());
        cmd_receivers.push(rx);
        targets.push(OtaTargetState {
            handle: AtomicU16::new(0),
            cmd_tx: tx,
        });
    }

    OTA_STATE
        .set(OtaGlobalState { targets })
        .ok()
        .expect("OTA_STATE already set");

    // Build and register GATT service table
    let svcs = build_ota_gatt_svcs(num_targets);
    unsafe {
        let rc = sys::ble_gatts_count_cfg(svcs.as_ptr());
        if rc != 0 {
            panic!("OTA ble_gatts_count_cfg failed: {rc}");
        }
        let rc = sys::ble_gatts_add_svcs(svcs.as_ptr());
        if rc != 0 {
            panic!("OTA ble_gatts_add_svcs failed: {rc}");
        }
    }

    log::info!("OTA GATT service registered with {num_targets} target(s)");

    OtaRegistration {
        num_targets,
        cmd_receivers,
    }
}

// ---------------------------------------------------------------------------
// OtaService
// ---------------------------------------------------------------------------

pub struct OtaService;

impl OtaRegistration {
    /// Finalize OTA service after host has started.
    /// Copies GATT handles and spawns handler threads.
    pub fn start(self, handlers: Vec<Box<dyn OtaHandler>>) -> OtaService {
        assert_eq!(handlers.len(), self.num_targets, "handler count must match num_targets");

        // Copy handles from leaked pointers into atomics
        let state = OTA_STATE.get().unwrap();
        if let Some(ptrs) = OTA_HANDLE_PTRS.get() {
            for (i, ptr) in ptrs.ptrs.iter().enumerate() {
                let handle = unsafe { **ptr };
                state.targets[i].handle.store(handle, Ordering::SeqCst);
                log::info!("OTA target {i} handle: {handle}");
            }
        }

        // Spawn one handler thread per target
        for (i, (mut handler, cmd_rx)) in handlers
            .into_iter()
            .zip(self.cmd_receivers.into_iter())
            .enumerate()
        {
            let attr_handle = state.targets[i].handle.load(Ordering::Relaxed);
            std::thread::Builder::new()
                .name(format!("ota-{i}"))
                .stack_size(4096)
                .spawn(move || {
                    ota_handler_thread(i, attr_handle, &mut *handler, cmd_rx);
                })
                .expect("failed to spawn OTA handler thread");
        }

        log::info!("OTA service running");
        OtaService
    }
}

fn ota_handler_thread(
    target_idx: usize,
    attr_handle: u16,
    handler: &mut dyn OtaHandler,
    cmd_rx: Receiver<OtaCommand>,
) {
    loop {
        let cmd = match cmd_rx.recv() {
            Ok(cmd) => cmd,
            Err(_) => {
                log::warn!("OTA target {target_idx}: command channel closed");
                return;
            }
        };

        match cmd {
            OtaCommand::Start { size, crc } => {
                log::info!("OTA target {target_idx}: START size={size} crc=0x{crc:08x}");
                let status = handler.start(size, crc);
                let rsp = [OTA_RSP_READY, status];
                notify_all_subscribers(attr_handle, &rsp);
            }
            OtaCommand::Data { offset, data } => {
                let next_offset = handler.data(offset, &data);
                let mut rsp = [0u8; 5];
                rsp[0] = OTA_RSP_ACK;
                rsp[1..5].copy_from_slice(&next_offset.to_le_bytes());
                notify_all_subscribers(attr_handle, &rsp);
            }
            OtaCommand::Finish => {
                log::info!("OTA target {target_idx}: FINISH");
                let status = handler.finish();
                let rsp = [OTA_RSP_RESULT, status];
                notify_all_subscribers(attr_handle, &rsp);
            }
            OtaCommand::Reboot => {
                log::info!("OTA target {target_idx}: REBOOT");
                handler.reboot();
                // If handler.reboot() returns (e.g. remote target), that's fine
            }
            OtaCommand::Abort => {
                log::info!("OTA target {target_idx}: ABORT");
                handler.abort();
            }
        }
    }
}
