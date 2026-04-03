pub mod ota;
pub mod ota_esp;
pub mod rome;
pub mod scan;

use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Condvar, Mutex, OnceLock};

use esp_idf_svc::sys::{self, EspError, ESP_FAIL};

/// Extended advertising instance ID
const EXT_ADV_INSTANCE: u8 = 0;

pub use ota::{OtaHandler, OtaService};
pub use ota_esp::EspSelfOtaHandler;
pub use rome::{RomePeripheral, RomeRegistration};
pub use scan::BleScanResult;

// ---------------------------------------------------------------------------
// Global BLE state (accessible from extern "C" callbacks)
// ---------------------------------------------------------------------------

static BLE_STATE: OnceLock<BleState> = OnceLock::new();

struct BleState {
    own_addr_type: AtomicU8,
    synced: AtomicBool,
    sync_notify: (Mutex<bool>, Condvar),
    connections: Mutex<Vec<Connection>>,
    handlers: BleGapHandlers,
}

pub(crate) struct Connection {
    pub conn_handle: u16,
    pub subscribed_telemetry: bool,
    pub subscribed_logs: bool,
}

impl Connection {
    fn new(conn_handle: u16) -> Self {
        Self {
            conn_handle,
            subscribed_telemetry: false,
            subscribed_logs: false,
        }
    }
}

/// Get a snapshot of connections with their subscription state
pub(crate) fn with_connections<F, R>(f: F) -> R
where
    F: FnOnce(&[Connection]) -> R,
{
    let state = BLE_STATE.get().unwrap();
    let conns = state.connections.lock().unwrap();
    f(&conns)
}

fn own_addr_type() -> u8 {
    BLE_STATE.get().unwrap().own_addr_type.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// UUID helper
// ---------------------------------------------------------------------------

/// Convert a u128 UUID to NimBLE's little-endian byte format
pub const fn uuid128_le(uuid: u128) -> [u8; 16] {
    uuid.to_le_bytes()
}

// ---------------------------------------------------------------------------
// BleServer
// ---------------------------------------------------------------------------

const MAX_CONNECTIONS: usize = 1;

pub struct BleServer {
    host_started: AtomicBool,
}

impl BleServer {
    /// Start the NimBLE host task. Must be called after GATT services are registered.
    pub fn start_host(&self) {
        if self.host_started.swap(true, Ordering::SeqCst) {
            log::warn!("NimBLE host already started");
            return;
        }
        unsafe {
            sys::nimble_port_freertos_init(Some(nimble_host_task));
        }
        // Wait for host sync
        let state = BLE_STATE.get().unwrap();
        let (lock, cvar) = &state.sync_notify;
        let mut synced = lock.lock().unwrap();
        while !*synced {
            synced = cvar.wait(synced).unwrap();
        }
        log::info!("NimBLE host synced and ready");
    }

    pub fn start_advertising(&self) -> Result<(), EspError> {
        log::info!("Start extended advertising");
        let rc = unsafe { sys::ble_gap_ext_adv_start(EXT_ADV_INSTANCE, 0, 0) };
        if rc != 0 {
            log::error!("ble_gap_ext_adv_start failed: {rc}");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }

    pub fn stop_advertising(&self) -> Result<(), EspError> {
        log::info!("Stop extended advertising");
        let rc = unsafe { sys::ble_gap_ext_adv_stop(EXT_ADV_INSTANCE) };
        if rc != 0 {
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }

    /// Setup extended advertising parameters and data (BLE 5.0, 1M PHY)
    pub fn setup_advertising(&self, device_name: &str, uuid_bytes: &[u8; 16]) -> Result<(), EspError> {
        let name_cstr: Vec<u8> = device_name.bytes().chain(std::iter::once(0)).collect();
        unsafe {
            sys::ble_svc_gap_device_name_set(name_cstr.as_ptr() as *const _);
        }

        // Configure ext adv instance
        let mut params: sys::ble_gap_ext_adv_params = unsafe { std::mem::zeroed() };
        params.set_connectable(1);
        params.set_scannable(0);
        params.set_legacy_pdu(0);
        params.own_addr_type = own_addr_type();
        params.primary_phy = sys::BLE_HCI_LE_PHY_1M as u8;
        params.secondary_phy = sys::BLE_HCI_LE_PHY_1M as u8;
        params.sid = 0;
        params.itvl_min = 0x50; // 50ms
        params.itvl_max = 0xA0; // 100ms
        params.tx_power = 127;

        let mut selected_tx_power: i8 = 0;
        let rc = unsafe {
            sys::ble_gap_ext_adv_configure(
                EXT_ADV_INSTANCE,
                &params,
                &mut selected_tx_power,
                Some(gap_event_handler),
                std::ptr::null_mut(),
            )
        };
        if rc != 0 {
            log::error!("ble_gap_ext_adv_configure failed: {rc}");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        log::info!("Ext adv configured, tx_power: {selected_tx_power}");

        // Build advertising data
        let mut adv_data: Vec<u8> = Vec::new();
        // Flags: LE General Discoverable + BR/EDR Not Supported
        adv_data.extend_from_slice(&[0x02, 0x01, 0x06]);
        // Complete Local Name
        let name_bytes = device_name.as_bytes();
        adv_data.push((name_bytes.len() + 1) as u8);
        adv_data.push(0x09);
        adv_data.extend_from_slice(name_bytes);
        // Complete List of 128-bit Service UUIDs
        adv_data.push(17); // 16 bytes + 1 type byte
        adv_data.push(0x07);
        adv_data.extend_from_slice(uuid_bytes);

        log::info!("Extended advertising data: {} bytes", adv_data.len());

        unsafe {
            let buf = sys::os_msys_get_pkthdr(adv_data.len() as u16, 0);
            if buf.is_null() {
                log::error!("Failed to get mbuf for adv data");
                return Err(EspError::from_infallible::<ESP_FAIL>());
            }
            let rc = sys::os_mbuf_append(buf, adv_data.as_ptr() as *const _, adv_data.len() as u16);
            if rc != 0 {
                sys::os_mbuf_free_chain(buf);
                return Err(EspError::from_infallible::<ESP_FAIL>());
            }
            let rc = sys::ble_gap_ext_adv_set_data(EXT_ADV_INSTANCE, buf);
            if rc != 0 {
                log::error!("ble_gap_ext_adv_set_data failed: {rc}");
                return Err(EspError::from_infallible::<ESP_FAIL>());
            }
        }

        Ok(())
    }

    /// Enable BLE security (Secure Connections, display-only IO)
    pub fn enable_security(&self) {
        unsafe {
            let cfg = std::ptr::addr_of_mut!(sys::ble_hs_cfg);
            (*cfg).sm_io_cap = sys::BLE_SM_IO_CAP_DISP_ONLY as u8;
            sys::ble_hs_cfg::set_sm_bonding_raw(cfg, 0);
            sys::ble_hs_cfg::set_sm_sc_raw(cfg, 1);
            sys::ble_hs_cfg::set_sm_mitm_raw(cfg, 0);
            (*cfg).sm_our_key_dist = 3;   // ENC | ID
            (*cfg).sm_their_key_dist = 3;  // ENC | ID
        }
    }

    pub fn set_peer_encryption(&self, conn_handle: u16) -> Result<(), EspError> {
        let rc = unsafe { sys::ble_gap_security_initiate(conn_handle) };
        if rc != 0 {
            log::error!("ble_gap_security_initiate failed: {rc}");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }

    pub fn set_conn_params_conf(
        &self,
        conn_handle: u16,
        min_int_ms: u32,
        max_int_ms: u32,
        latency: u16,
        timeout_ms: u32,
    ) -> Result<(), EspError> {
        // Convert ms to 1.25ms units for interval, 10ms units for timeout
        let params = sys::ble_gap_upd_params {
            itvl_min: ((min_int_ms * 1000) / 1250) as u16,
            itvl_max: ((max_int_ms * 1000) / 1250) as u16,
            latency,
            supervision_timeout: (timeout_ms / 10) as u16,
            min_ce_len: 0,
            max_ce_len: 0,
        };
        let rc = unsafe { sys::ble_gap_update_params(conn_handle, &params) };
        if rc != 0 {
            log::error!("ble_gap_update_params failed: {rc}");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// BleClient
// ---------------------------------------------------------------------------

pub struct BleClient;

impl BleClient {
    pub fn start_scanning(&self, duration_ms: u32) -> Result<(), EspError> {
        let mut disc_params: sys::ble_gap_disc_params = unsafe { std::mem::zeroed() };
        disc_params.itvl = 50;
        disc_params.window = 48;
        disc_params.filter_policy = 0;
        disc_params.set_passive(0);
        disc_params.set_limited(0);
        disc_params.set_filter_duplicates(1);

        let rc = unsafe {
            sys::ble_gap_disc(
                own_addr_type(),
                duration_ms as i32,
                &disc_params,
                Some(gap_event_handler),
                std::ptr::null_mut(),
            )
        };
        if rc != 0 {
            log::error!("ble_gap_disc failed: {rc}");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }

    pub fn stop_scanning(&self) -> Result<(), EspError> {
        let rc = unsafe { sys::ble_gap_disc_cancel() };
        if rc != 0 {
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// BleBuilder
// ---------------------------------------------------------------------------

pub struct BleBuilder {
    handlers: BleGapHandlers,
}

#[derive(Default)]
struct BleGapHandlers {
    on_scan_result: Option<Box<dyn Fn(BleScanResult) + Send + Sync + 'static>>,
    on_passkey_notification: Option<Box<dyn Fn([u8; 6], u32) + Send + Sync + 'static>>,
}

impl BleBuilder {
    pub fn new() -> Self {
        Self {
            handlers: Default::default(),
        }
    }

    pub fn with_scanner<F: Fn(BleScanResult) + Send + Sync + 'static>(mut self, f: F) -> Self {
        self.handlers.on_scan_result = Some(Box::new(f));
        self
    }

    pub fn with_passkey_notifier<F: Fn([u8; 6], u32) + Send + Sync + 'static>(mut self, f: F) -> Self {
        self.handlers.on_passkey_notification = Some(Box::new(f));
        self
    }

    /// Initialize NimBLE and return (server, client).
    ///
    /// GATT services must be registered before starting the host task.
    /// Call `server.start_host()` (done by `RomePeripheral::run()`) after registration.
    pub fn run(self) -> (BleServer, BleClient) {
        // Init NVS (safe to call multiple times)
        unsafe {
            let ret = sys::nvs_flash_init();
            if ret == sys::ESP_ERR_NVS_NO_FREE_PAGES || ret == sys::ESP_ERR_NVS_NEW_VERSION_FOUND {
                sys::nvs_flash_erase();
                sys::nvs_flash_init();
            }
        }

        // Initialize NimBLE port (controller + host)
        unsafe {
            let ret = sys::nimble_port_init();
            if ret != 0 {
                panic!("nimble_port_init failed: {ret}");
            }
        }

        // Configure host callbacks
        unsafe {
            sys::ble_hs_cfg.sync_cb = Some(on_sync);
            sys::ble_hs_cfg.reset_cb = Some(on_reset);
        }

        // Init GAP and GATT services
        unsafe {
            sys::ble_svc_gap_init();
            sys::ble_svc_gatt_init();
            // Request max ATT MTU for OTA throughput
            let rc = sys::ble_att_set_preferred_mtu(512);
            if rc != 0 {
                log::warn!("ble_att_set_preferred_mtu failed: {rc}");
            }
        }

        // Store global state
        BLE_STATE
            .set(BleState {
                own_addr_type: AtomicU8::new(0),
                synced: AtomicBool::new(false),
                sync_notify: (Mutex::new(false), Condvar::new()),
                connections: Mutex::new(Vec::new()),
                handlers: self.handlers,
            })
            .ok()
            .expect("BLE already initialized");

        log::info!("NimBLE initialized, host task not yet started");

        let server = BleServer {
            host_started: AtomicBool::new(false),
        };
        let client = BleClient;

        (server, client)
    }
}

// ---------------------------------------------------------------------------
// NimBLE callbacks
// ---------------------------------------------------------------------------

extern "C" fn nimble_host_task(_param: *mut c_void) {
    log::info!("NimBLE host task running");
    unsafe {
        sys::nimble_port_run();
        sys::nimble_port_freertos_deinit();
    }
}

extern "C" fn on_reset(reason: i32) {
    log::error!("NimBLE host reset, reason: {reason}");
}

extern "C" fn on_sync() {
    log::info!("NimBLE host synced");

    let state = BLE_STATE.get().unwrap();

    unsafe {
        let mut addr_type: u8 = 0;
        let rc = sys::ble_hs_id_infer_auto(0, &mut addr_type);
        if rc != 0 {
            log::error!("ble_hs_id_infer_auto failed: {rc}");
            return;
        }
        state.own_addr_type.store(addr_type, Ordering::SeqCst);

        let mut addr = [0u8; 6];
        sys::ble_hs_id_copy_addr(addr_type, addr.as_mut_ptr(), std::ptr::null_mut());
        log::info!(
            "BLE address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
        );
    }

    state.synced.store(true, Ordering::SeqCst);
    let (lock, cvar) = &state.sync_notify;
    *lock.lock().unwrap() = true;
    cvar.notify_all();
}

// ---------------------------------------------------------------------------
// GAP event handler
// ---------------------------------------------------------------------------

extern "C" fn gap_event_handler(event: *mut sys::ble_gap_event, _arg: *mut c_void) -> i32 {
    let ev = unsafe { &*event };
    let state = BLE_STATE.get().unwrap();

    #[allow(non_upper_case_globals)]
    match ev.type_ as u32 {
        sys::BLE_GAP_EVENT_CONNECT => {
            let connect = unsafe { &ev.__bindgen_anon_1.connect };
            if connect.status == 0 {
                log::info!("Connected, handle: {}", connect.conn_handle);
                // Request max Data Length Extension (251 octets, 2120 us)
                let rc = unsafe {
                    sys::ble_hs_hci_util_set_data_len(connect.conn_handle, 251, 2120)
                };
                if rc != 0 {
                    log::warn!("DLE request failed: {rc}");
                }
                // Request fast connection interval (7.5ms–15ms) for OTA throughput
                let conn_params = sys::ble_gap_upd_params {
                    itvl_min: 6,   // 6 × 1.25ms = 7.5ms
                    itvl_max: 12,  // 12 × 1.25ms = 15ms
                    latency: 0,
                    supervision_timeout: 200, // 200 × 10ms = 2s
                    min_ce_len: 0,
                    max_ce_len: 0,
                };
                let rc = unsafe {
                    sys::ble_gap_update_params(connect.conn_handle, &conn_params)
                };
                if rc != 0 {
                    log::warn!("Connection param update failed: {rc}");
                }
                let mut conns = state.connections.lock().unwrap();
                if conns.len() < MAX_CONNECTIONS {
                    conns.push(Connection::new(connect.conn_handle));
                }
            } else {
                log::error!("Connection failed: {}", connect.status);
                let _ = unsafe { sys::ble_gap_ext_adv_start(EXT_ADV_INSTANCE, 0, 0) };
            }
        }
        sys::BLE_GAP_EVENT_DISCONNECT => {
            let disconnect = unsafe { &ev.__bindgen_anon_1.disconnect };
            log::info!("Disconnected, reason: {}", disconnect.reason);
            {
                let mut conns = state.connections.lock().unwrap();
                if let Some(idx) = conns.iter().position(|c| c.conn_handle == disconnect.conn.conn_handle) {
                    conns.swap_remove(idx);
                }
            }
            // Restart advertising
            let _ = unsafe { sys::ble_gap_ext_adv_start(EXT_ADV_INSTANCE, 0, 0) };
        }
        sys::BLE_GAP_EVENT_ADV_COMPLETE => {
            log::info!("Advertising complete, restarting...");
            let _ = unsafe { sys::ble_gap_ext_adv_start(EXT_ADV_INSTANCE, 0, 0) };
        }
        sys::BLE_GAP_EVENT_SUBSCRIBE => {
            let sub = unsafe { &ev.__bindgen_anon_1.subscribe };
            let cur_notify = sub.cur_notify() != 0;
            log::info!(
                "Subscribe: conn={}, attr={}, notify={}",
                sub.conn_handle, sub.attr_handle, cur_notify
            );
            let mut conns = state.connections.lock().unwrap();
            if let Some(conn) = conns.iter_mut().find(|c| c.conn_handle == sub.conn_handle) {
                let telemetry_handle = rome::telemetry_attr_handle();
                let logs_handle = rome::logs_attr_handle();
                if sub.attr_handle == telemetry_handle {
                    conn.subscribed_telemetry = cur_notify;
                    log::info!("Client {} subscribed to telemetry: {}", sub.conn_handle, cur_notify);
                } else if sub.attr_handle == logs_handle {
                    conn.subscribed_logs = cur_notify;
                    log::info!("Client {} subscribed to logs: {}", sub.conn_handle, cur_notify);
                }
            }
        }
        sys::BLE_GAP_EVENT_MTU => {
            let mtu = unsafe { &ev.__bindgen_anon_1.mtu };
            log::info!("MTU update: conn={}, mtu={}", mtu.conn_handle, mtu.value);
        }
        sys::BLE_GAP_EVENT_ENC_CHANGE => {
            let enc = unsafe { &ev.__bindgen_anon_1.enc_change };
            if enc.status == 0 {
                log::info!("Encryption enabled for conn {}", enc.conn_handle);
            } else {
                log::error!("Encryption change failed: {}", enc.status);
            }
        }
        sys::BLE_GAP_EVENT_PASSKEY_ACTION => {
            let passkey = unsafe { &ev.__bindgen_anon_1.passkey };
            log::info!("Passkey action, conn: {}", passkey.conn_handle);
            if let Some(handler) = &state.handlers.on_passkey_notification {
                // Get peer address
                let mut desc: sys::ble_gap_conn_desc = unsafe { std::mem::zeroed() };
                let rc = unsafe { sys::ble_gap_conn_find(passkey.conn_handle, &mut desc) };
                if rc == 0 {
                    let addr = desc.peer_ota_addr.val;
                    let key = passkey.params.numcmp;
                    handler(addr, key);
                }
            }
        }
        sys::BLE_GAP_EVENT_DISC => {
            if let Some(handler) = &state.handlers.on_scan_result {
                let disc = unsafe { &ev.__bindgen_anon_1.disc };
                let data = if disc.length_data > 0 && !disc.data.is_null() {
                    unsafe { std::slice::from_raw_parts(disc.data, disc.length_data as usize) }.to_vec()
                } else {
                    Vec::new()
                };
                handler(BleScanResult {
                    addr: disc.addr.val,
                    addr_type: disc.addr.type_,
                    rssi: disc.rssi,
                    data,
                });
            }
        }
        _ => {}
    }

    0
}
