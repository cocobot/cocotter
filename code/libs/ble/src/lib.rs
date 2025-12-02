pub mod rome;
pub mod scan;

use std::sync::Arc;
use esp_idf_svc::bt::{
    ble::{
        gap::{
            AdvConfiguration,
            AuthenticationRequest,
            BleGapEvent,
            BleEncryption,
            EspBleGap,
            IOCapabilities,
            KeyMask,
            SecurityConfiguration,
        },
        gatt:: server::EspGatts,
    },
    BdAddr, Ble, BtDriver, BtStatus, BtUuid,
};
use esp_idf_svc::hal::sys::esp;
use esp_idf_svc::sys::{self, EspError, ESP_BLE_ADV_FLAG_GEN_DISC};

pub use rome::RomePeripheral;
pub use scan::BleScanResult;


/// Wrapper structure for peripheral implementation
///
/// Provide limited access to GAP.
pub struct BleServer {
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    pub gatts: EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>,
}

impl BleServer {
    pub fn start_advertising(&self) -> Result<(), EspError> {
        log::info!("Start advertising");
        self.gap.start_advertising()
    }

    pub fn stop_advertising(&self) -> Result<(), EspError> {
        log::info!("Stop advertising");
        self.gap.stop_advertising()
    }

    pub fn set_conn_params_conf(
        &self,
        addr: BdAddr,
        min_int_ms: u32,
        max_int_ms: u32,
        latency_ms: u32,
        timeout_ms: u32,
    ) -> Result<(), EspError> {
        self.gap.set_conn_params_conf(addr, min_int_ms, max_int_ms, latency_ms, timeout_ms)
    }

    /// Setup and start advertising
    pub fn setup_advertising(&self, device_name: &str, uuid: BtUuid) -> Result<(), EspError> {
        self.gap.set_device_name(device_name)?;
        self.gap.set_adv_conf(&AdvConfiguration {
            include_name: true,
            include_txpower: true,
            flag: ESP_BLE_ADV_FLAG_GEN_DISC as _,
            service_uuid: Some(uuid),
            ..Default::default()
        })
    }

    /// Enable server security
    ///
    /// If enabled, `set_peer_encryption()` must be called on `PeerConnected` event.
    /// TODO: Don't hardcode security parameters.
    pub fn enable_security(&self) -> Result<(), EspError> {
        self.gap.set_security_conf(&SecurityConfiguration {
            //XXX Is MITM protection needed? Is there a performance cost?
            //XXX After auth has been tested, enable bonding
            auth_req_mode: AuthenticationRequest::SecureOnly,
            // We could use `Display::YesNo` since we have buttons
            // But they are a bit difficult to use, so don't bother
            io_capabilities: IOCapabilities::DisplayOnly,
            initiator_key: Some(KeyMask::EncryptionKey | KeyMask::IdentityResolvingKey),
            responder_key: Some(KeyMask::EncryptionKey | KeyMask::IdentityResolvingKey),
            ..Default::default()
        })
    }

    pub fn set_peer_encryption(&self, addr: BdAddr) -> Result<(), EspError> {
        self.gap.set_encryption(addr, BleEncryption::Encryption)
    }
}


/// Wrapper structure for central implementation
///
/// Provide limited access to GAP.
pub struct BleClient {
    #[allow(dead_code)]
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
}

impl BleClient {
    pub fn start_scanning(&self, duration: u32) -> Result<(), EspError> {
        let scan_params = sys::esp_ble_scan_params_t {
            scan_type: sys::esp_ble_scan_type_t_BLE_SCAN_TYPE_ACTIVE,
            own_addr_type: sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
            scan_filter_policy: sys::esp_ble_scan_filter_t_BLE_SCAN_FILTER_ALLOW_ALL,
            scan_interval: 50,
            scan_window: 48,
            scan_duplicate: sys::esp_ble_scan_duplicate_t_BLE_SCAN_DUPLICATE_ENABLE,
        };

        esp!(unsafe {
            sys::esp_ble_gap_set_scan_params(&scan_params as *const _ as *mut _)
        })?;
        esp!(unsafe { sys::esp_ble_gap_start_scanning(duration) })
    }

    pub fn stop_scanning(&self) -> Result<(), EspError> {
        esp!(unsafe { sys::esp_ble_gap_stop_scanning() })
    }
}


/// Builder to setup BLE
pub struct BleBuilder {
    bt: BtDriver<'static, Ble>,
    handlers: BleGapHandlers,
}

/// Configurable GAP handlers
#[derive(Default)]
struct BleGapHandlers {
    on_scan_result: Option<Box<dyn Fn(BleScanResult) + Send + Sync + 'static>>,
    on_passkey_notification: Option<Box<dyn Fn(BdAddr, u32) + Send + Sync + 'static>>,
}

impl BleBuilder {
    pub fn new(bt: BtDriver<'static, Ble>) -> Self {
        Self {
            bt,
            handlers: Default::default(),
        }
    }

    /// Register a scan result handler
    pub fn with_scanner<F: Fn(BleScanResult) + Send + Sync + 'static>(mut self, f: F) -> Self {
        self.handlers.on_scan_result = Some(Box::new(f));
        self
    }

    /// Register passkey notification handler
    pub fn with_passkey_notifier<F: Fn(BdAddr, u32) + Send + Sync + 'static>(mut self, f: F) -> Self {
        self.handlers.on_passkey_notification = Some(Box::new(f));
        self
    }

    /// Start BLE for peripheral implementation, return a `(server, client)` pair
    ///
    /// The server/client separation is somehow artifical, to keep things separated.
    /// Server is used for peripheral role, client is used for central role.
    pub fn run(self) -> (BleServer, BleClient) {
        let Self { bt, handlers } = self;
        let bt = Arc::new(bt);
        let gap = Arc::new(EspBleGap::new(bt.clone()).unwrap());

        let instance = Arc::new(BleInstance {
            gap: gap.clone(),
            handlers,
        });

        gap.subscribe(move |event| { instance.on_gap_event(event); }).unwrap();

        let server = BleServer {
            gap: gap.clone(),
            gatts: EspGatts::new(bt).unwrap(),
        };
        let client = BleClient {
            gap,
        };

        (server, client)
    }
}


/// Internal structure to implement GAP event handler
struct BleInstance {
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    handlers: BleGapHandlers,
}

impl BleInstance {
    fn on_gap_event(&self, event: BleGapEvent) {
        match event {
            BleGapEvent::AdvertisingConfigured(status) => {
                if status != BtStatus::Success {
                    log::warn!("Unexpected AdvertisingConfigured status {:?}", status);
                } else {
                    if let Err(e) = self.gap.start_advertising() {
                        log::error!("Unable to start advertising: {:?}", e);
                    } else {
                        log::info!("GAP advertising configured and started");
                    }
                }
            }
            BleGapEvent::ScanResult(scan_result) => {
                if let Some(handler) = &self.handlers.on_scan_result {
                    handler(scan_result.into());
                } else {
                    log::warn!("ScanResult received but no handler set, use with_scanner()");
                }
            }
            BleGapEvent::ScanStarted(status) => {
                if status == esp_idf_svc::bt::BtStatus::Success {
                    log::info!("BLE scanning started");
                } else {
                    log::error!("Failed to start BLE scanning: {:?}", status);
                }
            }
            BleGapEvent::ScanStopped(_status) => {
                log::info!("BLE scanning stopped");
            }
            BleGapEvent::PasskeyNotification { addr, passkey } => {
                log::info!("Passkey notification for {addr}: key is {passkey:06}");
                if let Some(handler) = &self.handlers.on_passkey_notification {
                    handler(addr, passkey);
                } else {
                    log::warn!("PasskeyNotification received but no handler set, use with_passkey_notifier()");
                }
            }
            BleGapEvent::AuthenticationComplete { bd_addr, status } => {
                if status == esp_idf_svc::bt::BtStatus::Success {
                    log::info!("Client authenticated: {bd_addr}");
                } else {
                    log::error!("Client authentication failed: {:?}", status);
                }
            }
            _ => {}
        }
    }
}
