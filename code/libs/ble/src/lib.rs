pub mod rome;
pub mod scan;

use std::sync::Arc;
use esp_idf_svc::bt::{
    ble::{
        gap::{AdvConfiguration, BleGapEvent, EspBleGap},
        gatt:: server::EspGatts,
    },
    BdAddr, Ble, BtDriver, BtStatus, BtUuid,
};
use esp_idf_svc::hal::sys::esp;
use esp_idf_svc::sys::{self, EspError, ESP_BLE_ADV_FLAG_GEN_DISC};
use log::{error, info, warn};

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
        self.gap.start_advertising()
    }

    pub fn stop_advertising(&self) -> Result<(), EspError> {
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


/// Start BLE for peripheral implementation
pub fn run_ble(bt: BtDriver<'static, Ble>) -> BleServer {
    fn noop(_: BleScanResult) {}
    run_ble_with_central(bt, noop).0
}

/// Start BLE for peripheral and central implementations
pub fn run_ble_with_central<F: Fn(BleScanResult) + Send + Sync + 'static>(bt: BtDriver<'static, Ble>, scan_handler: F) -> (BleServer, BleClient) {
    let bt = Arc::new(bt);
    let gap = Arc::new(EspBleGap::new(bt.clone()).unwrap());

    let instance = Arc::new(BleInstance {
        gap: gap.clone(),
        scan_handler,
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


/// Internal structure to implement GAP event handler
struct BleInstance<F: Fn(BleScanResult) + Send + Sync + 'static> {
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    scan_handler: F,
}

impl<F: Fn(BleScanResult) + Send + Sync + 'static> BleInstance<F> {
    fn on_gap_event(&self, event: BleGapEvent) {
        match event {
            BleGapEvent::AdvertisingConfigured(status) => {
                if status != BtStatus::Success {
                    warn!("Unexpected AdvertisingConfigured status {:?}", status);
                } else {
                    if let Err(e) = self.gap.start_advertising() {
                        error!("Unable to start advertising: {:?}", e);
                    } else {
                        info!("GAP advertising started");
                    }
                }
            }
            BleGapEvent::ScanResult(scan_result) => {
                let result = scan_result.into();
                (self.scan_handler)(result);
            }
            BleGapEvent::ScanStarted(status) => {
                if status == esp_idf_svc::bt::BtStatus::Success {
                    info!("BLE scanning started");
                } else {
                    error!("Failed to start BLE scanning: {:?}", status);
                }
            }
            BleGapEvent::ScanStopped(_status) => {
                info!("BLE scanning stopped");
            }
            _ => {}
        }
    }
}
