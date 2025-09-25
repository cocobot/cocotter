pub mod game_controller;

use std::{sync::{mpsc::{self, Receiver, Sender}, Arc, Condvar, Mutex}, time::Duration};
use esp_idf_svc::{bt::{ble::{gap::{AdvConfiguration, BleGapEvent, EspBleGap}, gatt::{server::{ConnectionId, EspGatts, GattsEvent, TransferId}, AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface, GattResponse, GattServiceId, GattStatus, Handle, Permission, Property}}, BdAddr, Ble, BtDriver, BtStatus, BtUuid}, sys::{self, EspError, ESP_FAIL}};
use log::{info, warn, error};
use enumset::enum_set;

use crate::game_controller::GameControllerEvent;

const APP_ID: u16 = 0;
const MAX_CONNECTIONS: usize = 1;

pub const SERVICE_ROME_UUID: u128 = 0x81870000ff549699ab4e777ca411f95;
pub const CHAR_ROME_TX_UUID: u128 = 0x81870001ffa549699ab4e777ca411f95;
pub const CHAR_ROME_RX_UUID: u128 = 0x81870002ffa549699ab4e777ca411f95;

// BM769 24G characteristic UUID that sends data via indicate
pub const BM769_DATA_UUID: u128 = 0x91680003111166668888_0123456789ab;

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
    subscribed: bool,

    #[allow(dead_code)]
    mtu: Option<u16>,
}

struct CommState {
    name: String,
    gatt_if: Option<GattInterface>,
    service_handle: Option<Handle>,
    rx_handle: Option<u16>,
    tx_handle: Option<u16>,
    tx_cccd_handle: Option<u16>,
    mtu: Option<u16>,
    connections: Vec<Connection>,
    response: GattResponse,
    rome_tx: Option<Receiver<Box<[u8]>>>,
    tx_confirmed: Option<BdAddr>,
    // Central role state
    scanning: bool,
    remote_ctrl_addr: Option<BdAddr>,
    remote_ctrl_conn_id: Option<ConnectionId>,
    // GATT client state
    gattc_if: Option<u8>,
    gattc_registered: bool,
    bm769_char_handle: Option<u16>,
    current_search_handle: u16,
}

pub struct BleComm {
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    gatts: Arc<EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>>,

    state: Arc<Mutex<CommState>>,

    rome_rx: Sender<Box<[u8]>>,
    condvar: Arc<Condvar>,
}

impl BleComm {
    pub fn run(bt: BtDriver<'static, Ble>, name: String) -> (Sender<Box<[u8]>>, Receiver<Box<[u8]>>, Receiver<GameControllerEvent>) {
        let bt = Arc::new(bt);
        let gap = Arc::new(EspBleGap::new(bt.clone()).unwrap());
        let gatts = Arc::new(EspGatts::new(bt.clone()).unwrap());
        
        let (rome_rx, rome_receiver) = mpsc::channel();
        let (rome_sender, rome_tx) = mpsc::channel();
        let (game_ctrl_event_sender, game_ctrl_event_receiver) = mpsc::channel();

        let state = Arc::new(Mutex::new(CommState {
            name: name,
            gatt_if: None,
            service_handle: None,
            rx_handle: None,
            tx_handle: None,
            tx_cccd_handle: None,
            mtu: None,
            connections: Vec::new(),
            response: GattResponse::default(),
            rome_tx: Some(rome_tx),
            tx_confirmed: None,
            // Central role state
            scanning: false,
            remote_ctrl_addr: None,
            remote_ctrl_conn_id: None,
            // GATT client state
            gattc_if: None,
            gattc_registered: false,
            bm769_char_handle: None,
            current_search_handle: 1,
        }));


        let instance = Arc::new(Self {
            gap,
            gatts,
            // gattc,
            state,
            rome_rx,
            condvar: Arc::new(Condvar::new()),
        });

        let cloned_instance = instance.clone();
        instance.gap.subscribe(move |event| cloned_instance.on_gap_event(event)).unwrap();

        let cloned_instance = instance.clone();
        instance.gatts.subscribe(move |(gatt_if, event)| cloned_instance.on_gatts_event(gatt_if, event)).unwrap();
        instance.gatts.register_app(APP_ID).unwrap();

        // Set global instance for callbacks
        instance.set_global_instance();

        // Register GATT client app for connecting to BM769
        unsafe {
            sys::esp_ble_gattc_register_callback(Some(gattc_callback));
            sys::esp_ble_gattc_app_register(APP_ID + 1);
        }

        if instance.start_scanning().is_err() {
            log::error!("Failed to start scanning");
        }

        let cloned_instance = instance.clone();
        std::thread::spawn(move || {
            let rome_tx = cloned_instance.state.lock().unwrap().rome_tx.take().unwrap();

            loop {
                if let Ok(data) = rome_tx.recv() {
                    for peer_index in 0..MAX_CONNECTIONS {
                         let mut state = cloned_instance.state.lock().unwrap();

                        loop {
                            if state.connections.len() <= peer_index {
                                // We've send to everybody who is connected
                                break;
                            }

                            let Some(gatt_if) = state.gatt_if else {
                                // We lost the gatt interface in the meantime
                                break;
                            };

                            let Some(ind_handle) = state.tx_handle else {
                                // We lost the indication handle in the meantime
                                break;
                            };

                            if state.tx_confirmed.is_none() {
                                let conn = &state.connections[peer_index];

                                if let Err(e) = cloned_instance.gatts
                                    .indicate(gatt_if, conn.conn_id, ind_handle, &data) {
                                    warn!("Indicate err {:?}", e);
                                }                                                                   
                                state.tx_confirmed = Some(conn.peer); //note: we can do it after the indicate as we are locking the state mutex
                         
                                break;
                            } else {
                                state = cloned_instance.condvar.wait(state).unwrap();
                            }
                        }
                    }
                }
                else {
                    log::error!("Rome channel recv error");
                    std::thread::sleep(Duration::from_millis(1000));
                }
            }
        });         

        (rome_sender, rome_receiver, game_ctrl_event_receiver)
    }

    fn on_gap_event(&self, event: BleGapEvent) {

        match event {
            BleGapEvent::AdvertisingConfigured(status) => {
                if status != BtStatus::Success {
                    warn!("Unexpected status {:?}", status);
                }
                else {
                    if let Err(e) = self.gap.start_advertising() {
                        error!("Unable to start adv: {:?}", e);
                    }
                }
            }
            BleGapEvent::ScanResult(scan_result) => {
               // info!("Scan event: {:?}", scan_result);


                // The scan_result is of type esp_ble_gap_cb_param_t_ble_scan_result_evt_param
                // We can access the raw data from it
                unsafe {
                    // Get the BLE address
                    let addr = BdAddr::from_bytes(scan_result.bda);

                    // Check if this is an advertisement with a name
                    if scan_result.adv_data_len > 0 {
                        let adv_data = std::slice::from_raw_parts(
                            scan_result.ble_adv.as_ptr(),
                            scan_result.adv_data_len as usize
                        );

                        // Parse advertising data to find device name
                        if let Some(name) = parse_device_name(adv_data) {
                            if name == "BM769 24G" {
                                // Found the target device, connect to it as a client
                                let mut state = self.state.lock().unwrap();
                                if state.remote_ctrl_addr.is_none() {
                                    state.remote_ctrl_addr = Some(addr);
                                    state.scanning = false;
                                    drop(state);

                                    // Stop scanning
                                    if let Err(e) = self.stop_scanning() {
                                        error!("Failed to stop scanning: {:?}", e);
                                    }

                                    // Connect to the BM769 as a GATT client
                                    self.connect_as_client(addr);
                                }
                            }
                        }
                    }
                }
            }
            BleGapEvent::ScanStarted(status) => {
                if status == BtStatus::Success {
                } else {
                    error!("Failed to start BLE scanning: {:?}", status);
                    self.state.lock().unwrap().scanning = false;
                }
            }
            BleGapEvent::ScanStopped(status) => {
                if status == BtStatus::Success {
                } else {
                    error!("Error stopping BLE scan: {:?}", status);
                }
                self.state.lock().unwrap().scanning = false;
            }
            BleGapEvent::ScanParameterConfigured(status) => {
                if status == BtStatus::Success {
                } else {
                    error!("Failed to configure scan parameters: {:?}", status);
                    self.state.lock().unwrap().scanning = false;
                }
            }
            _ => {}
        }
    }

    fn on_gatts_event(
            &self,
            gatt_if: GattInterface,
            event: GattsEvent,
        ) {

            fn check_gatt_status(status: GattStatus) -> bool {
                match status {
                    GattStatus::Ok => true,
                    _ => {
                        warn!("Unexpected status: {:?}", status);
                        false
                    }
                }
            }
            fn check_result(r : Result<(), EspError>) {
                if let Err(e) = r {
                    error!("Unexpected error: {:?}", e);
                }
            }

            match event {
                GattsEvent::ServiceRegistered { status, app_id } => {
                    if check_gatt_status(status) {
                        if APP_ID == app_id {
                            check_result(self.create_service(gatt_if));
                        }
                    }
                }
                GattsEvent::ServiceCreated {
                    status,
                    service_handle,
                    ..
                } => {
                    if check_gatt_status(status) {
                        check_result(self.configure_and_start_service(service_handle));
                    }
                }
                GattsEvent::CharacteristicAdded {
                    status,
                    attr_handle,
                    service_handle,
                    char_uuid,
                } => {
                    if check_gatt_status(status) {
                        check_result(self.register_characteristic(service_handle, attr_handle, char_uuid));
                    }
                }
                GattsEvent::DescriptorAdded {
                    status,
                    attr_handle,
                    service_handle,
                    descr_uuid,
                } => {
                    if check_gatt_status(status) {
                        self.register_cccd_descriptor(service_handle, attr_handle, descr_uuid);
                    }
                }
                GattsEvent::Mtu { conn_id, mtu } => {
                    self.register_conn_mtu(conn_id, mtu);
                }
                GattsEvent::PeerConnected { conn_id, addr, .. } => {
                    check_result(self.create_conn(conn_id, addr));
                    // Check if this is the BM769 device connecting
                    let mut state = self.state.lock().unwrap();
                    if state.remote_ctrl_addr == Some(addr) {
                        state.remote_ctrl_conn_id = Some(conn_id);
                    }
                }
                GattsEvent::PeerDisconnected { addr, .. } => {
                    check_result(self.delete_conn(addr));
                    // Check if this was the BM769 device
                    let mut state = self.state.lock().unwrap();
                    if state.remote_ctrl_addr == Some(addr) {
                        state.remote_ctrl_addr = None;
                        state.remote_ctrl_conn_id = None;
                        drop(state);
                        // Resume scanning after disconnection
                        let _ = self.start_scanning();
                    }
                }
                GattsEvent::Read { conn_id, trans_id, handle, offset, need_rsp, .. } => {
                    if need_rsp {
                        let mut response = GattResponse::new();
                        response
                            .attr_handle(handle)
                            .auth_req(0)
                            .offset(offset);

                        check_result(self.gatts.send_response(
                            gatt_if,
                            conn_id,
                            trans_id,
                            GattStatus::Ok,
                            Some(&response)
                        ));
                    }
                }
                GattsEvent::Write {
                    conn_id,
                    trans_id,
                    addr,
                    handle,
                    offset,
                    need_rsp,
                    is_prep,
                    value,
                } => {
                    match self.recv(
                        gatt_if, conn_id, trans_id, addr, handle, offset, need_rsp, is_prep, value,
                    ) {
                        Ok(handled) => {                            
                            if handled {
                                check_result(self.send_write_response(
                                    gatt_if, conn_id, trans_id, handle, offset, need_rsp, is_prep, value,
                                ));
                            }
                        }
                        Err(e) => {
                            check_result(Err(e));
                        }
                    }
                }
                GattsEvent::Confirm { status, .. } => {
                    if check_gatt_status(status) {
                        self.confirm_indication();
                    }
                }
                _ => (),
            }
        }

        fn create_service(&self, gatt_if: GattInterface) -> Result<(), EspError> {
            self.state.lock().unwrap().gatt_if = Some(gatt_if);
            let name = self.state.lock().unwrap().name.clone();

            self.gap.set_device_name(&name)?;
            self.gap.set_adv_conf(&AdvConfiguration {
                include_name: true,
                include_txpower: true,
                flag: 2,
                service_uuid: Some(BtUuid::uuid128(SERVICE_ROME_UUID)),
                ..Default::default()
            })?;
            self.gatts.create_service(
                gatt_if,
                &GattServiceId {
                    id: GattId {
                        uuid: BtUuid::uuid128(SERVICE_ROME_UUID),
                        inst_id: 0,
                    },
                    is_primary: true,
                },
                8,
            )?;

            Ok(())
        }

        fn configure_and_start_service(&self, service_handle: Handle) -> Result<(), EspError> {
            self.state.lock().unwrap().service_handle = Some(service_handle);

            self.gatts.start_service(service_handle)?;
            self.add_characteristics(service_handle)?;

            Ok(())
        }

        fn add_characteristics(&self, service_handle: Handle) -> Result<(), EspError> {
            self.gatts.add_characteristic(
                service_handle,
                &GattCharacteristic {
                    uuid: BtUuid::uuid128(CHAR_ROME_RX_UUID),
                    permissions: enum_set!(Permission::Write),
                    properties: enum_set!(Property::Write),
                    max_len: 200,
                    auto_rsp: AutoResponse::ByApp,
                },
                &[],
            )?;

            self.gatts.add_characteristic(
                service_handle,
                &GattCharacteristic {
                    uuid: BtUuid::uuid128(CHAR_ROME_TX_UUID),
                    permissions: enum_set!(),
                    properties: enum_set!(Property::Indicate),
                    max_len: 200,
                    auto_rsp: AutoResponse::ByApp,
                },
                &[],
            )?;

            Ok(())
        }

        fn register_characteristic(
            &self,
            service_handle: Handle,
            attr_handle: Handle,
            char_uuid: BtUuid,
        ) -> Result<(), EspError> {
            let indicate_char = {
                let mut state = self.state.lock().unwrap();

                if state.service_handle != Some(service_handle) {
                    false
                } else if char_uuid == BtUuid::uuid128(CHAR_ROME_RX_UUID) {
                    state.rx_handle = Some(attr_handle);

                    false
                } else if char_uuid == BtUuid::uuid128(CHAR_ROME_TX_UUID) {
                    state.tx_handle = Some(attr_handle);

                    true
                } else {
                    false
                }
            };

            if indicate_char {
                self.gatts.add_descriptor(
                    service_handle,
                    &GattDescriptor {
                        uuid: BtUuid::uuid16(0x2902), // CCCD
                        permissions: enum_set!(Permission::Read | Permission::Write),
                    },
                )?;
            }

            Ok(())
        }

        fn register_cccd_descriptor(
            &self,
            service_handle: Handle,
            attr_handle: Handle,
            descr_uuid: BtUuid,
        ) {
            let mut state = self.state.lock().unwrap();

            if descr_uuid == BtUuid::uuid16(0x2902) // CCCD UUID
                && state.service_handle == Some(service_handle)
            {
                state.tx_cccd_handle = Some(attr_handle);
            }
        }

        fn register_conn_mtu(&self, _conn_id: ConnectionId, mtu: u16) {
            self.state.lock().unwrap().mtu = Some(mtu);
        }

        fn create_conn(&self, conn_id: ConnectionId, addr: BdAddr) -> Result<(), EspError> {
            let added = {
                let mut state = self.state.lock().unwrap();

                if state.connections.len() < MAX_CONNECTIONS {
                    state
                        .connections
                        .push(Connection {
                            peer: addr,
                            conn_id,
                            subscribed: false,
                            mtu: None,
                        });

                    true
                } else {
                    false
                }
            };

            if added {
                self.gap.set_conn_params_conf(addr, 10, 20, 0, 400)?;
            }

            Ok(())
        }

        fn delete_conn(&self, addr: BdAddr) -> Result<(), EspError> {
            let mut state = self.state.lock().unwrap();

            if let Some(index) = state
                .connections
                .iter()
                .position(|Connection { peer, .. }| *peer == addr)
            {
                state.connections.swap_remove(index);
            }

            if state.connections.len() == 0 {
                if let Err(e) = self.gap.start_advertising() {
                    error!("Unable to start adv: {:?}", e);
                }
            }

            Ok(())
        }

        fn confirm_indication(&self) {
            let mut state = self.state.lock().unwrap();
            if state.tx_confirmed.is_none() {
                error!("WTF? ind confirmed but no indication sent")                
            }
            else {
                state.tx_confirmed = None;
                self.condvar.notify_all();
            }
        }

        #[allow(clippy::too_many_arguments)]
        fn recv(
            &self,
            _gatt_if: GattInterface,
            conn_id: ConnectionId,
            _trans_id: TransferId,
            _addr: BdAddr,
            handle: Handle,
            offset: u16,
            _need_rsp: bool,
            _is_prep: bool,
            value: &[u8],
        ) -> Result<bool, EspError> {
            let mut state = self.state.lock().unwrap();

            let recv_handle = state.rx_handle;
            let ind_cccd_handle = state.tx_cccd_handle;

            let Some(conn) = state
                .connections
                .iter_mut()
                .find(|conn| conn.conn_id == conn_id)
            else {
                return Ok(false);
            };

            if Some(handle) == ind_cccd_handle {
                // Subscribe or unsubscribe to our indication characteristic

                if offset == 0 && value.len() == 2 {
                    let value = u16::from_le_bytes([value[0], value[1]]);
                    if value == 0x02 {
                        if !conn.subscribed {
                            conn.subscribed = true;
                        }
                    } else if conn.subscribed {
                        conn.subscribed = false;
                    }
                }
            } else if Some(handle) == recv_handle {
                // Receive data on the recv characteristic
                // Check if this is from the BM769 device
                let state = self.state.lock().unwrap();
                if state.remote_ctrl_addr == Some(_addr) {
                }
                drop(state);

                self.rome_rx.send(Box::from(value)).ok();
                //self.on_recv(addr, value, offset, conn.mtu);
            } else {
                return Ok(false);
            }

            Ok(true)
        }

        #[allow(clippy::too_many_arguments)]
        fn send_write_response(
            &self,
            gatt_if: GattInterface,
            conn_id: ConnectionId,
            trans_id: TransferId,
            handle: Handle,
            offset: u16,
            need_rsp: bool,
            is_prep: bool,
            value: &[u8],
        ) -> Result<(), EspError> {
            if !need_rsp {
                return Ok(());
            }

            if is_prep {
                let mut state = self.state.lock().unwrap();

                state
                    .response
                    .attr_handle(handle)
                    .auth_req(0)
                    .offset(offset)
                    .value(value)
                    .map_err(|_| EspError::from_infallible::<ESP_FAIL>())?;

                self.gatts.send_response(
                    gatt_if,
                    conn_id,
                    trans_id,
                    GattStatus::Ok,
                    Some(&state.response),
                )?;
            } else {
                self.gatts
                    .send_response(gatt_if, conn_id, trans_id, GattStatus::Ok, None)?;
            }

            Ok(())
        }

        pub fn start_scanning(&self) -> Result<(), EspError> {
            let mut state = self.state.lock().unwrap();
            if !state.scanning && state.remote_ctrl_addr.is_none() {
                state.scanning = true;
                drop(state);

                // Configure scan parameters
                let scan_params = sys::esp_ble_scan_params_t {
                    scan_type: sys::esp_ble_scan_type_t_BLE_SCAN_TYPE_ACTIVE,
                    own_addr_type: sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
                    scan_filter_policy: sys::esp_ble_scan_filter_t_BLE_SCAN_FILTER_ALLOW_ALL,
                    scan_interval: 100,  // in 0.625ms units
                    scan_window: 50,     // in 0.625ms units
                    scan_duplicate: sys::esp_ble_scan_duplicate_t_BLE_SCAN_DUPLICATE_ENABLE,
                };

                // Set scan parameters
                let result = unsafe {
                    sys::esp_ble_gap_set_scan_params(&scan_params as *const _ as *mut _)
                };

                if result != sys::ESP_OK {
                    error!("Failed to set scan params: {:?}", result);
                    self.state.lock().unwrap().scanning = false;
                    return Err(EspError::from(result).unwrap());
                }

                // Start scanning (30 seconds duration)
                let result = unsafe {
                    sys::esp_ble_gap_start_scanning(30)
                };

                if result != sys::ESP_OK {
                    error!("Failed to start scan: {:?}", result);
                    self.state.lock().unwrap().scanning = false;
                    return Err(EspError::from(result).unwrap());
                }

            }
            Ok(())
        }

        pub fn stop_scanning(&self) -> Result<(), EspError> {
            let result = unsafe {
                sys::esp_ble_gap_stop_scanning()
            };

            if result == sys::ESP_OK {
                self.state.lock().unwrap().scanning = false;
                Ok(())
            } else {
                Err(EspError::from(result).unwrap())
            }
        }

        fn connect_as_client(&self, addr: BdAddr) {

            // Use ESP-IDF's GAP API to connect
            let result = unsafe {
                let mut bd_addr = sys::esp_bd_addr_t::default();
                // Convert BdAddr to esp_bd_addr_t
                let addr_slice = std::slice::from_raw_parts(
                    &addr as *const BdAddr as *const u8, 6);
                for (i, &byte) in addr_slice.iter().enumerate() {
                    bd_addr[i] = byte;
                }

                // Set connection parameters
                let _conn_params = sys::esp_ble_gap_conn_params_t {
                    interval_min: 0x10,  // 20ms
                    interval_max: 0x20,  // 40ms
                    latency: 0,
                    supervision_timeout: 0x100, // 2.56s
                    scan_interval: 0x10,
                    scan_window: 0x10,
                    min_ce_len: 0,
                    max_ce_len: 0,
                };

                // Use GATTC open to connect
                let state = self.state.lock().unwrap();
                if let Some(gattc_if) = state.gattc_if {
                    drop(state);
                    sys::esp_ble_gattc_open(
                        gattc_if,
                        bd_addr.as_ptr() as *mut _,
                        sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
                        true  // is_direct
                    )
                } else {
                    drop(state);
                    error!("GATT client not registered yet");
                    return; // Exit early
                }
            };

            if result != sys::ESP_OK {
                error!("Failed to initiate GATT client connection: {:?}", result);
                let mut state = self.state.lock().unwrap();
                state.remote_ctrl_addr = None;
                drop(state);
                // Resume scanning on failure
                let _ = self.start_scanning();
            } else {
            }
        }

}

// Static reference to our BleComm instance for callbacks
static mut BLECOMM_INSTANCE: Option<*const BleComm> = None;

impl BleComm {
    fn set_global_instance(&self) {
        unsafe {
            BLECOMM_INSTANCE = Some(self as *const BleComm);
        }
    }

    fn on_gattc_event(&self, event: sys::esp_gattc_cb_event_t, gattc_if: u8, param: *mut sys::esp_ble_gattc_cb_param_t) {
        unsafe {
            if param.is_null() {
                return;
            }

            match event {
                sys::esp_gattc_cb_event_t_ESP_GATTC_REG_EVT => {
                    let mut state = self.state.lock().unwrap();
                    state.gattc_if = Some(gattc_if);
                    state.gattc_registered = true;

                    // If we found BM769 but couldn't connect before, try again now
                    if let Some(addr) = state.remote_ctrl_addr {
                        if state.remote_ctrl_conn_id.is_none() {
                            drop(state);
                            self.connect_as_client(addr);
                        }
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_OPEN_EVT => {
                    let open_param = &(*param).open;
                    if open_param.status == sys::esp_gatt_status_t_ESP_GATT_OK {
                        let mut state = self.state.lock().unwrap();
                        state.remote_ctrl_conn_id = Some(open_param.conn_id);

                        // Start service discovery to find BM769 characteristics
                        if let Some(gattc_if) = state.gattc_if {
                            drop(state);
                            let result = sys::esp_ble_gattc_search_service(gattc_if, open_param.conn_id, std::ptr::null_mut());
                            if result != sys::ESP_OK {
                                error!("Failed to start service discovery: {:?}", result);
                            }
                        }
                    } else {
                        error!("GATT client connection failed, status: {}", open_param.status);
                        let mut state = self.state.lock().unwrap();
                        state.remote_ctrl_addr = None;
                        drop(state);
                        let _ = self.start_scanning();
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_CLOSE_EVT => {
                    let close_param = &(*param).close;
                    let mut state = self.state.lock().unwrap();
                    if state.remote_ctrl_conn_id == Some(close_param.conn_id) {
                        state.remote_ctrl_addr = None;
                        state.remote_ctrl_conn_id = None;
                        drop(state);
                        let _ = self.start_scanning();
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_SEARCH_CMPL_EVT => {
                    let search_param = &(*param).search_cmpl;
                    if search_param.status == sys::esp_gatt_status_t_ESP_GATT_OK {
                        let state = self.state.lock().unwrap();
                        if let (Some(gattc_if), Some(conn_id)) = (state.gattc_if, state.remote_ctrl_conn_id) {
                            drop(state);

                            // Get all characteristics using a different method
                            self.discover_all_characteristics(gattc_if, conn_id);
                        }
                    } else {
                        error!("Service discovery failed: {:?}", search_param.status);
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_NOTIFY_EVT => {
                    let notify_param = &(*param).notify;
                    // Get notification data
                    let data_slice = std::slice::from_raw_parts(notify_param.value, notify_param.value_len as usize);
                    info!("Data (hex): {:02X?}", data_slice);

                    // Forward to rome_rx channel anyway
                    let _ = self.rome_rx.send(Box::from(data_slice));
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_READ_CHAR_EVT => {
                    let read_param = &(*param).read;
                    if read_param.status == sys::esp_gatt_status_t_ESP_GATT_OK && read_param.value_len > 0 {
                        let data_slice = std::slice::from_raw_parts(read_param.value, read_param.value_len as usize);
                        // Check if this looks like meaningful data (not all zeros or all 0xFF)
                        let all_zeros = data_slice.iter().all(|&b| b == 0);
                        let all_ff = data_slice.iter().all(|&b| b == 0xFF);

                        if !all_zeros && !all_ff {
                            // Forward to rome_rx channel
                            let _ = self.rome_rx.send(Box::from(data_slice));
                        }
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_WRITE_DESCR_EVT => {
                    let write_param = &(*param).write;
                    if write_param.status != sys::esp_gatt_status_t_ESP_GATT_OK {
                        error!("Failed to write CCCD descriptor: {:?}", write_param.status);
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_SEARCH_RES_EVT => {
                    // Service discovery result
                    let search_res = &(*param).search_res;
                    // Store service handles for characteristic discovery
                    let mut state = self.state.lock().unwrap();
                    state.current_search_handle = search_res.end_handle;
                    drop(state);
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_REG_FOR_NOTIFY_EVT => {
                    let reg_notify = &(*param).reg_for_notify;
                    if reg_notify.status != sys::esp_gatt_status_t_ESP_GATT_OK {
                        error!("Failed to register for notifications on handle {}: {:?}", reg_notify.handle, reg_notify.status);
                    }
                }
                _ => {
                    // Silently ignore all unhandled events
                }
            }
        }
    }

    fn discover_all_characteristics(&self, gattc_if: u8, conn_id: u16) {

        unsafe {
            // Get all characteristics for the discovered services
            // Use the handle range we stored from service discovery
            let state = self.state.lock().unwrap();
            let end_handle = if state.current_search_handle > 0 {
                state.current_search_handle
            } else {
                0xFFFF
            };
            drop(state);


            // Get all characteristics
            let mut char_elem_result = vec![sys::esp_gattc_char_elem_t::default(); 20];
            let mut char_count = 20u16;

            let result = sys::esp_ble_gattc_get_all_char(
                gattc_if,
                conn_id,
                1,
                end_handle,
                char_elem_result.as_mut_ptr(),
                &mut char_count as *mut _,
                0,
            );

            if result == sys::ESP_OK as u32 {

                // Check each characteristic for BM769 UUID
                let uuid_bytes = BM769_DATA_UUID.to_le_bytes();

                for i in 0..char_count as usize {
                    let char_elem = &char_elem_result[i];

                    // Check if this is our BM769 UUID
                    if char_elem.uuid.len == 16 {
                        let mut matches = true;
                        for j in 0..16 {
                            if char_elem.uuid.uuid.uuid128[j] != uuid_bytes[j] {
                                matches = false;
                                break;
                            }
                        }

                        if matches {
                            let mut state = self.state.lock().unwrap();
                            state.bm769_char_handle = Some(char_elem.char_handle);
                            drop(state);

                            self.subscribe_to_characteristic(gattc_if, conn_id, char_elem.char_handle);
                            return;
                        }
                    }

                }

                self.try_specific_handles(gattc_if, conn_id);
            } else {
                error!("Failed to get characteristics: {:?}, trying fallback", result);
                self.try_specific_handles(gattc_if, conn_id);
            }
        }
    }

    fn try_specific_handles(&self, gattc_if: u8, conn_id: u16) {

        // Only test a few likely handles
        let test_handles = [10, 12, 14, 16, 20, 24, 30, 35, 40, 45, 50];

        for handle in test_handles {
            unsafe {
                // Register for notifications
                let addr = self.state.lock().unwrap().remote_ctrl_addr.unwrap();
                // Convert BdAddr to bytes using unsafe
                let addr_ptr = &addr as *const BdAddr as *const [u8; 6];
                let result = sys::esp_ble_gattc_register_for_notify(
                    gattc_if,
                    (*addr_ptr).as_ptr() as *mut _,
                    handle,
                );

                if result == sys::ESP_OK {

                    // Try to enable notifications via CCCD
                    let notify_value: [u8; 2] = [0x01, 0x00];
                    for offset in [1, 2] {
                        let cccd_handle = handle + offset;
                        let write_result = sys::esp_ble_gattc_write_char_descr(
                            gattc_if,
                            conn_id,
                            cccd_handle,
                            2,
                            notify_value.as_ptr() as *mut u8,
                            sys::esp_gatt_write_type_t_ESP_GATT_WRITE_TYPE_RSP,
                            sys::esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_NONE,
                        );

                        if write_result == sys::ESP_OK {
                            // Save this as potential BM769 handle
                            let mut state = self.state.lock().unwrap();
                            if state.bm769_char_handle.is_none() {
                                state.bm769_char_handle = Some(handle);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    fn subscribe_to_characteristic(&self, gattc_if: u8, conn_id: u16, char_handle: u16) {

        unsafe {
            // First register for notifications
            let addr = self.state.lock().unwrap().remote_ctrl_addr.unwrap();
            // Convert BdAddr to bytes using unsafe
            let addr_ptr = &addr as *const BdAddr as *const [u8; 6];
            let result = sys::esp_ble_gattc_register_for_notify(
                gattc_if,
                (*addr_ptr).as_ptr() as *mut _,
                char_handle,
            );

            if result == sys::ESP_OK {

                // Then enable notifications via CCCD
                let notify_value: [u8; 2] = [0x01, 0x00]; // Enable notifications

                // Try common CCCD offsets
                for offset in [1, 2] {
                    let cccd_handle = char_handle + offset;
                    let write_result = sys::esp_ble_gattc_write_char_descr(
                        gattc_if,
                        conn_id,
                        cccd_handle,
                        2,
                        notify_value.as_ptr() as *mut u8,
                        sys::esp_gatt_write_type_t_ESP_GATT_WRITE_TYPE_RSP,
                        sys::esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_NONE,
                    );

                    if write_result == sys::ESP_OK {
                        break;
                    }
                }
            } else {
                error!("Failed to register for notifications: {:?}", result);
            }
        }
    }

}

// GATT client callback function
unsafe extern "C" fn gattc_callback(
    event: sys::esp_gattc_cb_event_t,
    gattc_if: sys::esp_gatt_if_t,
    param: *mut sys::esp_ble_gattc_cb_param_t,
) {
    if let Some(instance_ptr) = BLECOMM_INSTANCE {
        let instance = &*instance_ptr;
        instance.on_gattc_event(event, gattc_if, param);
    }
}

// Helper function to parse device name from advertising data
fn parse_device_name(adv_data: &[u8]) -> Option<String> {
    let mut i = 0;
    while i < adv_data.len() {
        if i + 1 >= adv_data.len() {
            break;
        }

        let len = adv_data[i] as usize;
        if len == 0 || i + len >= adv_data.len() {
            break;
        }

        let ad_type = adv_data[i + 1];

        // AD Type 0x09 is Complete Local Name
        // AD Type 0x08 is Shortened Local Name
        if ad_type == 0x09 || ad_type == 0x08 {
            if len > 1 {
                let name_bytes = &adv_data[i + 2..i + 1 + len];
                if let Ok(name) = std::str::from_utf8(name_bytes) {
                    return Some(name.to_string());
                }
            }
        }

        i += 1 + len;
    }
    None
}