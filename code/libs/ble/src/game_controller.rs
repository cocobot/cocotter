use std::sync::{mpsc::Sender, Arc, Mutex};
use esp_idf_svc::{
    bt::{
        ble::gap::BleGapEvent,
        BdAddr,
    },
    sys::{self, EspError},
};
use log::error;

// BM769 24G characteristic UUID that sends data via indicate
pub const BM769_DATA_UUID: u128 = 0x91680003111166668888_0123456789ab;

macro_rules! check_button {
    ($byte:expr, $data:expr, $byte_idx:expr, $mask:expr, $pressed_event:expr, $released_event:expr, $self:expr) => {
        if (*$byte & $mask) != 0 {
            if ($data[$byte_idx] & $mask) != 0 {
                let _ = $self.event_sender.send($pressed_event);
            } else {
                let _ = $self.event_sender.send($released_event);
            }
            *$byte &= !$mask;
        }
    };
}

macro_rules! read_analog_u8 {
    ($byte:expr, $data:expr, $byte_idx:expr, $event_ctor:expr, $self:expr) => {
        if *$byte != 0 {
            let value = $data[$byte_idx];
            let _ = $self.event_sender.send($event_ctor(value));
            *$byte = 0;
        }
    };
}

macro_rules! read_analog_i8 {
    ($byte:expr, $data:expr, $byte_idx:expr, $event_ctor:expr, $self:expr) => {
        if *$byte != 0 {
            let value = $data[$byte_idx];
            let value = value as isize;
            let value = (value - 128) as i8;
            let _ = $self.event_sender.send($event_ctor(value));
            *$byte = 0;
        }
    };
}

#[derive(Debug)]
pub enum GameControllerEvent {
    Connected,
    Disconnected,

    ButtonUpPressed,
    ButtonUpReleased,
    ButtonDownPressed,
    ButtonDownReleased,
    ButtonLeftPressed,
    ButtonLeftReleased,
    ButtonRightPressed,
    ButtonRightReleased,

    ButtonLeftJoyPressed,
    ButtonLeftJoyReleased,
    ButtonLeftJoyLeftRightValue(i8),
    ButtonLeftJoyUpDownValue(i8),

    ButtonRightJoyPressed,
    ButtonRightJoyReleased,
    ButtonRightJoyLeftRightValue(i8),
    ButtonRightJoyUpDownValue(i8),

    ButtonXPressed,
    ButtonXReleased,
    ButtonOPressed, 
    ButtonOReleased,
    ButtonSquarePressed,
    ButtonSquareReleased,
    ButtonTrianglePressed,
    ButtonTriangleReleased,

    ButtonL1Pressed,
    ButtonL1Released,
    ButtonL2Value(u8),
    ButtonR1Pressed,
    ButtonR1Released,
    ButtonR2Value(u8),

    ButtonStartPressed,
    ButtonStartReleased,
    ButtonSharePressed,
    ButtonShareReleased,
    ButtonOptionsPressed,
    ButtonOptionsReleased,
}

pub struct GameControllerCentral {
    state: Arc<Mutex<CentralState>>,
    event_sender: Sender<GameControllerEvent>,
}

struct CentralState {
    // Scanning state
    scanning: bool,

    // BM769 device state
    remote_ctrl_addr: Option<BdAddr>,
    remote_ctrl_conn_id: Option<u16>,

    // GATT client state
    gattc_if: Option<u8>,
    gattc_registered: bool,
    bm769_char_handle: Option<u16>,
    current_search_handle: u16,

    // Controller state tracking
    last_controller_data: Vec<u8>,
}

impl GameControllerCentral {
    pub fn new(
        event_sender: Sender<GameControllerEvent>
    ) -> Arc<Self> {
        let state = Arc::new(Mutex::new(CentralState {
            scanning: false,
            remote_ctrl_addr: None,
            remote_ctrl_conn_id: None,
            gattc_if: None,
            gattc_registered: false,
            bm769_char_handle: None,
            current_search_handle: 1,
            last_controller_data: Vec::new(),
        }));

        Arc::new(Self {
            state,
            event_sender,
        })
    }

    pub fn start(self: Arc<Self>) {
        // Set global instance for C callbacks
        self.set_global_instance();

        // Register GATT client
        unsafe {
            sys::esp_ble_gattc_register_callback(Some(gattc_callback));
            sys::esp_ble_gattc_app_register(1); // Use app_id 1 for client
        }

        // Start scanning
        if self.start_scanning().is_err() {
            error!("Failed to start scanning");
        }
    }

    pub fn on_gap_event(&self, event: &BleGapEvent) {
        match event {
            BleGapEvent::ScanResult(scan_result) => {
                unsafe {
                    let addr = BdAddr::from_bytes(scan_result.bda);

                    if scan_result.adv_data_len > 0 {
                        let adv_data = std::slice::from_raw_parts(
                            scan_result.ble_adv.as_ptr(),
                            scan_result.adv_data_len as usize,
                        );

                        if let Some(name) = parse_device_name(adv_data) {
                            if name == "BM769 24G" {
                                let mut state = self.state.lock().unwrap();
                                if state.remote_ctrl_addr.is_none() {
                                    state.remote_ctrl_addr = Some(addr);
                                    state.scanning = false;
                                    drop(state);

                                    // Stop scanning and connect
                                    if let Err(e) = self.stop_scanning() {
                                        error!("Failed to stop scanning: {:?}", e);
                                    }
                                    self.connect_as_client(addr);
                                }
                            }
                        }
                    }
                }
            }
            BleGapEvent::ScanStarted(status) => {
                if *status == esp_idf_svc::bt::BtStatus::Success {
                    self.state.lock().unwrap().scanning = true;
                } else {
                    error!("Failed to start BLE scanning: {:?}", status);
                    self.state.lock().unwrap().scanning = false;
                }
            }
            BleGapEvent::ScanStopped(_status) => {
                log::info!("BLE scanning stopped");
                self.state.lock().unwrap().scanning = false;
            }
            _ => {}
        }
    }

    pub fn on_gattc_event(
        &self,
        event: sys::esp_gattc_cb_event_t,
        gattc_if: u8,
        param: *mut sys::esp_ble_gattc_cb_param_t,
    ) {
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

                        if let Some(gattc_if) = state.gattc_if {
                            drop(state);
                            // Start service discovery
                            let result = sys::esp_ble_gattc_search_service(
                                gattc_if,
                                open_param.conn_id,
                                std::ptr::null_mut(),
                            );
                            if result != sys::ESP_OK {
                                error!("Failed to start service discovery: {:?}", result);
                            }
                        }

                        // Send connected event
                        let _ = self.event_sender.send(GameControllerEvent::Connected);
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
                        state.bm769_char_handle = None;
                        drop(state);

                        // Send disconnected event
                        let _ = self.event_sender.send(GameControllerEvent::Disconnected);

                        // Resume scanning
                        let _ = self.start_scanning();
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
                sys::esp_gattc_cb_event_t_ESP_GATTC_SEARCH_CMPL_EVT => {
                    let search_param = &(*param).search_cmpl;
                    if search_param.status == sys::esp_gatt_status_t_ESP_GATT_OK {
                        let state = self.state.lock().unwrap();
                        if let (Some(gattc_if), Some(conn_id)) =
                            (state.gattc_if, state.remote_ctrl_conn_id)
                        {
                            drop(state);
                            self.discover_all_characteristics(gattc_if, conn_id);
                        }
                    } else {
                        error!("Service discovery failed: {:?}", search_param.status);
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_NOTIFY_EVT => {
                    let notify_param = &(*param).notify;
                    // Get notification data
                    let data_slice = std::slice::from_raw_parts(
                        notify_param.value,
                        notify_param.value_len as usize,
                    );

                    // Parse controller data
                    self.parse_controller_data(data_slice);
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_REG_FOR_NOTIFY_EVT => {
                    let reg_notify = &(*param).reg_for_notify;
                    if reg_notify.status != sys::esp_gatt_status_t_ESP_GATT_OK {
                        error!(
                            "Failed to register for notifications on handle {}: {:?}",
                            reg_notify.handle, reg_notify.status
                        );
                    }
                }
                sys::esp_gattc_cb_event_t_ESP_GATTC_WRITE_DESCR_EVT => {
                    let write_param = &(*param).write;
                    if write_param.status != sys::esp_gatt_status_t_ESP_GATT_OK {
                        error!("Failed to write CCCD descriptor: {:?}", write_param.status);
                    }
                }
                _ => {
                    // Silently ignore other events
                }
            }
        }
    }

    fn parse_controller_data(&self, data: &[u8]) {
        let mut state = self.state.lock().unwrap();

        // Compare with previous data and show what changed
        if data.len() != state.last_controller_data.len() || data != &state.last_controller_data {
            let mut xor_data = state
                .last_controller_data
                .iter()
                .zip(data.iter())
                .map(|(a, b)| a ^ b)
                .collect::<Vec<u8>>();

            for (i, byte) in xor_data.iter_mut().enumerate() {
                if *byte != 0 {
                    // Check specific buttons
                    match i {
                        3 => {
                            read_analog_i8!(byte, data, 3, GameControllerEvent::ButtonLeftJoyLeftRightValue, self);
                        }
                        4 => {
                            read_analog_i8!(byte, data, 4, GameControllerEvent::ButtonLeftJoyUpDownValue, self);
                        }
                        5 => {
                            read_analog_i8!(byte, data, 5, GameControllerEvent::ButtonRightJoyLeftRightValue, self);
                        }
                        6 => {
                            read_analog_i8!(byte, data, 6, GameControllerEvent::ButtonRightJoyUpDownValue, self);
                        }
                        7 => {
                            check_button!(byte, data, 7, 0x01, GameControllerEvent::ButtonUpPressed, GameControllerEvent::ButtonUpReleased, self);
                            check_button!(byte, data, 7, 0x02, GameControllerEvent::ButtonDownPressed, GameControllerEvent::ButtonDownReleased, self);
                            check_button!(byte, data, 7, 0x04, GameControllerEvent::ButtonLeftPressed, GameControllerEvent::ButtonLeftReleased, self);
                            check_button!(byte, data, 7, 0x08, GameControllerEvent::ButtonRightPressed, GameControllerEvent::ButtonRightReleased, self);
                            check_button!(byte, data, 7, 0x10, GameControllerEvent::ButtonXPressed, GameControllerEvent::ButtonXReleased, self);
                            check_button!(byte, data, 7, 0x20, GameControllerEvent::ButtonOPressed, GameControllerEvent::ButtonOReleased, self);
                            check_button!(byte, data, 7, 0x40, GameControllerEvent::ButtonSquarePressed, GameControllerEvent::ButtonSquareReleased, self);
                            check_button!(byte, data, 7, 0x80, GameControllerEvent::ButtonTrianglePressed, GameControllerEvent::ButtonTriangleReleased, self);
                        }
                        8 => {
                            check_button!(byte, data, 8, 0x01, GameControllerEvent::ButtonL1Pressed, GameControllerEvent::ButtonL1Released, self);
                            check_button!(byte, data, 8, 0x02, GameControllerEvent::ButtonR1Pressed, GameControllerEvent::ButtonR1Released, self);
                            check_button!(byte, data, 8, 0x04, GameControllerEvent::ButtonLeftJoyPressed, GameControllerEvent::ButtonLeftJoyReleased, self);
                            check_button!(byte, data, 8, 0x08, GameControllerEvent::ButtonRightJoyPressed, GameControllerEvent::ButtonRightJoyReleased, self);
                            check_button!(byte, data, 8, 0x10, GameControllerEvent::ButtonSharePressed, GameControllerEvent::ButtonShareReleased, self);
                            check_button!(byte, data, 8, 0x20, GameControllerEvent::ButtonOptionsPressed, GameControllerEvent::ButtonOptionsReleased, self);
                            check_button!(byte, data, 8, 0x40, GameControllerEvent::ButtonStartPressed, GameControllerEvent::ButtonStartReleased, self);
                        }
                        10 => {
                            read_analog_u8!(byte, data, 10, GameControllerEvent::ButtonL2Value, self);
                        }
                        11 => {
                            read_analog_u8!(byte, data, 11, GameControllerEvent::ButtonR2Value, self);
                        }
                        _ => {}
                    }
                }
            }

            for (i, &byte) in xor_data.iter().enumerate() {
                if byte != 0 {
                    println!("Unhandled byte {} changed: 0x{:02X} -> 0x{:02X}", i, state.last_controller_data.get(i).cloned().unwrap_or(0), data[i]);
                }
            }           
        }

        // Store new data
        state.last_controller_data = data.to_vec();
}

    fn start_scanning(&self) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();
        if !state.scanning && state.remote_ctrl_addr.is_none() {
            state.scanning = true;
            drop(state);

            // Configure scan parameters
            let scan_params = sys::esp_ble_scan_params_t {
                scan_type: sys::esp_ble_scan_type_t_BLE_SCAN_TYPE_ACTIVE,
                own_addr_type: sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
                scan_filter_policy: sys::esp_ble_scan_filter_t_BLE_SCAN_FILTER_ALLOW_ALL,
                scan_interval: 100,
                scan_window: 50,
                scan_duplicate: sys::esp_ble_scan_duplicate_t_BLE_SCAN_DUPLICATE_ENABLE,
            };

            unsafe {
                let result =
                    sys::esp_ble_gap_set_scan_params(&scan_params as *const _ as *mut _);
                if result != sys::ESP_OK {
                    self.state.lock().unwrap().scanning = false;
                    return Err(EspError::from(result).unwrap());
                }

                let result = sys::esp_ble_gap_start_scanning(30);
                if result != sys::ESP_OK {
                    self.state.lock().unwrap().scanning = false;
                    return Err(EspError::from(result).unwrap());
                }
            }
        }
        Ok(())
    }

    fn stop_scanning(&self) -> Result<(), EspError> {
        unsafe {
            let result = sys::esp_ble_gap_stop_scanning();
            if result == sys::ESP_OK {
                self.state.lock().unwrap().scanning = false;
                Ok(())
            } else {
                Err(EspError::from(result).unwrap())
            }
        }
    }

    fn connect_as_client(&self, addr: BdAddr) {
        unsafe {
            let mut bd_addr = sys::esp_bd_addr_t::default();
            // Convert BdAddr to esp_bd_addr_t
            let addr_slice =
                std::slice::from_raw_parts(&addr as *const BdAddr as *const u8, 6);
            for (i, &byte) in addr_slice.iter().enumerate() {
                bd_addr[i] = byte;
            }

            let state = self.state.lock().unwrap();
            if let Some(gattc_if) = state.gattc_if {
                drop(state);
                let result = sys::esp_ble_gattc_open(
                    gattc_if,
                    bd_addr.as_ptr() as *mut _,
                    sys::esp_ble_addr_type_t_BLE_ADDR_TYPE_PUBLIC,
                    true, // is_direct
                );
                if result != sys::ESP_OK {
                    error!("Failed to initiate GATT client connection: {:?}", result);
                    let mut state = self.state.lock().unwrap();
                    state.remote_ctrl_addr = None;
                    drop(state);
                    let _ = self.start_scanning();
                }
            } else {
                drop(state);
                error!("GATT client not registered yet");
            }
        }
    }

    fn discover_all_characteristics(&self, gattc_if: u8, conn_id: u16) {
        unsafe {
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

                // Fallback: try specific handles
                self.try_specific_handles(gattc_if, conn_id);
            } else {
                error!("Failed to get characteristics: {:?}", result);
                self.try_specific_handles(gattc_if, conn_id);
            }
        }
    }

    fn try_specific_handles(&self, gattc_if: u8, conn_id: u16) {
        let test_handles = [10, 12, 14, 16, 20, 24, 30, 35, 40, 45, 50];

        for handle in test_handles {
            unsafe {
                let addr = self.state.lock().unwrap().remote_ctrl_addr.unwrap();
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
            let addr = self.state.lock().unwrap().remote_ctrl_addr.unwrap();
            let addr_ptr = &addr as *const BdAddr as *const [u8; 6];
            let result = sys::esp_ble_gattc_register_for_notify(
                gattc_if,
                (*addr_ptr).as_ptr() as *mut _,
                char_handle,
            );

            if result == sys::ESP_OK {
                // Enable notifications via CCCD
                let notify_value: [u8; 2] = [0x01, 0x00];
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

    fn set_global_instance(&self) {
        unsafe {
            GAME_CONTROLLER_INSTANCE = Some(self as *const GameControllerCentral);
        }
    }
}

// Static reference for C callbacks
static mut GAME_CONTROLLER_INSTANCE: Option<*const GameControllerCentral> = None;

// GATT client callback function
unsafe extern "C" fn gattc_callback(
    event: sys::esp_gattc_cb_event_t,
    gattc_if: sys::esp_gatt_if_t,
    param: *mut sys::esp_ble_gattc_cb_param_t,
) {
    if let Some(instance_ptr) = GAME_CONTROLLER_INSTANCE {
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