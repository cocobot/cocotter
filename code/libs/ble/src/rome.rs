use std::sync::{mpsc::{Receiver, Sender}, Arc, Condvar, Mutex};
use std::time::Duration;
use esp_idf_svc::{
    bt::{
        ble::{
            gap::{AdvConfiguration, BleGapEvent, EspBleGap},
            gatt::{
                server::{ConnectionId, EspGatts, GattsEvent, TransferId},
                AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface,
                GattResponse, GattServiceId, GattStatus, Handle, Permission, Property,
            },
        },
        BdAddr, Ble, BtDriver, BtStatus, BtUuid,
    },
    sys::{EspError, ESP_FAIL},
};
use enumset::enum_set;
use log::{error, warn};

// ROME Protocol UUIDs
pub const SERVICE_ROME_UUID: u128 = 0x81870000ff549699ab4e777ca411f95;
pub const CHAR_ROME_TX_UUID: u128 = 0x81870001ffa549699ab4e777ca411f95;
pub const CHAR_ROME_RX_UUID: u128 = 0x81870002ffa549699ab4e777ca411f95;

const APP_ID: u16 = 0;
const MAX_CONNECTIONS: usize = 1;

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
    subscribed: bool,
    #[allow(dead_code)]
    mtu: Option<u16>,
}

pub struct RomePeripheral {
    gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    gatts: Arc<EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    state: Arc<Mutex<PeripheralState>>,
    rome_rx: Sender<Box<[u8]>>,
    condvar: Arc<Condvar>,
}

struct PeripheralState {
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
}

impl RomePeripheral {
    pub fn new(
        gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
        gatts: Arc<EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
        name: String,
        rome_rx: Sender<Box<[u8]>>,
        rome_tx: Receiver<Box<[u8]>>,
    ) -> Arc<Self> {
        let state = Arc::new(Mutex::new(PeripheralState {
            name,
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
        }));

        Arc::new(Self {
            gap,
            gatts,
            state,
            rome_rx,
            condvar: Arc::new(Condvar::new()),
        })
    }

    pub fn start(self: Arc<Self>) {
        // Register GATT server callbacks
        let cloned = self.clone();
        self.gatts
            .subscribe(move |(gatt_if, event)| cloned.on_gatts_event(gatt_if, event))
            .unwrap();

        self.gatts.register_app(APP_ID).unwrap();

        // Start the indication sender thread
        self.start_indication_sender();
    }

    fn start_indication_sender(self: &Arc<Self>) {
        let cloned = self.clone();
        std::thread::spawn(move || {
            let rome_tx = cloned.state.lock().unwrap().rome_tx.take().unwrap();

            loop {
                if let Ok(data) = rome_tx.recv() {
                    for peer_index in 0..MAX_CONNECTIONS {
                        let mut state = cloned.state.lock().unwrap();

                        loop {
                            if state.connections.len() <= peer_index {
                                break;
                            }

                            let Some(gatt_if) = state.gatt_if else {
                                break;
                            };

                            let Some(ind_handle) = state.tx_handle else {
                                break;
                            };

                            if state.tx_confirmed.is_none() {
                                let conn = &state.connections[peer_index];

                                if let Err(e) = cloned.gatts.indicate(
                                    gatt_if,
                                    conn.conn_id,
                                    ind_handle,
                                    &data,
                                ) {
                                    warn!("Indicate err {:?}", e);
                                }
                                state.tx_confirmed = Some(conn.peer);
                                break;
                            } else {
                                state = cloned.condvar.wait(state).unwrap();
                            }
                        }
                    }
                } else {
                    error!("Rome channel recv error");
                    std::thread::sleep(Duration::from_millis(1000));
                }
            }
        });
    }

    pub fn on_gap_event(&self, event: &BleGapEvent) {
        match event {
            BleGapEvent::AdvertisingConfigured(status) => {
                if *status != BtStatus::Success {
                    warn!("Unexpected status {:?}", status);
                } else {
                    if let Err(e) = self.gap.start_advertising() {
                        error!("Unable to start adv: {:?}", e);
                    }
                }
            }
            _ => {}
        }
    }

    fn on_gatts_event(&self, gatt_if: GattInterface, event: GattsEvent) {
        fn check_gatt_status(status: GattStatus) -> bool {
            match status {
                GattStatus::Ok => true,
                _ => {
                    warn!("Unexpected status: {:?}", status);
                    false
                }
            }
        }
        fn check_result(r: Result<(), EspError>) {
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
                    check_result(self.register_characteristic(
                        service_handle,
                        attr_handle,
                        char_uuid,
                    ));
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
            }
            GattsEvent::PeerDisconnected { addr, .. } => {
                check_result(self.delete_conn(addr));
            }
            GattsEvent::Read {
                conn_id,
                trans_id,
                handle,
                offset,
                need_rsp,
                ..
            } => {
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
                        Some(&response),
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

        if descr_uuid == BtUuid::uuid16(0x2902)
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
                state.connections.push(Connection {
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
        } else {
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
            drop(state);
            self.rome_rx.send(Box::from(value)).ok();
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
}