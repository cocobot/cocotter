use std::sync::{Arc, Condvar, Mutex};
use std::sync::mpsc::{self, Receiver, Sender};
use std::time::Duration;
use esp_idf_svc::{
    bt::{
        ble::{
            gatt::{
                server::{ConnectionId, GattsEvent},
                AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface,
                GattResponse, GattServiceId, GattStatus, Handle, Permission, Property,
            },
        },
        BdAddr, BtUuid,
    },
};
use esp_idf_svc::sys::{EspError, ESP_FAIL};
use enumset::enum_set;
use log::{debug, error, info, warn};
use crate::BleServer;

const SERVICE_ROME_UUID: BtUuid = BtUuid::uuid128(0x8187_0000_ffa549699ab4e777ca411f95);
const CHAR_ROME_TX_UUID: BtUuid = BtUuid::uuid128(0x8187_0001_ffa549699ab4e777ca411f95);
const CHAR_ROME_RX_UUID: BtUuid = BtUuid::uuid128(0x8187_0002_ffa549699ab4e777ca411f95);

const CCCD_UUID: BtUuid = BtUuid::uuid16(0x2902);

const APP_ID: u16 = 0;
const MAX_CONNECTIONS: usize = 1;

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
}

struct PeripheralState {
    name: String,
    gatt_if: Option<GattInterface>,
    service_handle: Option<Handle>,
    rx_handle: Option<u16>,
    tx_handle: Option<u16>,
    tx_cccd_handle: Option<u16>,
    connections: Vec<Connection>,
    tx_confirmed: Option<BdAddr>,
}

pub struct RomePeripheral {
    server: BleServer,
    state: Arc<Mutex<PeripheralState>>,
    rome_rx: Sender<Box<[u8]>>,
    condvar: Arc<Condvar>,
}

impl RomePeripheral {
    pub fn run(server: BleServer, name: String) -> (Sender<Box<[u8]>>, Receiver<Box<[u8]>>) {
        let (rome_rx, rome_receiver) = mpsc::channel();
        let (rome_sender, rome_tx) = mpsc::channel::<Box<[u8]>>();

        let state = Arc::new(Mutex::new(PeripheralState {
            name,
            gatt_if: None,
            service_handle: None,
            rx_handle: None,
            tx_handle: None,
            tx_cccd_handle: None,
            connections: Vec::new(),
            tx_confirmed: None,
        }));

        let instance = Arc::new(Self {
            server,
            state,
            rome_rx,
            condvar: Arc::new(Condvar::new()),
        });

        // Register GATT server callbacks
        let cloned_instance = instance.clone();
        instance.server.gatts.subscribe(move |(gatt_if, event)| {
            if let Err(e) = cloned_instance.on_gatts_event(gatt_if, event) {
                warn!("GATTS event error: {e:?}");
            }
        }).unwrap();

        instance.server.gatts.register_app(APP_ID).unwrap();

        // Enable secure connections
        instance.server.enable_security().unwrap();

        log::info!("Spawning ROME peripheral thread");
        std::thread::spawn(move || {
            loop {
                if let Ok(data) = rome_tx.recv() {
                    //TODO Use notifications for telemetry and indications for order ACKs
                    for peer_index in 0..MAX_CONNECTIONS {
                        let mut state = instance.state.lock().unwrap();

                        // Wait confirmation of previous indication
                        state = instance.condvar.wait_while(state, |state| {
                            state.tx_confirmed.is_some()
                        }).unwrap();

                        let Some(conn) = state.connections.get(peer_index) else { continue; };
                        let Some(gatt_if) = state.gatt_if else { continue; };
                        let Some(tx_handle) = state.tx_handle else { continue; };

                        if let Err(e) = instance.server.gatts.indicate(gatt_if, conn.conn_id, tx_handle, &data) {
                            warn!("GATT server indicate error: {e:?}");
                        }                                                                   
                        state.tx_confirmed = Some(conn.peer);
                    }
                } else {
                    warn!("No data recv on rome TX");
                    std::thread::sleep(Duration::from_millis(1000));
                }
            }
        });

        (rome_sender, rome_receiver)
    }

    fn on_gatts_event(&self, gatt_if: GattInterface, event: GattsEvent) -> Result<(), EspError> {
        debug!("Gatts event: {event:?}");

        fn check_gatt_status(status: GattStatus) -> Result<(), EspError> {
            if status != GattStatus::Ok {
                // Bad status is logged right away, but another error will be logged above
                warn!("GATTS unexpected status: {status:?}");
                Err(EspError::from_infallible::<ESP_FAIL>())
            } else {
                Ok(())
            }
        }


        match event {
            GattsEvent::ServiceRegistered { status, app_id } => {
                check_gatt_status(status)?;
                if app_id == APP_ID {
                    self.create_service(gatt_if)?;
                }
            }
            GattsEvent::ServiceCreated { status, service_handle, .. } => {
                check_gatt_status(status)?;
                self.configure_and_start_service(service_handle)?;
            }
            GattsEvent::CharacteristicAdded { status, attr_handle, service_handle, char_uuid, } => {
                check_gatt_status(status)?;
                self.register_characteristic(service_handle, attr_handle, char_uuid)?;
            }
            GattsEvent::DescriptorAdded { status, attr_handle, service_handle, descr_uuid } => {
                check_gatt_status(status)?;
                self.register_descriptor(service_handle, attr_handle, descr_uuid)?;
            }
            GattsEvent::Mtu { conn_id, mtu } => {
                info!("Connection {conn_id} requested MTU {mtu}");
            }
            GattsEvent::PeerConnected { conn_id, addr, .. } => {
                info!("Peer connected ({conn_id}): {addr}");
                self.create_conn(conn_id, addr)?;
            }
            GattsEvent::PeerDisconnected { conn_id, addr, .. } => {
                info!("Peer disconnected ({conn_id}): {addr}");
                self.delete_conn(conn_id)?;
            }
            GattsEvent::Read { conn_id, trans_id, addr, handle, offset, need_rsp, .. } => {
                debug!("Read data from conn {conn_id}, addr {addr}, need_rsp: {need_rsp:?}");
                if need_rsp {
                    let mut response = GattResponse::new();
                    response
                        .attr_handle(handle)
                        .auth_req(0)
                        .offset(offset);

                    self.server.gatts.send_response(
                        gatt_if,
                        conn_id,
                        trans_id,
                        GattStatus::Ok,
                        Some(&response)
                    )?;
                }
            }
            GattsEvent::Write { conn_id, trans_id, handle, offset, need_rsp, is_prep, value, .. } => {
                if self.on_write_data(handle, value) {
                    if need_rsp {
                        let response = if is_prep {
                            let mut response = GattResponse::new();
                            response
                                .attr_handle(handle)
                                .auth_req(0)
                                .offset(offset)
                                .value(value)
                                .map_err(|_| EspError::from_infallible::<ESP_FAIL>())?;
                            Some(response)
                        } else {
                            None
                        };
                        self.server.gatts.send_response(gatt_if, conn_id, trans_id, GattStatus::Ok, (&response).into())?;
                    }
                }
            }
            GattsEvent::Confirm { status, .. } => {
                check_gatt_status(status)?;
                self.confirm_indication();
            }
            _ => (),
        }

        Ok(())
    }

    /// Called after server app creation, create the GATT server service
    fn create_service(&self, gatt_if: GattInterface) -> Result<(), EspError> {
        self.state.lock().unwrap().gatt_if = Some(gatt_if);
        let name = &self.state.lock().unwrap().name;
        self.server.setup_advertising(name, SERVICE_ROME_UUID)?;

        self.server.gatts.create_service(
            gatt_if,
            &GattServiceId {
                id: GattId {
                    uuid: SERVICE_ROME_UUID,
                    inst_id: 0,
                },
                is_primary: true,
            },
            8,
        )?;

        Ok(())
    }

    /// Called after GATT server service creation, start service and setup characteristics
    fn configure_and_start_service(&self, service_handle: Handle) -> Result<(), EspError> {
        self.state.lock().unwrap().service_handle = Some(service_handle);

        self.server.gatts.start_service(service_handle)?;

        // Characteristics added here must be matched in `register_characteristic()`

        self.server.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: CHAR_ROME_RX_UUID,
                permissions: enum_set!(Permission::Write),
                properties: enum_set!(Property::Write),
                max_len: 200,
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        self.server.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: CHAR_ROME_TX_UUID,
                permissions: enum_set!(),
                properties: enum_set!(Property::Indicate),
                max_len: 200,
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        Ok(())
    }

    /// Called during GATT server service setup, when a characteristic is added
    fn register_characteristic(&self, service_handle: Handle, attr_handle: Handle, char_uuid: BtUuid) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();
        if state.service_handle != Some(service_handle) {
            return Ok(());
        }

        // Descriptors added here must be matched in `register_descriptor()`

        if char_uuid == CHAR_ROME_RX_UUID {
            state.rx_handle = Some(attr_handle);
        } else if char_uuid == CHAR_ROME_TX_UUID {
            state.tx_handle = Some(attr_handle);
            self.server.gatts.add_descriptor(
                service_handle,
                &GattDescriptor {
                    uuid: CCCD_UUID,
                    permissions: enum_set!(Permission::Read | Permission::Write),
                },
            )?;
        } else {
            // Should never happen
        }
        Ok(())
    }

    /// Called during GATT server service setup, when a descriptor is added
    fn register_descriptor(&self, service_handle: Handle, attr_handle: Handle, descr_uuid: BtUuid) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();
        if state.service_handle != Some(service_handle) {
            return Ok(());
        }

        if Some(attr_handle) == state.tx_handle && descr_uuid == CCCD_UUID {
            state.tx_cccd_handle = Some(attr_handle);
        } else {
            // Should never happen
        }

        Ok(())
    }

    /// Called on an incoming connection
    fn create_conn(&self, conn_id: ConnectionId, addr: BdAddr) -> Result<(), EspError> {
        // Note: clients will "use" a slot even if they fail to authenticate
        // Therefore, DOS attacks are very easy to perform.
        let added = {
            let mut state = self.state.lock().unwrap();
            if state.connections.len() < MAX_CONNECTIONS {
                state.connections.push(Connection { peer: addr, conn_id, });
                true
            } else {
                false
            }
        };

        if added {
            self.server.set_peer_encryption(addr)?;
            self.server.set_conn_params_conf(addr, 10, 20, 0, 400)?;
        }

        Ok(())
    }

    /// Called when a peer disconnects
    fn delete_conn(&self, conn_id: ConnectionId) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();

        if let Some(index) = state
            .connections
            .iter()
            .position(|c| c.conn_id == conn_id)
        {
            state.connections.swap_remove(index);
        }

        if state.connections.is_empty() {
            // Restart advertising, just in case (currently, should not be needed)
            if let Err(e) = self.server.start_advertising() {
                error!("Unable to restart advertising: {e:?}");
            }
        }

        Ok(())
    }

    /// Called after a peer confirmed an indication
    fn confirm_indication(&self) {
        let mut state = self.state.lock().unwrap();
        if state.tx_confirmed.is_none() {
            error!("Indication confirmation received but none was expected")
        } else {
            state.tx_confirmed = None;
            self.condvar.notify_all();
        }
    }

    /// Called on incoming "write", return true if message has been processed
    fn on_write_data(&self, handle: Handle, value: &[u8]) -> bool {
        let state = self.state.lock().unwrap();

        if Some(handle) == state.tx_cccd_handle {
            // Nothing to do, could be used to keep track of subscribers
        } else if Some(handle) == state.rx_handle {
            if let Err(e) = self.rome_rx.send(Box::from(value)) {
                error!("Failed to push RX data: {e:?}");
            }
        } else {
            return false;
        }

        true
    }
}
