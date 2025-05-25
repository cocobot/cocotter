use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::efuse::Efuse;
use esp_wifi::ble::controller::BleConnector;
use trouble_host::prelude::*;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

pub const L2CAP_MTU: usize = 255;

use crate::{asserv::MotorSetpointOverride, events::{Event, EventSystem}, pwm::{OverrideState, PWMEvent}};

static EVENT_QUEUE: Channel<CriticalSectionRawMutex, Event, 32> = Channel::new();

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
    pami: PamiService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

/// Battery service
#[gatt_service(uuid = "c10e0000-5a32-42a0-886b-cf9d57a5fd4a")]
struct PamiService {
    #[characteristic(uuid = "c10e0001-5a32-42a0-886b-cf9d57a5fd4a", notify)]
    override_pwm: [u8; 13],

    #[characteristic(uuid = "c10e0002-5a32-42a0-886b-cf9d57a5fd4a", notify)]
    position: [u8; 20],

    #[characteristic(uuid = "c10e0003-5a32-42a0-886b-cf9d57a5fd4a", notify)]
    motor_debug: [u8; 14],

    #[characteristic(uuid = "c10e0004-5a32-42a0-886b-cf9d57a5fd4a", write)]
    override_motor: [u8; 5],
}


pub struct CommBle {

}

impl CommBle {
    pub async fn new(
            connector: BleConnector<'static>,
            spawner: Spawner, 
            event: &EventSystem,
        ) -> CommBle {
        spawner.spawn(start_ble_thread(connector, event.clone())).unwrap();

        event.register_receiver_callback(Some(CommBle::filter), CommBle::event_callback).await;

        Self {

        }
    }

    fn filter(event: &Event) -> bool {
        match event {
            Event::Position { .. } => true,
            Event::VbattPercent { .. } => true,
            Event::MotorDebug { .. } => true,
            _ => false,
        }
    }

    async fn event_callback(event: Event) {
        EVENT_QUEUE.try_send(event).ok();
    }
}

#[embassy_executor::task]
async fn start_ble_thread(
    connector: BleConnector<'static>,
    event: EventSystem) 
{

    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    let mac_address = Efuse::read_base_mac_address();
    let address: Address = Address::random(mac_address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, runner, ..
    } = stack.build();

    log::info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Cocotter PAMI",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Cocotter PAMI", &mut peripheral).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn, &event);
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

async fn gatt_events_task(server: &Server<'_>, conn: &Connection<'_>, event: &EventSystem) -> Result<(), Error> {
    let level = server.battery_service.level;
    let override_pwm = server.pami.override_pwm;
    let override_motor = server.pami.override_motor;

    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                log::info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            ConnectionEvent::Gatt { data } => {
                match data.process(server).await {
                    // Server processing emits
                    Ok(Some(evt)) => {
                        match &evt {
                            GattEvent::Read(evt) => {
                                if evt.handle() == level.handle {
                                    let value = server.get(&level);
                                    log::info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                                }
                            }
                            GattEvent::Write(evt) => {
                                if evt.handle() == level.handle {
                                    log::info!("[gatt] Write Event to Level Characteristic: {:?}", evt.data());
                                }
                                if evt.handle() == override_pwm.handle {
                                    let data = evt.data();
                                    if data.len() > 0 {
                                        let override_state = OverrideState::from(data[0]);
                                        if let Ok(value) = PWMEvent::try_from(data) {
                                            event.send_event(Event::OverridePwm { pwm_event: value, override_state: override_state });
                                        }
                                    }
                                }
                                if evt.handle() == override_motor.handle {
                                    let data = evt.data();
                                    log::info!("[gatt] Write Event to Override Motor Characteristic: {:?}", data);
                                    if data.len() == 5 {
                                        let after_filter = data[0] > 0;
                                        let left = i16::from_le_bytes([data[1], data[2]]);
                                        let right = i16::from_le_bytes([data[3], data[4]]);

                                        event.send_event(Event::MotorOverrideSetpoint {
                                            ovr: Some(MotorSetpointOverride {
                                                after_filter,
                                                left,
                                                right,
                                            })
                                        });
                                    }
                                }
                            }
                        }

                        // This step is also performed at drop(), but writing it explicitly is necessary
                        // in order to ensure reply is sent.
                        match evt.accept() {
                            Ok(reply) => {
                                reply.send().await;
                            }
                            Err(e) => {
                                log::warn!("[gatt] error sending response: {:?}", e);
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        log::warn!("[gatt] error processing event: {:?}", e);
                    }
                }
            }
        }
    }
    log::info!("[gatt] task finished");
    Ok(())
}

async fn advertise<'a, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C>,
) -> Result<Connection<'a>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x0f, 0x18])]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            },
        )
        .await?;
    log::info!("[adv] advertising");
    let conn = advertiser.accept().await?;
    log::info!("[adv] connection established");
    Ok(conn)
}

async fn custom_task<C: Controller>(server: &Server<'_>, conn: &Connection<'_>, _stack: &Stack<'_, C>) {
    let level = server.battery_service.level;
    let position = server.pami.position;
    let motor_debug = server.pami.motor_debug;

    let receiver = EVENT_QUEUE.receiver();
    receiver.clear();

    let should_quit = |e: &Error| {
        match e  {
            Error::Disconnected => true,
            _ => false,
        }
    };
    
    loop {
        let event = receiver.receive().await;

        let r = match event {
            Event::Position { coords } => {
                let mut data = [0 as u8; 20];

                data[0..4].copy_from_slice(&f32::to_le_bytes(coords.get_x_mm()));
                data[4..8].copy_from_slice(&f32::to_le_bytes(coords.get_y_mm()));
                data[8..12].copy_from_slice(&f32::to_le_bytes(coords.get_a_rad()));
                data[12..16].copy_from_slice(&f32::to_le_bytes(coords.get_distance_speed_mm_s()));
                data[16..20].copy_from_slice(&f32::to_le_bytes(coords.get_angle_speed_rad_s()));

        
                position.notify(server, conn, &data).await
            }

            Event::MotorDebug { timestamp, left_tick, right_tick, left_pwm, right_pwm } => {
                let mut data = [0 as u8; 14];

                data[0..2].copy_from_slice(&u16::to_le_bytes(timestamp));
                data[2..6].copy_from_slice(&i32::to_le_bytes(left_tick));
                data[6..10].copy_from_slice(&i32::to_le_bytes(right_tick));
                data[10..12].copy_from_slice(&i16::to_le_bytes(left_pwm));
                data[12..14].copy_from_slice(&i16::to_le_bytes(right_pwm));

                motor_debug.notify(server, conn, &data).await
            }

            Event::VbattPercent { percent } => {
                level.notify(server, conn, &percent).await
            }
            _ => {
                log::error!("Unexpected event {:?}", event);
                continue;
            }
        };

        if let Err(e) = r {
            if should_quit(&e) {
                log::info!("[custom_task] error notifying connection {:?}", e);
                return;
            }
        };

        /*
        if let Ok(rssi) = conn.rssi(stack).await {
            log::info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            log::info!("[custom_task] error getting RSSI");
            break;
        };*/
    }
}