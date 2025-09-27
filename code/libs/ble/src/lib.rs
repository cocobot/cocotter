pub mod rome;
pub mod game_controller;

use std::sync::{mpsc::{Receiver, Sender}, Arc};
use esp_idf_svc::{
    bt::{
        ble::gap::EspBleGap,
        Ble, BtDriver,
    },
};

use crate::rome::RomePeripheral;
use crate::game_controller::GameControllerCentral;
pub use crate::game_controller::GameControllerEvent;

pub struct BleComm {
  
}

impl BleComm {
    pub fn run(
        bt: BtDriver<'static, Ble>,
        name: String,
        enable_remote_control: bool,
    ) -> (Sender<Box<[u8]>>, Receiver<Box<[u8]>>, Receiver<GameControllerEvent>) {
        let bt = Arc::new(bt);
        let gap = Arc::new(EspBleGap::new(bt.clone()).unwrap());
        let gatts = Arc::new(esp_idf_svc::bt::ble::gatt::server::EspGatts::new(bt.clone()).unwrap());

        // Create communication channels
        let (rome_rx, rome_receiver) = std::sync::mpsc::channel();
        let (rome_sender, rome_tx) = std::sync::mpsc::channel();
        let (game_ctrl_event_sender, game_ctrl_event_receiver) = std::sync::mpsc::channel();

        // Create ROME peripheral (server role)
        let rome_peripheral = RomePeripheral::new(
            gap.clone(),
            gatts,
            name,
            rome_rx.clone(),
            rome_tx,
        );

        // Create Game Controller central (client role)
        let game_controller_central = GameControllerCentral::new(
            gap.clone(),
            game_ctrl_event_sender
        );

        // Set up GAP event subscriptions
        let cloned_rome = rome_peripheral.clone();
        let cloned_gc = game_controller_central.clone();
        gap.subscribe(move |event| {
            cloned_rome.on_gap_event(&event);
            cloned_gc.on_gap_event(&event);
        }).unwrap();

        // Start both roles
        rome_peripheral.start();
        if enable_remote_control {
            game_controller_central.start();
        }        

        (rome_sender, rome_receiver, game_ctrl_event_receiver)
    }
}