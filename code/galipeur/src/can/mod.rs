use std::sync::{Arc, Mutex};
use board_sabotter::SabotterBoard;
use cancaner::{CanInterface, CanMessage};
use embedded_can::Frame;

type Callback = Box<dyn FnMut(&CanMessage) + Send + 'static>;

pub mod ota_relay;

pub struct GalipeurCan<B: SabotterBoard> {
    callbacks: Arc<Mutex<Vec<Callback>>>,
    can: B::Can,
}

impl<B: SabotterBoard> Clone for GalipeurCan<B> {
    fn clone(&self) -> Self {
        Self {
            callbacks: self.callbacks.clone(),
            can: self.can.clone(),
        }
    }
}

impl<B: SabotterBoard> GalipeurCan<B> {
    pub fn new(can: B::Can) -> Self {
        let callbacks: Arc<Mutex<Vec<Callback>>> = Arc::new(Mutex::new(Vec::new()));
        let cloned_can = can.clone();
        let cloned_callbacks = callbacks.clone();

        std::thread::Builder::new()
            .name("can-rx".into())
            .stack_size(8192)
            .spawn(move || loop {
                match cloned_can.can_receive() {
                    Ok(frame) => {
                        if let Some(msg) = CanMessage::from_frame(&frame) {
                            let mut cbs = cloned_callbacks.lock().unwrap();
                            for cb in cbs.iter_mut() {
                                cb(&msg);
                            }
                        } else {
                            log::warn!("CAN: unknown or invalid frame: id={:?} len={}", frame.id(), frame.data().len());
                        }
                    }
                    Err(e) => {
                        log::error!("CAN receive error: {:?}", e);
                    }
                }
            })
            .expect("spawn can-rx");

        Self { callbacks, can }
    }

    pub fn add_callback<F: FnMut(&CanMessage) + Send + 'static>(&self, callback: F) {
        self.callbacks.lock().unwrap().push(Box::new(callback));
    }

    pub fn send(&self, msg: &CanMessage) {
        let msg = msg.encode();
        const MAX_TX_TRIES: u32 = 3;
        
        if let Some(frame) = Frame::new(msg.id, &msg.data[..msg.len]) {
            if let Err(e) = self.can.can_transmit(&frame) {
                log::error!("CAN transmit error: {:?}", e);
            }
        }
        else {
            log::error!("CAN: failed to encode message for transmission");
        }
    }
}