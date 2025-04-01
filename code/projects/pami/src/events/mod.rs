use core::{future::Future, pin::Pin};

use alloc::{boxed::Box, sync::Arc, vec::Vec};
use cocotter::position::robot_coordinate::RobotCoordinate;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, Sender, TrySendError}, mutex::Mutex};

use crate::{asserv::MotorSetpointOverride, pwm::{OverrideState, PWMEvent}};
extern crate alloc;
pub const EVENT_QUEUE_COUNT: usize = 32;

static EVENT_QUEUE: Channel<CriticalSectionRawMutex, Event, EVENT_QUEUE_COUNT> = Channel::new();

pub type EventFilter = fn(&Event) -> bool;
pub type EventCallback = Box<dyn Fn(Event) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>> + Send + Sync + 'static>;

#[derive(Debug, Clone, Copy)]
pub enum Event {
    //analog inputs
    Vbatt {voltage_mv: f32},
    VbattPercent {percent: u8},
    
    //pwm outputs
    Pwm { pwm_event : PWMEvent},
    OverridePwm { pwm_event : PWMEvent, override_state: OverrideState},

    //asserv
    Position { coords: RobotCoordinate::<2> },
    MotorDebug {timestamp: u16, left_tick: i32, right_tick: i32, left_pwm: i16, right_pwm: i16}, 
    MotorOverrideSetpoint { ovr : Option<MotorSetpointOverride>},
}

#[derive(Clone)]
pub struct EventSystem {
    sender: Sender<'static, CriticalSectionRawMutex, Event, EVENT_QUEUE_COUNT>,
    callbacks: Arc<Mutex<CriticalSectionRawMutex, Vec<(Option<EventFilter>, EventCallback)>>>,
}

impl EventSystem {
    pub fn new(spawner: Spawner) -> Self {
        let cb = Arc::new(Mutex::new(Vec::new()));

        spawner.spawn(start_event_thread(cb.clone())).unwrap();

        Self {
            sender: EVENT_QUEUE.sender(),
            callbacks: cb,
        }
    }

    pub fn send_event(&self, event: Event) {
        match self.sender.try_send(event) {
            Ok(_) => {}
            Err(TrySendError::Full(_)) => {
                log::error!("Event queue full");
            }
        }
    }

    pub async fn register_receiver_callback<F, Fut>(
        &self, 
        filter: Option<EventFilter>, 
        callback: F
    ) where 
        F: Fn(Event) -> Fut + Send + Sync + 'static,
        Fut: Future<Output = ()> + Send + 'static
    {
        self.callbacks.lock().await.push((
            filter, 
            Box::new(move |evt| Box::pin(callback(evt)) as Pin<Box<dyn Future<Output = ()> + Send + 'static>>)
        ));
    }
}

#[embassy_executor::task]
async fn start_event_thread(callbacks: Arc<Mutex<CriticalSectionRawMutex, Vec<(Option<EventFilter>, EventCallback)>>>) {
    let receiver = EVENT_QUEUE.receiver();
  
    loop {
        let event = receiver.receive().await;
        let callbacks = callbacks.lock().await;
        
        // Créer un vecteur pour stocker les futures à exécuter
        let mut futures = Vec::new();
        
        for (filter, callback) in callbacks.iter() {
            if filter.is_none() || filter.unwrap()(&event) {
                // Appeler le callback et stocker le future
                futures.push(callback(event));
            }
        }
        
        // Libérer le mutex avant d'attendre les futures
        drop(callbacks);
        
        // Exécuter tous les futures en parallèle
        for future in futures {
            // On pourrait utiliser join! de futures-util ici pour les exécuter en parallèle
            // mais pour la simplicité, on les exécute séquentiellement
            future.await;
        }
    }
}
