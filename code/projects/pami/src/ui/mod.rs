use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, Receiver, TrySendError}};
use embassy_time::{Duration, Timer};

use crate::{events::{Event, EventSystem}, pwm::PWMEvent};

static EVENT_QUEUE: Channel<CriticalSectionRawMutex, Event, 32> = Channel::new();

#[derive(Clone)]
pub struct UI {
}

impl UI {
    pub async fn new(spawner: Spawner, event: &EventSystem) {
        let thread = UIThread::new(EVENT_QUEUE.receiver(), event.clone());

        spawner.spawn(start_ui_thread(thread)).unwrap();

        event.register_receiver_callback(Some(UI::filter_events), move |evt| {
            async move {
                UI::send_event(evt);
            }
        }).await;
    }

    fn filter_events(evt: &Event) -> bool {
        match evt {
            Event::Vbatt { .. } => true,
            _ => false,
        }
    } 

    fn send_event(event: Event) {
        match EVENT_QUEUE.try_send(event) {
            Ok(_) => {}
            Err(TrySendError::Full(_)) => {
                log::error!("UI event queue full");
            }
        }
    }
}


struct UIThread {
    event_receiver: Receiver<'static, CriticalSectionRawMutex, Event, 32>,
    event: EventSystem,
}

impl UIThread {
    pub fn new(event_receiver: Receiver<'static, CriticalSectionRawMutex, Event, 32>, event: EventSystem) -> Self {
        Self {
            event_receiver,
            event,
        }
    }

    fn handle_battery_event(&self, voltage_mv: f32) {
        // LiFePO4 voltage to percentage lookup table (voltage in mV, percentage points)
        const LIFEPO4_CURVE: [(f32, f32); 11] = [
            (3400.0, 100.0),
            (3350.0, 90.0), 
            (3320.0, 80.0),  
            (3300.0, 70.0), 
            (3270.0, 60.0), 
            (3260.0, 50.0), 
            (3350.0, 40.0), 
            (3220.0, 35.0), 
            (3200.0, 20.0), 
            (3000.0, 10.0), 
            (2500.0, 0.0), 
        ];

        // Find percentage using the lookup table
        let percentage = if voltage_mv >= LIFEPO4_CURVE[0].0 {
            LIFEPO4_CURVE[0].1
        } else if voltage_mv <= LIFEPO4_CURVE[LIFEPO4_CURVE.len() - 1].0 {
            LIFEPO4_CURVE[LIFEPO4_CURVE.len() - 1].1
        } else {
            // Find the two voltage points to interpolate between
            let mut i = 0;
            while i < LIFEPO4_CURVE.len() - 1 && voltage_mv < LIFEPO4_CURVE[i].0 {
                i += 1;
            }
            
            // Linear interpolation between the two points
            let v1 = LIFEPO4_CURVE[i].0;
            let p1 = LIFEPO4_CURVE[i].1;
            let v2 = LIFEPO4_CURVE[i + 1].0;
            let p2 = LIFEPO4_CURVE[i + 1].1;
            
            p1 + (p2 - p1) * (voltage_mv - v1) / (v2 - v1)
        };

        if percentage < 30.0 {
            self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([1.0, 0.0, 0.0]) } ); // error
        }
        else if percentage < 90.0 {
            self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([1.0, 0.27, 0.0]) } );// low battery
        }
        else {
            self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([0.0, 1.0, 0.0]) } ); // ok battery
        }

        self.event.send_event(Event::VbattPercent { percent: percentage.clamp(0.0, 100.0) as u8 });
    }

    fn handle_event(&self, event: Event) {
        match event {
            Event::Vbatt { voltage_mv } => self.handle_battery_event(voltage_mv),

            _ => {}
        }
    }

    async fn run(self) {
        loop {
            let rx_evt = select(self.event_receiver.receive(), Timer::after(Duration::from_millis(250))).await;
            if let Either::First(rx_evt) = rx_evt {
                self.handle_event(rx_evt);
            }

            //update display
            //TODO !!!
        }
    }
}

#[embassy_executor::task]
async fn start_ui_thread(thread: UIThread) {
    thread.run().await;
}