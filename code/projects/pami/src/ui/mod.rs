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
        //log::info!("vbatt {}", voltage_mv);

        // lifepo4 discharge example:  https://cleversolarpower.com/lifepo4-voltage-chart/
        const VBATT_LVL_WARN_MV : f32 = 3150.0; // corresponds to around 15% SOC battery SOC
        const VBATT_LVL_ERR_MV  : f32 = 3000.0; // corresponds to around 5% battery SOC

        match voltage_mv {
            mv if mv <= VBATT_LVL_ERR_MV  => self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([0.5, 0.0, 0.0]) } ), // error : stop required
            mv if mv <= VBATT_LVL_WARN_MV => self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([0.5, 0.5, 0.0]) } ), // low battery
            _                                  => self.event.send_event(Event::Pwm { pwm_event: PWMEvent::LedVbatt([0.0, 0.5, 0.0]) } ), // ok
        };

        let percentage = (voltage_mv - VBATT_LVL_ERR_MV) / (VBATT_LVL_WARN_MV - VBATT_LVL_ERR_MV) * 100.0;
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