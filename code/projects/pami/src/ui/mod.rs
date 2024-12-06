use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, Receiver, Sender, TrySendError}};
use embassy_time::{Duration, Timer};

static EVENT_QUEUE: Channel<CriticalSectionRawMutex, UIEvent, 32> = Channel::new();

pub enum UIEvent {
    Vbatt {voltage_mv: f32},
}

#[derive(Clone)]
pub struct UI {
    event_sender: Sender<'static, CriticalSectionRawMutex, UIEvent, 32>,
}

impl UI {
    pub fn new(spawner: Spawner) -> Self {
        let thread = UIThread::new(EVENT_QUEUE.receiver());

        spawner.spawn(start_ui_thread(thread)).unwrap();

        Self {
            event_sender: EVENT_QUEUE.sender(),
        }
    }

    pub fn send_event(&self, event: UIEvent) {
        match self.event_sender.try_send(event) {
            Ok(_) => {}
            Err(TrySendError::Full(_)) => {
                log::error!("UI event queue full");
            }
        }
    }
}


struct UIThread {
    event_receiver: Receiver<'static, CriticalSectionRawMutex, UIEvent, 32>
}

impl UIThread {
    pub fn new(event_receiver: Receiver<'static, CriticalSectionRawMutex, UIEvent, 32>) -> Self {
        Self {
            event_receiver
        }
    }

    fn handle_event(&self, event: UIEvent) {
        match event {
            UIEvent::Vbatt { voltage_mv } => {log::info!("vbatt {}", voltage_mv)},
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