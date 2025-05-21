mod screens;

use board_pami_2023::DisplayType;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::{Drawable, Point, Primitive, Size}, primitives::{PrimitiveStyle, Rectangle}};
use esp_idf_svc::sys::random;
use screens::{bottom::Bottom, score::Score, start::Start, tof::Tof, top::Top};

use crate::{events::{Event, EventSystem}, pwm::PWMEvent};
use std::{sync::mpsc, time::{Duration, Instant}};

pub struct UI {
    event: EventSystem,
    display: DisplayType,

    top: Top,
    bottom: Bottom,

    screens: Vec<Box<dyn UIScreen + Send>>,
}

impl UI {
    pub fn new(display: DisplayType, id: usize, color: String, event: &EventSystem) {
        let instance: UI = UI {
            event: event.clone(),
            display,
            top: Top::new(id, color),
            bottom: Bottom::new(),
            screens: vec![
                Box::new(Score::new()),
                Box::new(Start::new()),
                Box::new(Tof::new()),
            ],
        };

        let (sender, receiver) = mpsc::channel();

        event.register_receiver_callback(None, move |evt| {
            sender.send(evt.clone()).ok();
        });

        std::thread::Builder::new().stack_size(16 * 8192).name("UI".to_string()).spawn(move || {
            instance.run(receiver);
        }).unwrap();
    }

    fn handle_battery_event(&mut self, voltage_mv: f32) {        
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
            let v1 = LIFEPO4_CURVE[i - 1].0;
            let p1 = LIFEPO4_CURVE[i - 1].1;
            let v2 = LIFEPO4_CURVE[i].0;
            let p2 = LIFEPO4_CURVE[i].1;
            
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

        self.top.set_battery(voltage_mv, percentage);
    }

    fn run(mut self, receiver: mpsc::Receiver<Event>) {
        let mut last_screen_refresh = Instant::now();
        let mut current_screen = 0;
        let mut last_screen_change = Instant::now();
        loop {
            match receiver.recv() {
                Ok(event) => {
                    match event {
                        Event::Vbatt { voltage_mv } => self.handle_battery_event(voltage_mv),
                        _ => {}
                    }
                    for i in 0..self.screens.len() {
                        self.screens[i].handle_event(&event);
                    }
                }
                Err(_) => {}
            }

            if last_screen_refresh.elapsed() > Duration::from_millis(250) {                
             
                //select the screen to draw
                let mut highest_priority = 0;
                let mut highest_priority_screen_id = 0;
                for i in 0..self.screens.len() {
                        let prio = self.screens[i].get_priority();
                        if prio > highest_priority {
                            highest_priority = self.screens[i].get_priority();
                            highest_priority_screen_id = i;
                        }
                }
                if highest_priority > 0 {
                    if current_screen != highest_priority_screen_id {
                        last_screen_change = Instant::now();
                        current_screen = highest_priority_screen_id;
                    }
                }
                else if last_screen_change.elapsed() > Duration::from_millis(3000) {
                    let mut candidates = Vec::new();
                    for i in 0..self.screens.len() {
                        if self.screens[i].get_priority() == 0 && i != current_screen {
                            candidates.push(i);
                        }
                    }
                    if candidates.len() > 0 {
                        current_screen = candidates[((unsafe { random() } & 0xFFFF) as usize) % candidates.len()];
                        last_screen_change = Instant::now();
                    }
                }

                let current_screen = &mut self.screens[current_screen];

                self.bottom.set_screen_name(current_screen.get_screen_name());

                self.top.draw(&mut self.display, Point::zero(), Size::new(128, 7));
                self.bottom.draw(&mut self.display, Point::new(0, 64 - 7), Size::new(128, 7));

                Rectangle::new(Point::new(0, 7), Size::new(128, 64 - 14))
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                    .draw(&mut self.display)
                    .unwrap();
                current_screen.draw(&mut self.display, Point::new(0, 7), Size::new(128, 64 - 14));

                self.display.flush().unwrap();

                last_screen_refresh = Instant::now();
            }
        }
    }
}

trait UIScreen {
    fn draw(&mut self, display: &mut DisplayType, offset: Point, size: Size);
    fn get_screen_name(&self) -> &'static str;
    fn handle_event(&mut self, _event: &Event) {}
    fn get_priority(&self) -> isize {
        0
    }
}