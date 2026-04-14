use std::time::Duration;
use board_common::{Color, Team};
use board_sabotter::{SabotterBoard, SabotterLeds};
use flume::{Receiver, Sender};
use embedded_hal::digital::OutputPin;
use board_sabotter::{SmartLedsWrite, RGB8};


pub enum LedMessage {
    GameSide { side: Team },
    RomeActivity,
    LowPowerBattery,
    LowLogicBattery,
    GroundSensor(bool, bool, bool),
}

pub struct Leds {
    tx: Sender<LedMessage>,
}

impl Leds {
    pub fn new<B: SabotterBoard + 'static>(board: &mut B) -> Self {
        let (tx, rx) = flume::unbounded();

        let mut internal = LedsInternal::new(board, rx);

        //start led thread
        std::thread::spawn(move || {
            internal.run();
        });

        Leds { tx }
    }

    pub fn sender(&self) -> Sender<LedMessage> {
        self.tx.clone()
    }
}

struct LedsInternal<B: SabotterBoard> {
    leds: SabotterLeds<B::OutputPin, B::ExOutputPin, B::SmartLeds>,
    rx: Receiver<LedMessage>,

    game_color: RGB8,
    low_logic_battery: bool,
    low_power_battery: bool,
    ground_detected: (bool, bool, bool),
}

impl<B: SabotterBoard> LedsInternal<B> {
    pub fn new(board: &mut B, rx: Receiver<LedMessage>) -> Self {
        Self {
            leds: board.leds().unwrap(),
            rx,
            game_color: RGB8 { r: 0, g: 0, b: 0 },
            low_logic_battery: false,
            low_power_battery: false,
            ground_detected: (false, false, false),
        }
    }

    fn color_to_rgb8(c: Color) -> RGB8 {
        RGB8 {
            r: (c.r * 255.0) as u8,
            g: (c.g * 255.0) as u8,
            b: (c.b * 255.0) as u8,
        }
    }

    fn run(&mut self) {
        let start = std::time::Instant::now();
        let mut rome_activity = false;
        let mut previous_rome_activity = !rome_activity;
        let mut previous_heartbeat = false;
        let mut pixels = [BLACK; 41];

        const BLACK : RGB8 = RGB8 { r: 0, g: 0, b: 0 };
        const RED : RGB8 = RGB8 { r: 255, g: 0, b: 0 };
        const BROWN : RGB8 = RGB8 {r: 137,g: 81, b: 41};

        loop {
            //Process all pending messages
            while let Ok(msg) = self.rx.try_recv() {
                match msg {
                    LedMessage::GameSide { side } => {
                        self.game_color = Self::color_to_rgb8(side.color());
                    }
                    LedMessage::RomeActivity => {
                        rome_activity = true;
                    }
                    LedMessage::LowPowerBattery => {
                        self.low_power_battery = true;
                    }
                    LedMessage::LowLogicBattery => {
                        self.low_logic_battery = true;
                    }
                    LedMessage::GroundSensor(s0, s1, s2 ) => {
                        self.ground_detected = (s0, s1, s2);
                    }
                }
            }

            // Battery low: blink all LEDs red, overrides everything
            if self.low_logic_battery || self.low_power_battery {
                let blink_on = (start.elapsed().subsec_millis() % 500) < 250;
                let color = if blink_on { RED } else { BLACK };
                for i in 0..41 {
                    if !self.low_logic_battery && ((i >= 1 && i <= 7) || (i >= 27)) {
                        pixels[i] = BLACK
                    } else if !self.low_power_battery && (i >= 8 && i <= 27) {
                        pixels[i] = BLACK
                    } else {
                        pixels[i] = color;
                    }
                }
            } else {
                pixels[0] = self.game_color;

                for i in 0..41 {
                    pixels[i] = BLACK
                }

                let ground_blink_on = (start.elapsed().subsec_millis() % 100) < 50;
                let ground_error_color = if ground_blink_on { RED } else { BLACK };

                if !self.ground_detected.0 {
                    pixels[31] = ground_error_color;
                    pixels[34] = ground_error_color;
                }
                if !self.ground_detected.1 {
                    pixels[4] = ground_error_color;
                    pixels[7] = ground_error_color;
                }
                if !self.ground_detected.2 {
                    pixels[18] = ground_error_color;
                    pixels[21] = ground_error_color;
                }
            }

            self.leds.rgba.write(pixels).ok();

            //update rome led
            if rome_activity != previous_rome_activity {
                if rome_activity {
                    self.leds.com.set_high().ok();
                }
                else {
                    self.leds.com.set_low().ok();
                }
                previous_rome_activity = rome_activity;                
            }
            rome_activity = false;

            //update heartbeat
            let heartbeat_on = (start.elapsed().as_millis() % 1000) < 500;
            if heartbeat_on != previous_heartbeat {
                if heartbeat_on {
                    self.leds.heartbeat.set_high().ok();
                    for led in &mut self.leds.motors {
                        led.set_high().ok();
                    }
                } else {
                    self.leds.heartbeat.set_low().ok();
                    for led in &mut self.leds.motors {
                        led.set_low().ok();
                    }
                }
                previous_heartbeat = heartbeat_on;
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
