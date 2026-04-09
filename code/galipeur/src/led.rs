use std::time::Duration;
use flume::{Receiver, Sender};
use smart_leds::{RGB8, hsv::{Hsv, hsv2rgb}};
use smart_leds::SmartLedsWrite;
use super::SmartLeds;

pub enum LedMessage {
    LidarClosest { angle: f32, distance: u16 },
    GameColor { color: RGB8 },
    BatteryLow,
}

pub struct Leds {
    tx: Sender<LedMessage>,
}

impl Leds {
    pub fn new(leds: SmartLeds) -> Self {
        let (tx, rx) = flume::unbounded();

        let mut internal = LedsInternal::new(leds, rx);

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

struct LedsInternal {
    leds: SmartLeds,
    rx: Receiver<LedMessage>,
    lidar_angle: f32,
    lidar_distance: u16,
    game_color: RGB8,
    battery_low: bool,
}

impl LedsInternal {
    pub fn new(leds: SmartLeds, rx: Receiver<LedMessage>) -> Self {
        Self {
            leds,
            rx,
            lidar_angle: 0.0,
            lidar_distance: 0,
            game_color: RGB8 { r: 0, g: 0, b: 0 },
            battery_low: false,
        }
    }

    fn run(&mut self) {
        let start = std::time::Instant::now();
        loop {
            // Process all pending messages
            while let Ok(msg) = self.rx.try_recv() {
                match msg {
                    LedMessage::LidarClosest { angle, distance } => {
                        self.lidar_angle = angle;
                        self.lidar_distance = distance;
                    }
                    LedMessage::GameColor { color } => {
                        self.game_color = color;
                    }
                    LedMessage::BatteryLow => {
                        self.battery_low = true;
                    }
                }
            }

            // Display lidar closest point direction on LED ring
            // Map angle (0-360°) to LED index (1-40), LED 0 is separate
            // Angle 0° = LED 14, rotation inverted (counter-clockwise)
            let led_offset = (40 - ((self.lidar_angle / 360.0) * 40.0) as usize) % 40;
            let led_index = 1 + (14 + led_offset - 1) % 40;

            // Map distance to hue: 150mm = red (0), 1000mm = green (85)
            let hue = if self.lidar_distance <= 150 {
                0 // Red
            } else if self.lidar_distance >= 1000 {
                85 // Green
            } else {
                // Linear interpolation: 150->0, 1000->85
                ((self.lidar_distance - 150) as u32 * 85 / (1000 - 150)) as u8
            };

            let mut pixels = Vec::new();

            // Battery low: blink all LEDs red, overrides everything
            if self.battery_low {
                let blink_on = (start.elapsed().subsec_millis() % 500) < 250;
                let color = if blink_on { RGB8 { r: 255, g: 0, b: 0 } } else { RGB8 { r: 0, g: 0, b: 0 } };
                log::info!("Color: {:?}, Blink: {}, Time: {}", color, blink_on, start.elapsed().subsec_millis());
                for _ in 0..41 {
                    pixels.push(color);
                }
            } else {
                for i in 0..41 {
                    if i == 0 {
                        pixels.push(self.game_color);
                    } else if i == led_index {
                        pixels.push(hsv2rgb(Hsv {
                            hue,
                            sat: 255,
                            val: 255,
                        }));
                    } else {
                        pixels.push(hsv2rgb(Hsv {
                            hue: 0,
                            sat: 0,
                            val: 0,
                        }));
                    }
                }
            }

            self.leds.write(pixels).unwrap();

            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
