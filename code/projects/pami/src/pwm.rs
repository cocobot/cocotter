use board_pami_2023::{I2C0PamiDevice, Pca9685, PWM_EXTENDED_CHANEL_SERVO, PWM_EXTENDED_CHANEL_VACCUM, PWM_EXTENDED_ENABLE_TOF, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_LINE_LED, PWM_EXTENDED_RESET_TOF, PWM_EXTENDED_VBAT_RGB};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}};


static EVENT_QUEUE: Channel<CriticalSectionRawMutex, PWMEvent, 32> = Channel::new();

pub enum PWMEvent {
    LedVbatt([f32; 3]),
    LedBottom([f32; 3]),
    Vaccum(f32),
    Servo0(f32),
    Servo1(f32),
    Servo2(f32),
    Servo3(f32),
    LineLedLvl(f32),
    ResetTof(bool),
    EnableTof(bool),
}

pub struct PWM {
    device: Pca9685<I2C0PamiDevice>,
}

impl PWM {
    pub async fn new(mut device: Pca9685<I2C0PamiDevice>, spawner: Spawner) {
        // 60Hz for servomotor        
        if device.set_prescale(100).await.is_err() {
            //only display error once if device is not soldered
            log::error!("No pca9685 working");
        }
        device.enable().await.ok();
        let instance = Self {
            device
        };
        spawner.spawn(start_pwm_thread(instance)).unwrap();
    }

    pub fn send_event(event: PWMEvent) {
        match EVENT_QUEUE.try_send(event) {
            Ok(_) => {}
            Err(TrySendError::Full(_)) => {
                log::error!("PWM event queue full");
            }
        }
    }

    pub async fn run(mut self) {
        fn clamp_duty(v: i16) -> u16 {
            if v > 4095 {
                4095
            }
            else if v < 0 {
                0
            }
            else {
                v as u16
            } 
        }

        loop {
            let evt = EVENT_QUEUE.receive().await;

            match evt {
                PWMEvent::LedVbatt(color) => {
                    self.device.set_channel_off(PWM_EXTENDED_VBAT_RGB[0], clamp_duty((4095.0 * (1.0 - color[0])) as i16)).await.ok();
                    self.device.set_channel_off(PWM_EXTENDED_VBAT_RGB[1], clamp_duty((4095.0 * (1.0 - color[1])) as i16)).await.ok();
                    self.device.set_channel_off(PWM_EXTENDED_VBAT_RGB[2], clamp_duty((4095.0 * (1.0 - color[2])) as i16)).await.ok();
                }
                PWMEvent::LedBottom(color) => {
                    self.device.set_channel_off(PWM_EXTENDED_LED_RGB[0], clamp_duty((4095.0 * (1.0 - color[0])) as i16)).await.ok();
                    self.device.set_channel_off(PWM_EXTENDED_LED_RGB[1], clamp_duty((4095.0 * (1.0 - color[1])) as i16)).await.ok();
                    self.device.set_channel_off(PWM_EXTENDED_LED_RGB[2], clamp_duty((4095.0 * (1.0 - color[2])) as i16)).await.ok();
                }
                PWMEvent::Vaccum(speed) => {
                    self.device.set_channel_off(PWM_EXTENDED_CHANEL_VACCUM, clamp_duty((4095.0 * speed) as i16)).await.ok();
                }
                PWMEvent::Servo0(pos) => {
                    self.device.set_channel_off(PWM_EXTENDED_CHANEL_SERVO[0], clamp_duty((4095.0 * pos/10.0) as i16)).await.ok();
                }
                PWMEvent::Servo1(pos) => {
                    self.device.set_channel_off(PWM_EXTENDED_CHANEL_SERVO[1], clamp_duty((4095.0 * pos/10.0) as i16)).await.ok();
                }
                PWMEvent::Servo2(pos) => {
                    self.device.set_channel_off(PWM_EXTENDED_CHANEL_SERVO[2], clamp_duty((4095.0 * pos/10.0) as i16)).await.ok();
                }
                PWMEvent::Servo3(pos) => {
                    self.device.set_channel_off(PWM_EXTENDED_CHANEL_SERVO[3], clamp_duty((4095.0 * pos/10.0) as i16)).await.ok();
                }
                PWMEvent::LineLedLvl(lvl) => {
                    self.device.set_channel_off(PWM_EXTENDED_LINE_LED, clamp_duty((4095.0 * lvl) as i16)).await.ok();
                }
                PWMEvent::ResetTof(state) => {
                    match state{
                        true => self.device.set_channel_off(PWM_EXTENDED_RESET_TOF,4095).await.ok(),
                        false => self.device.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).await.ok(),
                    };
                }
                PWMEvent::EnableTof(state) => {
                        match state{
                            true => self.device.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 4095).await.ok(),
                            false => self.device.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).await.ok(),
                        };
                }
            }
        }
    }
}


#[embassy_executor::task]
async fn start_pwm_thread(thread: PWM) {
    thread.run().await;
}
