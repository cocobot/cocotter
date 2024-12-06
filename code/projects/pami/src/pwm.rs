use board_pami_2023::{I2C0PamiDevice, Pca9685, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_VBAT_RGB};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}};


static EVENT_QUEUE: Channel<CriticalSectionRawMutex, PWMEvent, 32> = Channel::new();

pub enum PWMEvent {
    LedVbatt([f32; 3]),
    LedBottom([f32; 3]),
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
            }
        }
    }
}


#[embassy_executor::task]
async fn start_pwm_thread(thread: PWM) {
    thread.run().await;
}
