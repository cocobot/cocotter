use board_pami::{PwmController, PWM_EXTENDED_CHANEL_VACCUM, PWM_EXTENDED_CHANNEL_SERVO, PWM_EXTENDED_ENABLE_TOF, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_LINE_LED, PWM_EXTENDED_RESET_TOF, PWM_EXTENDED_VBAT_RGB};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use std::sync::mpsc::{self, channel, Sender};

#[derive(Debug, Clone, Copy)]
pub enum OverrideState {
    Override,
    ResetOverride,
}

impl From<u8> for OverrideState {
    fn from(value: u8) -> Self {
        match value & 0x80 {
            0 => OverrideState::ResetOverride,
            _ => OverrideState::Override,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
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
    device: PwmController,
}

impl PWM {
    pub fn init(device: PwmController) -> Sender<PWMEvent> {
        let instance = PWM {
            device,
        };

        let (sender, receiver) = channel();

        ThreadSpawnConfiguration {
            name: Some("PWM\0".as_bytes()),
            stack_size: 8192 * 4,
            ..Default::default()
        }.set().unwrap();
        std::thread::Builder::new().spawn(move || {
            instance.run(receiver);
        }).unwrap();

        sender
    }


    pub fn run(self, receiver: mpsc::Receiver<PWMEvent>) {
        let device = self.device.clone();
        let mut device = device.lock().unwrap();

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

        // 60Hz for servomotor        
        if device.set_prescale(100).is_err() {
            //only display error once if device is not soldered
            log::error!("No pca9685 working");
        }
        device.enable().ok();
     
        loop {
            let evt = receiver.recv().unwrap();

            match evt {
                PWMEvent::LedVbatt(color) => {
                    device.set_channel_off(PWM_EXTENDED_VBAT_RGB[0], clamp_duty((4095.0 * (1.0 - color[0])) as i16)).ok();
                    device.set_channel_off(PWM_EXTENDED_VBAT_RGB[1], clamp_duty((4095.0 * (1.0 - color[1])) as i16)).ok();
                    device.set_channel_off(PWM_EXTENDED_VBAT_RGB[2], clamp_duty((4095.0 * (1.0 - color[2])) as i16)).ok();
                }
                PWMEvent::LedBottom(color) => {
                    device.set_channel_off(PWM_EXTENDED_LED_RGB[0], clamp_duty((4095.0 * (1.0 - color[0])) as i16)).ok();
                    device.set_channel_off(PWM_EXTENDED_LED_RGB[1], clamp_duty((4095.0 * (1.0 - color[1])) as i16)).ok();
                    device.set_channel_off(PWM_EXTENDED_LED_RGB[2], clamp_duty((4095.0 * (1.0 - color[2])) as i16)).ok();
                }
                PWMEvent::Vaccum(speed) => {
                    device.set_channel_off(PWM_EXTENDED_CHANEL_VACCUM, clamp_duty((4095.0 * speed) as i16)).ok();
                }
                PWMEvent::Servo0(pos) => {
                    device.set_channel_off(PWM_EXTENDED_CHANNEL_SERVO[0], clamp_duty((4095.0 * pos/10.0) as i16)).ok();
                }
                PWMEvent::Servo1(pos) => {
                    device.set_channel_off(PWM_EXTENDED_CHANNEL_SERVO[1], clamp_duty((4095.0 * pos/10.0) as i16)).ok();
                }
                PWMEvent::Servo2(pos) => {
                    device.set_channel_off(PWM_EXTENDED_CHANNEL_SERVO[2], clamp_duty((4095.0 * pos/10.0) as i16)).ok();
                }
                PWMEvent::Servo3(pos) => {
                    device.set_channel_off(PWM_EXTENDED_CHANNEL_SERVO[3], clamp_duty((4095.0 * pos/10.0) as i16)).ok();
                }
                PWMEvent::LineLedLvl(lvl) => {
                    device.set_channel_off(PWM_EXTENDED_LINE_LED, clamp_duty((4095.0 * lvl) as i16)).ok();
                }
                PWMEvent::ResetTof(state) => {
                    match state{
                        true => device.set_channel_off(PWM_EXTENDED_RESET_TOF,4095).ok(),
                        false => device.set_channel_off(PWM_EXTENDED_RESET_TOF, 0).ok(),
                    };
                }
                PWMEvent::EnableTof(state) => {
                        match state{
                            true => device.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 4095).ok(),
                            false => device.set_channel_off(PWM_EXTENDED_ENABLE_TOF, 0).ok(),
                        };
                }
            }
        }
    }
}