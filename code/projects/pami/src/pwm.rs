use board_pami_2023::{I2C0PamiDevice, Pca9685, PWM_EXTENDED_CHANEL_SERVO, PWM_EXTENDED_CHANEL_VACCUM, PWM_EXTENDED_ENABLE_TOF, PWM_EXTENDED_LED_RGB, PWM_EXTENDED_LINE_LED, PWM_EXTENDED_RESET_TOF, PWM_EXTENDED_VBAT_RGB};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}};

use crate::events::{Event, EventSystem};

const PWM_MAX_EVENT_COUNT: usize = 10;

static EVENT_QUEUE: Channel<CriticalSectionRawMutex, (PWMEvent, Option<OverrideState>), 32> = Channel::new();

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

impl Into<usize> for PWMEvent {
    fn into(self) -> usize {
        match self {
            PWMEvent::LedVbatt(_) => 0,
            PWMEvent::LedBottom(_) => 1,
            PWMEvent::Vaccum(_) => 2,
            PWMEvent::Servo0(_) => 3,
            PWMEvent::Servo1(_) => 4,
            PWMEvent::Servo2(_) => 5,
            PWMEvent::Servo3(_) => 6,
            PWMEvent::LineLedLvl(_) => 7,
            PWMEvent::ResetTof(_) => 8,
            PWMEvent::EnableTof(_) => 9,
        }
    }
}

impl TryFrom<&[u8]> for PWMEvent {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() < 13 {
            return Err(());
        }

        let event_id = value[0] & 0x7F;
        let first_float = f32::from_le_bytes([value[1], value[2], value[3], value[4]]);
        let second_float = f32::from_le_bytes([value[5], value[6], value[7], value[8]]);
        let third_float = f32::from_le_bytes([value[9], value[10], value[11], value[12]]);
        let first_bool = value[1] != 0;

        match event_id {
            0 => Ok(PWMEvent::LedVbatt([first_float, second_float, third_float])),
            1 => Ok(PWMEvent::LedBottom([first_float, second_float, third_float])),
            2 => Ok(PWMEvent::Vaccum(first_float)),
            3 => Ok(PWMEvent::Servo0(first_float)),
            4 => Ok(PWMEvent::Servo1(first_float)),
            5 => Ok(PWMEvent::Servo2(first_float)),
            6 => Ok(PWMEvent::Servo3(first_float)),
            7 => Ok(PWMEvent::LineLedLvl(first_float)),
            8 => Ok(PWMEvent::ResetTof(first_bool)),
            9 => Ok(PWMEvent::EnableTof(first_bool)),
            _ => Err(()),
        }
    }
}

pub struct PWM {
    device: Pca9685<I2C0PamiDevice>,
    overrided: [bool; PWM_MAX_EVENT_COUNT],
}

impl PWM {
    pub async fn new(mut device: Pca9685<I2C0PamiDevice>, spawner: Spawner, event: &EventSystem) {
        // 60Hz for servomotor        
        if device.set_prescale(100).await.is_err() {
            //only display error once if device is not soldered
            log::error!("No pca9685 working");
        }
        device.enable().await.ok();
        let instance = Self {
            device,
            overrided: [false; PWM_MAX_EVENT_COUNT],
        };
        spawner.spawn(start_pwm_thread(instance)).unwrap();

        event.register_receiver_callback(Some(PWM::filter_events), |evt| {
            async move {
                match evt {
                    Event::Pwm { pwm_event } => {
                        PWM::send_event(pwm_event, None);
                    }
                    Event::OverridePwm { pwm_event , override_state} => {
                        PWM::send_event(pwm_event, Some(override_state));
                    }
                    _ => {}
                }
            }
        }).await;
    }

    pub fn filter_events(evt: &Event) -> bool {
        match evt {
            Event::Pwm { .. } | Event::OverridePwm { .. } => true,
            _ => false,
        }
    } 

    fn send_event(event: PWMEvent, override_event: Option<OverrideState>) {
        match EVENT_QUEUE.try_send((event, override_event)) {
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
            let (evt, override_state) = EVENT_QUEUE.receive().await;

            if let Some(override_state) = override_state {
                match override_state {
                    OverrideState::Override => {
                        self.overrided[<PWMEvent as Into<usize>>::into(evt)] = true;
                    }
                    OverrideState::ResetOverride => {
                        self.overrided[<PWMEvent as Into<usize>>::into(evt)] = false;
                    }
                }
            }
            else if self.overrided[<PWMEvent as Into<usize>>::into(evt)] {
                continue;
            }

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
