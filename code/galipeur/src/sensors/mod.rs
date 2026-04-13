use std::time::Duration;
use board_sabotter::{SabotterBoard};
use cancaner::CanMessage;
use flume::Sender;
use board_sabotter::SabotterAdc;

use crate::{can::GalipeurCan, led::LedMessage};

const BATTERY_LOW_MV: f32 = 14_830.0; // 4S LiPo discharged threshold

pub struct Sensors<B: SabotterBoard> {
    _phantom: core::marker::PhantomData<B>,
}

impl<B: SabotterBoard> Sensors<B> {
    pub fn new(board: &mut B, can: GalipeurCan<B>, led_sender: Sender<LedMessage>) {
        let battery_led_sender = led_sender.clone();
        can.add_callback(move |msg| {
            match msg {
                CanMessage::BatteryStatus { voltage_mv, modules_mask: _ } => {
                    if *voltage_mv < (BATTERY_LOW_MV as u16) {
                        battery_led_sender.send(LedMessage::LowPowerBattery).ok();
                    }
                }
                CanMessage::GroundValue { sensor, value, threshold } => {
                    //only used in debug when requested by sabotter for calibration
                    log::info!("GroundValue: sensor: {} value: {}/{}", sensor, value, threshold);
                }
                _ => {}
            }
        });

        if let Some(mut battery_adc) = board.battery_adc() {
            std::thread::spawn(move || {
                const VBATT_RL_KOHMS: f32 = 6.8;
                const VBATT_RH_KOHMS: f32 = 100.0;
                const ADC_INPUT_IMP_KOHMS: f32 = 35.5; // measured empirically (DB_12 attenuation)
                const RL_EFF_KOHMS: f32 = VBATT_RL_KOHMS * ADC_INPUT_IMP_KOHMS / (VBATT_RL_KOHMS + ADC_INPUT_IMP_KOHMS);
                loop {
                    if let Ok(raw) = battery_adc.read() {
                        let mv = raw as f32 * (1.0 + VBATT_RH_KOHMS / RL_EFF_KOHMS);
                        log::info!("Battery: {:.0} mV (raw: {})", mv, raw);
                        if mv < BATTERY_LOW_MV {
                            log::warn!("Battery low! {:.0} mV", mv);
                            led_sender.send(LedMessage::LowLogicBattery).ok();
                        }
                    }
                    std::thread::sleep(Duration::from_secs(1));
                }
            });
        }
    }
}