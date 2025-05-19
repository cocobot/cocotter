use std::rc::Rc;

use esp_idf_svc::hal::adc::{oneshot::{AdcChannelDriver, AdcDriver}, ADC1};
use esp_idf_svc::hal::gpio;

type VbattPinType = gpio::Gpio1;
type IMotLeftType = gpio::Gpio9;
type IMotRightType = gpio::Gpio10;

pub enum PamiAdcChannel {
    VBat,
    IMotLeft,
    IMotRight,
}

pub struct PamiAdc {
    vbatt_pin: AdcChannelDriver<'static, VbattPinType, Rc<AdcDriver<'static, ADC1>>>,
    imot_left_pin: AdcChannelDriver<'static, IMotLeftType, Rc<AdcDriver<'static, ADC1>>>,
    imot_right_pin: AdcChannelDriver<'static, IMotRightType, Rc<AdcDriver<'static, ADC1>>>,
}

impl PamiAdc {
    pub fn new( vbatt_pin: AdcChannelDriver<'static, VbattPinType, Rc<AdcDriver<'static, ADC1>>>, 
                imot_left_pin: AdcChannelDriver<'static, IMotLeftType, Rc<AdcDriver<'static, ADC1>>>,
                imot_right_pin: AdcChannelDriver<'static, IMotRightType, Rc<AdcDriver<'static, ADC1>>>
               ) -> Self {
        Self {
            vbatt_pin,
            imot_left_pin,
            imot_right_pin,
        }
    }

    pub fn read(&mut self, channel: PamiAdcChannel) -> u16 {
        fn convert_to_mot_current_ma(voltage_mv : f32) -> f32 {
            const RES_OHMS : f32 = 768.0;
            const GAIN_UA_A : f32 = 1500.0;
            return ((voltage_mv * 1000.0) / RES_OHMS) * 1000.0 / GAIN_UA_A;
        }

        match channel {
            PamiAdcChannel::VBat => {
                let voltage = self.vbatt_pin.read().unwrap() as f32;
                const VBATT_RL_KOHMS : f32= 91.0;
                const VBATT_RH_KOHMS : f32= 91.0;
                const ADC_INPUT_IMP_KOHMS : f32= 500.0;
                let rl_kohms = VBATT_RL_KOHMS*ADC_INPUT_IMP_KOHMS/(VBATT_RL_KOHMS+ADC_INPUT_IMP_KOHMS);
                let battery_voltage = voltage * (1.0 + VBATT_RH_KOHMS/rl_kohms);
                battery_voltage as u16
            },
            PamiAdcChannel::IMotLeft => {
                let voltage = self.imot_left_pin.read().unwrap() as f32;
                convert_to_mot_current_ma(voltage) as u16
            },
            PamiAdcChannel::IMotRight => {
                let voltage = self.imot_right_pin.read().unwrap() as f32;
                convert_to_mot_current_ma(voltage) as u16
            },
        }
    }
}