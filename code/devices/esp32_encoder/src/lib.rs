use std::sync::{atomic::{AtomicI32, Ordering}, Arc};

use esp_idf_svc::{
    hal::{
        gpio::{AnyInputPin, InputPin}, 
        pcnt::{Pcnt, PcntChannel, PcntChannelConfig, PcntControlMode, PcntCountMode, PcntDriver, PcntEvent, PcntEventType, PinIndex}, 
        peripheral::Peripheral
    }, 
    sys::EspError
};

const LOW_LIMIT: i16 = -100;
const HIGH_LIMIT: i16 = 100;

/// ESP32 Encoder driver using PCNT peripheral
/// 
/// This encoder handles quadrature signals and tracks overflow/underflow
/// to provide a full 32-bit signed counter value.
pub struct Encoder<'d> {
    unit: PcntDriver<'d>,
    approx_value: Arc<AtomicI32>,
}

impl<'d> Encoder<'d> {
    /// Create a new encoder instance
    /// 
    /// # Arguments
    /// * `pcnt` - PCNT peripheral (pcnt0, pcnt1, pcnt2, pcnt3)
    /// * `pin_a` - Encoder A signal pin
    /// * `pin_b` - Encoder B signal pin
    /// 
    /// # Returns
    /// * `Result<Self, EspError>` - The encoder instance or an error
    pub fn new<PCNT: Pcnt>(
        pcnt: impl Peripheral<P = PCNT> + 'd,
        pin_a: impl Peripheral<P = impl InputPin> + 'd,
        pin_b: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self, EspError> {
        let mut unit = PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;
        
        // Configure channel 0: pin_a as signal, pin_b as control
        unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;
        
        // Configure channel 1: pin_b as signal, pin_a as control
        unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;

        // Set filter to reduce noise
        unit.set_filter_value(1023)?;
        unit.filter_enable()?;

        let approx_value = Arc::new(AtomicI32::new(0));
        
        // Setup interrupt handler for overflow/underflow detection
        unsafe {
            let approx_value = approx_value.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                }
            })?;
        }
        
        // Enable high and low limit events
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?;
        
        // Initialize counter
        unit.counter_pause()?;
        unit.counter_clear()?;
        unit.counter_resume()?;

        Ok(Self { unit, approx_value })
    }

    /// Get the current encoder value
    /// 
    /// This returns the full 32-bit signed value, accounting for overflows.
    /// 
    /// # Returns
    /// * `Result<i32, EspError>` - The current encoder count
    pub fn get_value(&self) -> Result<i32, EspError> {
        let value = self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i32;
        Ok(value)   
    }
    
    /// Reset the encoder count to zero
    /// 
    /// # Returns
    /// * `Result<(), EspError>` - Success or error
    pub fn reset(&mut self) -> Result<(), EspError> {
        self.approx_value.store(0, Ordering::SeqCst);
        self.unit.counter_pause()?;
        self.unit.counter_clear()?;
        self.unit.counter_resume()?;
        Ok(())
    }
}