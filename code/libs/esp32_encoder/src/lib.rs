use esp_idf_svc::{
    hal::{
        gpio::{AnyInputPin, InputPin},        
        pcnt::{PcntUnitDriver, config::{ChannelEdgeAction, ChannelLevelAction, GlitchFilterConfig, UnitConfig}},
    }, 
    sys::EspError
};

/// ESP32 Encoder driver using PCNT peripheral
/// 
/// This encoder handles quadrature signals and tracks overflow/underflow
/// to provide a full 32-bit signed counter value.
pub struct Encoder<'d> {
    unit: PcntUnitDriver<'d>,
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
    pub fn new(
        pin_a: impl InputPin + 'd,
        pin_b: impl InputPin + 'd,
    ) -> Result<Self, EspError> {
        const LOW_LIMIT: i32 = -10000;
        const HIGH_LIMIT: i32 = 10000;
        const FILTER_VALUE: u64 = 10;

         let mut unit = PcntUnitDriver::new(&UnitConfig {
            low_limit: LOW_LIMIT,
            high_limit: HIGH_LIMIT,
            accum_count: true,
            ..Default::default()
        })?;

        unit.set_glitch_filter(Some(&GlitchFilterConfig {
            max_glitch: std::time::Duration::from_nanos(FILTER_VALUE),
            ..Default::default()
        }))?;

        // SAFETY: copied from example code
        let (dup_pin_a, dup_pin_b) = unsafe {
            (
                AnyInputPin::steal(pin_a.pin()),
                AnyInputPin::steal(pin_b.pin()),
            )
        };

        unit.add_channel(Some(pin_a), Some(pin_b), &Default::default())?
            .set_edge_action(ChannelEdgeAction::Decrease, ChannelEdgeAction::Increase)?
            .set_level_action(ChannelLevelAction::Keep, ChannelLevelAction::Inverse)?;

        unit.add_channel(Some(dup_pin_b), Some(dup_pin_a), &Default::default())?
            .set_edge_action(ChannelEdgeAction::Increase, ChannelEdgeAction::Decrease)?
            .set_level_action(ChannelLevelAction::Keep, ChannelLevelAction::Inverse)?;

        unit.enable()?;
        unit.add_watch_points_and_clear([LOW_LIMIT, HIGH_LIMIT])?;

        unit.stop()?;
        unit.clear_count()?;
        unit.start()?;

        Ok(Self { unit })
    }

    /// Get the current encoder value
    /// 
    /// This returns the full 32-bit signed value, accounting for overflows.
    pub fn get_value(&self) -> Result<i32, EspError> {
        self.unit.get_count()
    }
    
    /// Reset the encoder count to zero
    pub fn reset(&mut self) -> Result<(), EspError> {
        self.unit.stop()?;
        self.unit.clear_count()?;
        self.unit.start()?;
        Ok(())
    }
}
