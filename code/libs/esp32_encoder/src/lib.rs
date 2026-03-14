use esp_idf_svc::{
    hal::{
        gpio::{AnyInputPin, InputPin}, 
        pcnt::{PcntUnitDriver, config::{ChannelEdgeAction, ChannelLevelAction, GlitchFilterConfig, UnitConfig}},
    }, 
    sys::EspError
};

const LOW_LIMIT: i32 = -10000;
const HIGH_LIMIT: i32 = 10000;

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
    /// * `pin_a` - Encoder A signal pin from a PCNT peripheral
    /// * `pin_b` - Encoder B signal pin from a PCNT peripheral
    /// 
    /// # Returns
    /// * `Result<Self, EspError>` - The encoder instance or an error
    pub fn new(pin_a: impl InputPin + 'd, pin_b: impl InputPin + 'd) -> Result<Self, EspError> {
        let mut unit = PcntUnitDriver::new(&UnitConfig {
            low_limit: LOW_LIMIT,
            high_limit: HIGH_LIMIT,
            accum_count: true,
            ..Default::default()
        })?;

        unit.set_glitch_filter(Some(&GlitchFilterConfig {
            max_glitch: std::time::Duration::from_nanos(10),
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
    /// 
    /// # Returns
    /// * `Result<i32, EspError>` - The current encoder count
    pub fn get_value(&self) -> Result<i32, EspError> {
        self.unit.get_count()
    }
    
    /// Reset the encoder count to zero
    /// 
    /// # Returns
    /// * `Result<(), EspError>` - Success or error
    pub fn reset(&mut self) -> Result<(), EspError> {
        self.unit.stop()?;
        self.unit.clear_count()?;
        self.unit.start()?;
        Ok(())
    }
}
