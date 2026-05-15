#[derive(Debug)]
pub enum StrategyError {
    OpponentDetected,
    SensorUnavailable,
    StupidOrder,
    PreEndOfMatch,
    EndOfMatch,
}