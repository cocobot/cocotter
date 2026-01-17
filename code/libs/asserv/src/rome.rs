use rome::Message;

/// Support for common asserv ROME messages
pub trait AsservRome {
    /// ROME message event handler, return true if message has been processed
    fn on_rome_message(&mut self, message: &Message) -> bool;
    /// Create an `AsservTmStatus` message from current asserv state
    fn asserv_tm_status(&self) -> Message;
    /// Create an `AsservTmVelocity` message from current asserv state (optional)
    fn asserv_tm_velocity(&self) -> Option<Message>;
}

