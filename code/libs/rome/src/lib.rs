pub mod message;
mod deserialize;
mod serialize;

use deserialize::Reader;
use serialize::Serialize;

pub use message::{Message, MessageId, params};


#[derive(Debug)]
pub enum DecodeError {
    /// Not enough data
    EndOfData,
    /// Unknown message ID
    UnknownMessage(u8),
    /// Some data has not been read, message is probably corrupted
    UnparsedData(u8, usize),
    /// Invalid choice value
    BadChoiceValue(u8),
}


impl Message {
    pub fn encode(&self) -> Box<[u8]> {
        let buffer_size = self.serialized_size();
        let mut buffer = Vec::with_capacity(buffer_size);
        self.serialize(&mut buffer);
        assert_eq!(buffer.len(), buffer_size);
        buffer.into_boxed_slice()
    }

    pub fn decode(mut data: &[u8]) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 1];
        data.read(&mut buffer)?;
        let message = Self::deserialize_with_id(buffer[0], &mut data)?;
        if !data.is_empty() {
            Err(DecodeError::UnparsedData(buffer[0], data.len()))
        } else {
            Ok(message)
        }
    }
}


/// Macros to log using a ROME logger
mod macros {
    #[macro_export]
    #[clippy::format_args]
    macro_rules! debug {
        ($logger:expr, $($arg:tt)*) => { $crate::_log!($logger, 'D', $($arg)*) }
    }

    #[macro_export]
    #[clippy::format_args]
    macro_rules! info {
        ($logger:expr, $($arg:tt)*) => { $crate::_log!($logger, 'I', $($arg)*) }
    }

    #[macro_export]
    #[clippy::format_args]
    macro_rules! warn {
        ($logger:expr, $($arg:tt)*) => { $crate::_log!($logger, 'W', $($arg)*) }
    }

    #[macro_export]
    #[clippy::format_args]
    macro_rules! error {
        ($logger:expr, $($arg:tt)*) => { $crate::_log!($logger, 'E', $($arg)*) }
    }

    //TODO For now, assume `$logger` is a sender and don't log the time
    #[macro_export]
    macro_rules! _log {
        ($logger:expr, $level:literal, $fmt:literal) => {
            $logger.send(format!(concat!("{} {} ", $fmt), 0, $level)).unwrap();
        };
        ($logger:expr, $level:literal, $fmt:literal, $($arg:tt)*) => {
            $logger.send(format!(concat!("{} {} ", $fmt), 0, $level, $($arg)*)).unwrap();
        };
    }
}

