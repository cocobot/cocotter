pub mod message;
mod deserialize;
mod serialize;

use deserialize::Reader;
use serialize::Serialize;

pub use message::{Message, MessageId};


#[derive(Debug)]
pub enum DecodeError {
    /// Not enough data
    EndOfData,
    /// Unknown message ID
    UnknownMessage(u8),
    /// Some data has not been read, message is probably corrupted
    UnparsedData(Message, usize),
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
            Err(DecodeError::UnparsedData(message, data.len()))
        } else {
            Ok(message)
        }
    }
}

