//! cancaner - Shared CAN communication protocol for cocotter robots
//!
//! This crate provides driver-agnostic CAN protocol definitions used by
//! both picotter (Embassy/STM32) and sabotter (esp-idf/ESP32).
//!
//! ## ID Format
//!
//! ```text
//! ID = 0x[DOMAIN][CMD][TARGET]
//!       │        │     │
//!       │        │     └── Target (0-F) - interpretation depends on domain
//!       │        └──────── Command (0-F) within domain
//!       └───────────────── Domain (0-7)
//! ```
//!
//! ## Domains
//!
//! | Domain | Name   | Description |
//! |--------|--------|-------------|
//! | 0x0    | SYSTEM | Ping, errors, info, reboot |
//! | 0x1    | ARM    | Servo/pump/valve commands |
//! | 0x2    | GROUND | Ground sensors |
//! | 0x3    | LOG    | Log messages (picotter → sabotter) |
//! | 0x4    | OTA    | Firmware updates |
//!
//! See PROTOCOL.md for full protocol documentation.

#![cfg_attr(not(feature = "std"), no_std)]

mod interface;
mod message;
mod protocol;
mod types;
pub mod log;
pub mod ota;

pub use interface::CanInterface;
pub use message::{ping_response, CanMessage, EncodedMessage};
pub use protocol::*;
pub use types::*;
pub use log::{LogEncoder, LogDecoder};
pub use ota::{OtaSender, OtaSenderState, OtaReceiver, OtaReceiverState, OtaStorage};
