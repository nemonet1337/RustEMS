//! RustEMS Device Protocol (RDP) — core wire layer.
//!
//! This crate implements the transport-independent core of the new RustEMS
//! device API that replaces the legacy TunerStudio-compatible protocol. It is
//! shared by both the device firmware (`no_std`) and host-side tooling.
//!
//! See `docs/api/` for the full design:
//! - `02-transport-and-framing.md` — framing, COBS, CRC16, fragmentation
//! - `03-message-protocol.md` — message kinds, opcodes, error codes
//! - `04-parameter-model.md` — value types
//!
//! # Layers
//!
//! ```text
//! ┌───────────────────────────────────────────────┐
//! │ message: KIND + OP + BODY                       │  this crate
//! ├───────────────────────────────────────────────┤
//! │ frame:   VER FLAGS SEQ LEN PAYLOAD CRC16        │  this crate
//! ├───────────────────────────────────────────────┤
//! │ cobs:    self-synchronizing 0x00-delimited      │  this crate
//! └───────────────────────────────────────────────┘
//! ```
//!
//! The body codec (CBOR for control messages, packed binary for telemetry) and
//! the device-side request handlers are layered on top of this crate.
//!
//! All public APIs operate on caller-provided buffers — no heap allocation, no
//! `unsafe`, no panics.

#![cfg_attr(not(test), no_std)]

pub mod cobs;
pub mod crc16;
pub mod frame;
pub mod message;
pub mod defrag;
pub mod cbor;

// Re-export so device/host code uses the same minicbor version as the bodies.
pub use minicbor;

pub use crc16::crc16_ccitt;
pub use frame::{
    decode_frame, encode_frame, encode_message, Flags, FrameError, FrameHeader,
    MAX_ENCODED_FRAME_LEN, MAX_PAYLOAD_LEN, MAX_RAW_FRAME_LEN, VERSION,
};
pub use message::{ErrorCode, Kind, MessageHeader, MessageError, ValueType};
pub use defrag::Defragmenter;
