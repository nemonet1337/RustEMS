//! rusEFI Binary Protocol
//!
//! Implements the binary framing protocol used between the rusEFI ECU firmware
//! and host-side tooling (Java console, TunerStudio, etc.).
//!
//! # Packet format
//!
//! Every packet on the wire is:
//!
//! ```text
//! [u16 payload_length (big-endian)]
//! [payload_length bytes of payload]
//! [u32 CRC32 of payload (big-endian)]
//! ```
//!
//! A *command* packet has the opcode as the first byte of the payload,
//! optionally followed by command-specific data.
//!
//! # Modules
//!
//! - [`packet`] — low-level encode/decode
//! - [`opcode`] — opcode constants and typed commands
//! - [`io`]     — [`IoStream`](io::IoStream) async I/O trait
//! - [`transport`] — TCP (and optionally serial) transports

pub mod io;
pub mod opcode;
pub mod packet;
pub mod transport;

pub use packet::{decode_packet, encode_packet, ProtocolError};
