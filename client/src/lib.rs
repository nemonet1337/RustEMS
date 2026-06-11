//! High-level rusEFI ECU client.
//!
//! Wraps a [`rusefi_protocol::io::IoStream`] transport and exposes ECU
//! operations as simple async methods:
//!
//! - [`EcuClient::hello`] — identify and get firmware signature
//! - [`EcuClient::read_image`] — read the full configuration image in chunks
//! - [`EcuClient::write_chunk`] — write bytes to ECU RAM
//! - [`EcuClient::burn`] — persist ECU RAM to flash
//! - [`EcuClient::request_output_channels`] — read live-data output channels

pub mod client;
pub mod error;
pub mod image;
pub mod rdp;

pub use client::EcuClient;
pub use error::ClientError;
pub use image::ConfigImage;
pub use rdp::{RdpClient, RdpError};
