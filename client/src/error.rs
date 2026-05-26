//! Client-level error types.

use rusefi_protocol::packet::ProtocolError;
use thiserror::Error;

/// Errors that can occur during ECU client operations.
#[derive(Debug, Error)]
pub enum ClientError {
    #[error("protocol framing error: {0}")]
    Protocol(#[from] ProtocolError),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("ECU returned error response code 0x{code:02X}")]
    EcuError { code: u8 },

    #[error("unexpected response length: expected {expected}, got {got}")]
    UnexpectedLength { expected: usize, got: usize },

    #[error("empty response from ECU")]
    EmptyResponse,

    #[error("read timeout: image read incomplete after {bytes_read} of {total} bytes")]
    ReadTimeout { bytes_read: usize, total: usize },

    #[error("anyhow: {0}")]
    Anyhow(#[from] anyhow::Error),
}
