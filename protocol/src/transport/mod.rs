//! Transport implementations for the rusEFI binary protocol.
//!
//! - [`tcp`] — connect to a rusEFI TCP gateway (e.g. the Java console proxy)
//! - [`serial`] — USB-UART or direct serial connection (requires `serial` feature)

pub mod tcp;

#[cfg(feature = "serial")]
pub mod serial;
