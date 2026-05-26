//! Serial transport for the rusEFI binary protocol.
//!
//! Connects to rusEFI ECU via USB-UART or direct serial connection.
//! Uses `tokio-serial` for async serial I/O.

use crate::io::IoStream;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};

/// Serial port transport for rusEFI protocol.
pub struct SerialTransport {
    port: tokio_serial::SerialStream,
}

impl SerialTransport {
    /// Open a serial port with the given settings.
    ///
    /// # Arguments
    /// * `port_name` — Serial port name (e.g., "COM3" on Windows, "/dev/ttyUSB0" on Linux)
    /// * `baud_rate` — Baud rate (typically 115200 for rusEFI)
    pub fn open(port_name: &str, baud_rate: u32) -> std::io::Result<Self> {
        let builder = tokio_serial::new(port_name, baud_rate);
        let port = tokio_serial::SerialStream::open(&builder)?;
        Ok(Self { port })
    }

    /// Open with default rusEFI settings (115200 baud, 8N1).
    pub fn open_default(port_name: &str) -> std::io::Result<Self> {
        Self::open(port_name, 115200)
    }
}

impl AsyncRead for SerialTransport {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.get_mut().port).poll_read(cx, buf)
    }
}

impl AsyncWrite for SerialTransport {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<std::io::Result<usize>> {
        Pin::new(&mut self.get_mut().port).poll_write(cx, buf)
    }

    fn poll_flush(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.get_mut().port).poll_flush(cx)
    }

    fn poll_shutdown(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.get_mut().port).poll_shutdown(cx)
    }
}

impl IoStream for SerialTransport {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serial_transport_basic() {
        // Note: This test doesn't actually open a port
        // Real testing requires a connected ECU or loopback adapter
        
        // Test that SerialTransport implements required traits
        fn assert_io_stream<T: IoStream>() {}
        assert_io_stream::<SerialTransport>();
    }
}
