//! Async I/O abstraction for the rusEFI binary protocol.
//!
//! [`IoStream`] is the central trait that any transport (serial, TCP, etc.)
//! must implement.  Higher-level code (e.g. the binary protocol client) works
//! exclusively against this trait, so transports are interchangeable.

use crate::packet::{self, ProtocolError};
use anyhow::Result;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// A full-duplex async byte stream plus packet-level helpers.
///
/// Blanket implementations are provided for any type that is both
/// [`AsyncRead`] + [`AsyncWrite`] + [`Unpin`] + [`Send`].
pub trait IoStream: AsyncRead + AsyncWrite + Unpin + Send {
    /// Send a pre-encoded payload as a framed packet.
    ///
    /// This is the primary way to send a command: build the payload with
    /// [`Command::to_payload`](crate::opcode::Command::to_payload), then call
    /// this method.
    fn send_payload<'a>(&'a mut self, payload: &'a [u8]) -> impl Future<Output = Result<()>> + Send + 'a
    where
        Self: Sized;

    /// Read exactly one framed packet and return the verified payload bytes.
    fn recv_packet(&mut self) -> impl Future<Output = Result<Vec<u8>>> + Send
    where
        Self: Sized;
}

use std::future::Future;

/// Provides blanket [`IoStream`] implementations for any compatible async type.
pub struct FramedStream<T: AsyncRead + AsyncWrite + Unpin + Send>(pub T);

impl<T: AsyncRead + AsyncWrite + Unpin + Send> IoStream for FramedStream<T> {
    async fn send_payload(&mut self, payload: &[u8]) -> Result<()> {
        let framed = packet::encode_packet_vec(payload)?;
        self.0.write_all(&framed).await?;
        self.0.flush().await?;
        Ok(())
    }

    async fn recv_packet(&mut self) -> Result<Vec<u8>> {
        // Read the 2-byte length header
        let mut header = [0u8; 2];
        self.0.read_exact(&mut header).await?;
        let length = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;

        if length == 0 {
            return Err(ProtocolError::EmptyPayload.into());
        }
        if length > packet::MAX_PAYLOAD_LEN {
            return Err(ProtocolError::PayloadTooLarge(length).into());
        }

        // Read payload + 4-byte CRC
        let mut rest = vec![0u8; length + 4];
        self.0.read_exact(&mut rest).await?;

        // Reconstruct the full buffer so decode_packet can verify CRC
        let mut full = Vec::with_capacity(2 + length + 4);
        full.extend_from_slice(&header);
        full.extend_from_slice(&rest);

        let payload = packet::decode_packet(&full)?;
        Ok(payload.to_vec())
    }
}

impl<T: AsyncRead + AsyncWrite + Unpin + Send> AsyncRead for FramedStream<T> {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut tokio::io::ReadBuf<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        std::pin::Pin::new(&mut self.0).poll_read(cx, buf)
    }
}

impl<T: AsyncRead + AsyncWrite + Unpin + Send> AsyncWrite for FramedStream<T> {
    fn poll_write(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &[u8],
    ) -> std::task::Poll<std::io::Result<usize>> {
        std::pin::Pin::new(&mut self.0).poll_write(cx, buf)
    }

    fn poll_flush(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        std::pin::Pin::new(&mut self.0).poll_flush(cx)
    }

    fn poll_shutdown(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        std::pin::Pin::new(&mut self.0).poll_shutdown(cx)
    }
}
