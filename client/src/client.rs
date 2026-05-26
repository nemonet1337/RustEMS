//! High-level ECU client operations.
//!
//! [`EcuClient`] wraps a [`FramedStream`] and exposes typed async methods for
//! every operation the Java `BinaryProtocol` class supports.

use crate::error::ClientError;
use crate::image::ConfigImage;
use rusefi_protocol::io::{FramedStream, IoStream};
use rusefi_protocol::opcode::{
    Command, TS_RESPONSE_OK,
};
use tokio::io::{AsyncRead, AsyncWrite};
use tracing::{debug, info};

/// Default chunk size used when no `blocking_factor` is known.
pub const DEFAULT_BLOCKING_FACTOR: usize = 128;

/// High-level rusEFI ECU client.
///
/// Owns a [`FramedStream`] wrapping any async transport (TCP, serial…).
/// All methods are `async` and return [`ClientError`] on failure.
pub struct EcuClient<T: AsyncRead + AsyncWrite + Unpin + Send> {
    stream: FramedStream<T>,
    /// Firmware signature returned by the ECU (set after [`Self::hello`])
    pub signature: Option<String>,
    /// Chunk size for multi-part reads/writes (bytes). Default: 128.
    pub blocking_factor: usize,
}

impl<T: AsyncRead + AsyncWrite + Unpin + Send> EcuClient<T> {
    /// Wrap an existing async stream.
    pub fn new(inner: T) -> Self {
        Self {
            stream: FramedStream(inner),
            signature: None,
            blocking_factor: DEFAULT_BLOCKING_FACTOR,
        }
    }

    /// Set the chunk size used for chunked reads/writes.
    pub fn with_blocking_factor(mut self, factor: usize) -> Self {
        self.blocking_factor = factor;
        self
    }

    // -----------------------------------------------------------------------
    // Low-level helpers
    // -----------------------------------------------------------------------

    /// Send a command payload and receive the response payload.
    async fn execute(&mut self, cmd: &Command) -> Result<Vec<u8>, ClientError> {
        let payload = cmd.to_payload();
        self.stream.send_payload(&payload).await?;
        let response = self.stream.recv_packet().await?;
        if response.is_empty() {
            return Err(ClientError::EmptyResponse);
        }
        Ok(response)
    }

    /// Like [`Self::execute`] but also checks that the first byte is `TS_RESPONSE_OK`.
    async fn execute_ok(&mut self, cmd: &Command) -> Result<Vec<u8>, ClientError> {
        let response = self.execute(cmd).await?;
        if response[0] != TS_RESPONSE_OK {
            return Err(ClientError::EcuError { code: response[0] });
        }
        Ok(response)
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Send a Hello (`S`) command and return the raw response bytes.
    ///
    /// The ECU replies with an OK byte followed by the firmware signature string.
    /// The signature is cached in [`Self::signature`].
    pub async fn hello(&mut self) -> Result<String, ClientError> {
        let response = self.execute_ok(&Command::Hello).await?;
        // Bytes after the OK code are the signature string
        let sig = String::from_utf8_lossy(&response[1..]).trim().to_string();
        info!("ECU signature: {}", sig);
        self.signature = Some(sig.clone());
        Ok(sig)
    }

    /// Request the firmware version string (`V` command).
    pub async fn get_firmware_version(&mut self) -> Result<String, ClientError> {
        let response = self.execute_ok(&Command::GetFirmwareVersion).await?;
        let ver = String::from_utf8_lossy(&response[1..]).trim().to_string();
        Ok(ver)
    }

    /// Read the full ECU configuration image of `total_size` bytes.
    ///
    /// Uses chunked `R` commands of at most `blocking_factor` bytes each.
    /// The page index is always 0 (primary config page).
    pub async fn read_image(
        &mut self,
        total_size: usize,
        signature: impl Into<String>,
    ) -> Result<ConfigImage, ClientError> {
        let mut image = ConfigImage::new(total_size, signature);
        let blocking_factor = self.blocking_factor;
        let mut offset = 0usize;

        while offset < total_size {
            let chunk = (total_size - offset).min(blocking_factor);
            debug!("read_image offset={offset} chunk={chunk}");

            let cmd = Command::ReadPage {
                page: 0,
                offset: offset as u16,
                length: chunk as u16,
            };
            let response = self.execute_ok(&cmd).await?;

            // Response is [OK_byte, data...]
            let expected_len = chunk + 1;
            if response.len() != expected_len {
                return Err(ClientError::UnexpectedLength {
                    expected: expected_len,
                    got: response.len(),
                });
            }

            image.write_range(offset, &response[1..]);
            offset += chunk;
        }

        info!("read_image complete: {} bytes", total_size);
        Ok(image)
    }

    /// Write a byte slice to the ECU at the given page/offset.
    ///
    /// Large writes are automatically split into `blocking_factor`-sized chunks.
    pub async fn write_chunk(
        &mut self,
        page: u16,
        offset: usize,
        data: &[u8],
    ) -> Result<(), ClientError> {
        let blocking_factor = self.blocking_factor;
        let mut pos = 0usize;

        while pos < data.len() {
            let chunk_end = (pos + blocking_factor).min(data.len());
            let chunk = &data[pos..chunk_end];
            debug!("write_chunk page={page} offset={} len={}", offset + pos, chunk.len());

            let cmd = Command::WriteChunk {
                page,
                offset: (offset + pos) as u16,
                data: chunk.to_vec(),
            };
            self.execute_ok(&cmd).await?;
            pos = chunk_end;
        }

        Ok(())
    }

    /// Persist ECU RAM configuration to flash (`B` command).
    pub async fn burn(&mut self) -> Result<(), ClientError> {
        info!("Burning configuration to ECU flash");
        self.execute_ok(&Command::Burn).await?;
        Ok(())
    }

    /// Read live output channels (`O` command).
    ///
    /// `total_size` is the `ochBlockSize` from the INI file.
    /// Returns the raw channel bytes (without the leading OK byte).
    pub async fn request_output_channels(
        &mut self,
        total_size: usize,
    ) -> Result<Vec<u8>, ClientError> {
        let blocking_factor = self.blocking_factor;
        let mut buf = vec![0u8; total_size];
        let mut pos = 0usize;

        while pos < total_size {
            let chunk = (total_size - pos).min(blocking_factor);
            debug!("output_channels offset={pos} chunk={chunk}");

            let cmd = Command::OutputChannels {
                offset: pos as u16,
                length: chunk as u16,
            };
            let response = self.execute_ok(&cmd).await?;

            let expected = chunk + 1;
            if response.len() != expected {
                return Err(ClientError::UnexpectedLength {
                    expected,
                    got: response.len(),
                });
            }

            buf[pos..pos + chunk].copy_from_slice(&response[1..]);
            pos += chunk;
        }

        Ok(buf)
    }

    /// Send a CRC check command for a page range and return the ECU's CRC32.
    pub async fn crc_check(
        &mut self,
        page: u16,
        offset: u16,
        length: u16,
    ) -> Result<u32, ClientError> {
        let cmd = Command::CrcCheck { page, offset, length };
        let response = self.execute_ok(&cmd).await?;

        // Response: [OK, crc_b0, crc_b1, crc_b2, crc_b3]
        if response.len() < 5 {
            return Err(ClientError::UnexpectedLength { expected: 5, got: response.len() });
        }
        let crc = u32::from_be_bytes([response[1], response[2], response[3], response[4]]);
        Ok(crc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusefi_protocol::packet::encode_packet_vec;
    use tokio::io::duplex;

    /// Build a fake ECU response packet: [OK, ...data]
    fn ok_response(data: &[u8]) -> Vec<u8> {
        let mut payload = vec![TS_RESPONSE_OK];
        payload.extend_from_slice(data);
        encode_packet_vec(&payload).unwrap()
    }

    #[tokio::test]
    async fn test_hello() {
        let (client_io, mut server_io) = duplex(4096);

        tokio::spawn(async move {
            // Consume the Hello command packet
            let mut header = [0u8; 2];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut header).await.unwrap();
            let len = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;
            let mut rest = vec![0u8; len + 4];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut rest).await.unwrap();

            // Reply: OK + signature string
            let resp = ok_response(b"rusEFI v1.0");
            tokio::io::AsyncWriteExt::write_all(&mut server_io, &resp).await.unwrap();
        });

        let mut client = EcuClient::new(client_io);
        let sig = client.hello().await.unwrap();
        assert_eq!(sig, "rusEFI v1.0");
        assert_eq!(client.signature.as_deref(), Some("rusEFI v1.0"));
    }

    #[tokio::test]
    async fn test_read_image_single_chunk() {
        let (client_io, mut server_io) = duplex(4096);
        let expected_data = vec![0xABu8; 32];

        let expected_clone = expected_data.clone();
        tokio::spawn(async move {
            // Drain the ReadPage command
            let mut header = [0u8; 2];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut header).await.unwrap();
            let len = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;
            let mut rest = vec![0u8; len + 4];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut rest).await.unwrap();

            let resp = ok_response(&expected_clone);
            tokio::io::AsyncWriteExt::write_all(&mut server_io, &resp).await.unwrap();
        });

        let mut client = EcuClient::new(client_io).with_blocking_factor(256);
        let image = client.read_image(32, "test-sig").await.unwrap();
        assert_eq!(image.as_bytes(), expected_data.as_slice());
    }

    #[tokio::test]
    async fn test_burn() {
        let (client_io, mut server_io) = duplex(4096);

        tokio::spawn(async move {
            let mut header = [0u8; 2];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut header).await.unwrap();
            let len = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;
            let mut rest = vec![0u8; len + 4];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut rest).await.unwrap();

            let resp = ok_response(&[]);
            tokio::io::AsyncWriteExt::write_all(&mut server_io, &resp).await.unwrap();
        });

        let mut client = EcuClient::new(client_io);
        client.burn().await.unwrap();
    }

    #[tokio::test]
    async fn test_ecu_error_propagated() {
        let (client_io, mut server_io) = duplex(4096);

        tokio::spawn(async move {
            let mut header = [0u8; 2];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut header).await.unwrap();
            let len = (u16::from(header[0]) << 8 | u16::from(header[1])) as usize;
            let mut rest = vec![0u8; len + 4];
            tokio::io::AsyncReadExt::read_exact(&mut server_io, &mut rest).await.unwrap();

            // Reply with error code 0x01
            let resp = encode_packet_vec(&[0x01]).unwrap();
            tokio::io::AsyncWriteExt::write_all(&mut server_io, &resp).await.unwrap();
        });

        let mut client = EcuClient::new(client_io);
        let result = client.hello().await;
        assert!(matches!(result, Err(ClientError::EcuError { code: 0x01 })));
    }
}
