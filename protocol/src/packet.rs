//! Packet-level framing for the rusEFI binary protocol.
//!
//! Wire format (all multi-byte integers are **big-endian**):
//!
//! ```text
//! ┌──────────────┬─────────────────────┬──────────────────────┐
//! │ u16 length   │ payload (N bytes)   │ u32 CRC32 of payload │
//! └──────────────┴─────────────────────┴──────────────────────┘
//! ```

use crc32fast::Hasher;
use thiserror::Error;

/// Overhead added by framing: 2 bytes length + 4 bytes CRC32
pub const FRAMING_OVERHEAD: usize = 6;

/// Maximum allowed payload size (matches Java `MAX_PAGE_SIZE`)
pub const MAX_PAYLOAD_LEN: usize = 32_768;

/// Errors that can occur during packet encode/decode
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ProtocolError {
    #[error("payload too large: {0} bytes (max {MAX_PAYLOAD_LEN})")]
    PayloadTooLarge(usize),

    #[error("buffer too small: need {needed} bytes, have {have}")]
    BufferTooSmall { needed: usize, have: usize },

    #[error("CRC mismatch: received 0x{received:08X}, computed 0x{computed:08X}")]
    CrcMismatch { received: u32, computed: u32 },

    #[error("empty payload is not allowed")]
    EmptyPayload,
}

/// Compute CRC32 over a byte slice (matches Java `IoHelper.getCrc32`)
pub fn crc32(data: &[u8]) -> u32 {
    let mut h = Hasher::new();
    h.update(data);
    h.finalize()
}

/// Encode `payload` into `buf`, writing the full framed packet.
///
/// Returns the total number of bytes written on success.
///
/// The caller must ensure `buf` is at least `payload.len() + FRAMING_OVERHEAD` bytes.
pub fn encode_packet(payload: &[u8], buf: &mut [u8]) -> Result<usize, ProtocolError> {
    if payload.is_empty() {
        return Err(ProtocolError::EmptyPayload);
    }
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(ProtocolError::PayloadTooLarge(payload.len()));
    }

    let total = payload.len() + FRAMING_OVERHEAD;
    if buf.len() < total {
        return Err(ProtocolError::BufferTooSmall { needed: total, have: buf.len() });
    }

    let len_u16 = payload.len() as u16;
    buf[0] = (len_u16 >> 8) as u8;
    buf[1] = len_u16 as u8;

    buf[2..2 + payload.len()].copy_from_slice(payload);

    let checksum = crc32(payload);
    let crc_offset = 2 + payload.len();
    buf[crc_offset]     = (checksum >> 24) as u8;
    buf[crc_offset + 1] = (checksum >> 16) as u8;
    buf[crc_offset + 2] = (checksum >> 8)  as u8;
    buf[crc_offset + 3] =  checksum        as u8;

    Ok(total)
}

/// Encode `payload` into a freshly allocated `Vec<u8>`.
pub fn encode_packet_vec(payload: &[u8]) -> Result<Vec<u8>, ProtocolError> {
    let mut buf = vec![0u8; payload.len() + FRAMING_OVERHEAD];
    encode_packet(payload, &mut buf)?;
    Ok(buf)
}

/// Decode a complete framed packet from `buf`.
///
/// `buf` must contain at least `2 + length + 4` bytes starting at offset 0.
/// Returns a slice pointing into `buf` that is the verified payload.
pub fn decode_packet(buf: &[u8]) -> Result<&[u8], ProtocolError> {
    if buf.len() < FRAMING_OVERHEAD {
        return Err(ProtocolError::BufferTooSmall { needed: FRAMING_OVERHEAD, have: buf.len() });
    }

    let length = (u16::from(buf[0]) << 8 | u16::from(buf[1])) as usize;

    if length == 0 {
        return Err(ProtocolError::EmptyPayload);
    }

    let needed = length + FRAMING_OVERHEAD;
    if buf.len() < needed {
        return Err(ProtocolError::BufferTooSmall { needed, have: buf.len() });
    }

    let payload = &buf[2..2 + length];

    let crc_offset = 2 + length;
    let received = u32::from_be_bytes([
        buf[crc_offset],
        buf[crc_offset + 1],
        buf[crc_offset + 2],
        buf[crc_offset + 3],
    ]);
    let computed = crc32(payload);

    if received != computed {
        return Err(ProtocolError::CrcMismatch { received, computed });
    }

    Ok(payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_simple() {
        let payload = b"\x53hello";
        let encoded = encode_packet_vec(payload).unwrap();
        assert_eq!(encoded.len(), payload.len() + FRAMING_OVERHEAD);
        let decoded = decode_packet(&encoded).unwrap();
        assert_eq!(decoded, payload);
    }

    #[test]
    fn crc_mismatch_detected() {
        let payload = b"\x01\x02\x03";
        let mut encoded = encode_packet_vec(payload).unwrap();
        // Corrupt the last byte of CRC
        let last = encoded.len() - 1;
        encoded[last] ^= 0xFF;
        assert!(matches!(decode_packet(&encoded), Err(ProtocolError::CrcMismatch { .. })));
    }

    #[test]
    fn empty_payload_rejected() {
        assert_eq!(encode_packet_vec(&[]), Err(ProtocolError::EmptyPayload));
    }

    #[test]
    fn buffer_too_small_detected() {
        let payload = b"\x53hello";
        let mut buf = vec![0u8; 3]; // too small
        assert!(matches!(
            encode_packet(payload, &mut buf),
            Err(ProtocolError::BufferTooSmall { .. })
        ));
    }

    #[test]
    fn crc32_matches_java() {
        // CRC32 of b"hello" == 0x3610a686 (standard CRC32 / zlib polynomial)
        assert_eq!(crc32(b"hello"), 0x3610_a686);
    }

    #[test]
    fn length_field_is_big_endian() {
        let payload = vec![0x42u8; 256];
        let encoded = encode_packet_vec(&payload).unwrap();
        assert_eq!(encoded[0], 0x01); // high byte of 256
        assert_eq!(encoded[1], 0x00); // low byte of 256
    }

    #[test]
    fn decode_truncated_buffer_error() {
        let payload = b"\x01\x02\x03\x04";
        let encoded = encode_packet_vec(payload).unwrap();
        // Give only the header, no payload
        assert!(matches!(
            decode_packet(&encoded[..2]),
            Err(ProtocolError::BufferTooSmall { .. })
        ));
    }
}
