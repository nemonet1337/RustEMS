//! Frame layer: the unit that COBS wraps on stream transports and that maps to
//! one GATT characteristic write/notify on BLE.
//!
//! Raw frame layout (before COBS), all multi-byte integers **little-endian**:
//!
//! ```text
//! ┌──────┬───────┬────────┬────────┬──────────────┬────────┐
//! │ VER  │ FLAGS │ SEQ    │ LEN    │ PAYLOAD      │ CRC16  │
//! │ u8   │ u8    │ u16    │ u16    │ LEN bytes    │ u16    │
//! └──────┴───────┴────────┴────────┴──────────────┴────────┘
//! ```
//!
//! `CRC16` (CRC-16/CCITT-FALSE) covers `VER..=PAYLOAD` (everything before the
//! CRC itself). On stream transports the encoded frame is COBS-wrapped and a
//! single `0x00` delimiter is appended by [`encode_frame`].

use crate::cobs::{self, CobsError};
use crate::crc16::crc16_ccitt;

/// Framing-layer version (see `docs/api/02-transport-and-framing.md`).
pub const VERSION: u8 = 0x01;

/// Fixed header size: VER + FLAGS + SEQ(2) + LEN(2).
pub const HEADER_LEN: usize = 6;

/// Trailing CRC16 size.
pub const CRC_LEN: usize = 2;

/// Maximum payload carried by a single frame. Larger messages must be split
/// across frames using the fragmentation flags.
pub const MAX_PAYLOAD_LEN: usize = 512;

/// Maximum size of a raw (pre-COBS) frame.
pub const MAX_RAW_FRAME_LEN: usize = HEADER_LEN + MAX_PAYLOAD_LEN + CRC_LEN;

/// Maximum size of an encoded frame including the trailing `0x00` delimiter.
pub const MAX_ENCODED_FRAME_LEN: usize = cobs::max_encoded_len(MAX_RAW_FRAME_LEN) + 1;

/// Frame delimiter used on stream transports.
pub const DELIMITER: u8 = 0x00;

/// Frame flag bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Flags(pub u8);

impl Flags {
    /// Sender wants a correlated response (matched on [`FrameHeader::seq`]).
    pub const RESPONSE_REQUESTED: u8 = 0b0000_0001;
    /// This frame is one fragment of a larger message.
    pub const FRAGMENT: u8 = 0b0000_0010;
    /// This is the last fragment of a fragmented message.
    pub const LAST_FRAGMENT: u8 = 0b0000_0100;

    /// Empty flag set.
    pub const fn none() -> Self {
        Flags(0)
    }

    /// True if `bit` is set.
    pub const fn has(self, bit: u8) -> bool {
        self.0 & bit != 0
    }

    /// Return a copy with `bit` set.
    pub const fn with(self, bit: u8) -> Self {
        Flags(self.0 | bit)
    }
}

/// Parsed frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    /// Framing-layer version.
    pub version: u8,
    /// Flag bits.
    pub flags: Flags,
    /// Sequence number for request/response correlation and stream ordering.
    pub seq: u16,
}

impl FrameHeader {
    /// Construct a header for the current [`VERSION`].
    pub const fn new(flags: Flags, seq: u16) -> Self {
        FrameHeader {
            version: VERSION,
            flags,
            seq,
        }
    }
}

/// Errors from frame encode/decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Payload exceeds [`MAX_PAYLOAD_LEN`].
    PayloadTooLarge,
    /// Output buffer too small for the encoded frame.
    OutputTooSmall,
    /// Frame is shorter than the minimum header + CRC.
    TooShort,
    /// Declared `LEN` is inconsistent with the decoded byte count.
    LengthMismatch,
    /// CRC check failed.
    CrcMismatch,
    /// Unsupported framing version.
    UnsupportedVersion(u8),
    /// COBS decode failure.
    Cobs(CobsError),
}

impl From<CobsError> for FrameError {
    fn from(e: CobsError) -> Self {
        FrameError::Cobs(e)
    }
}

/// Encode a frame for a stream transport: build the raw frame, COBS-encode it
/// into `out`, and append the `0x00` delimiter. Returns the bytes written.
///
/// For packet transports (BLE GATT) use [`encode_frame_raw`] instead and let
/// the link layer provide framing.
pub fn encode_frame(
    header: &FrameHeader,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    let mut raw = [0u8; MAX_RAW_FRAME_LEN];
    let raw_len = encode_frame_raw(header, payload, &mut raw)?;

    let n = cobs::encode(&raw[..raw_len], out)?;
    // Append the delimiter.
    *out.get_mut(n).ok_or(FrameError::OutputTooSmall)? = DELIMITER;
    Ok(n + 1)
}

/// Build the raw (pre-COBS) frame into `out`. Returns the bytes written.
pub fn encode_frame_raw(
    header: &FrameHeader,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(FrameError::PayloadTooLarge);
    }
    let total = HEADER_LEN + payload.len() + CRC_LEN;
    if out.len() < total {
        return Err(FrameError::OutputTooSmall);
    }

    let len = payload.len() as u16;
    out[0] = header.version;
    out[1] = header.flags.0;
    out[2] = (header.seq & 0xFF) as u8;
    out[3] = (header.seq >> 8) as u8;
    out[4] = (len & 0xFF) as u8;
    out[5] = (len >> 8) as u8;
    out[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);

    let crc = crc16_ccitt(&out[..HEADER_LEN + payload.len()]);
    let crc_at = HEADER_LEN + payload.len();
    out[crc_at] = (crc & 0xFF) as u8;
    out[crc_at + 1] = (crc >> 8) as u8;

    Ok(total)
}

/// Decode a COBS-encoded frame (delimiter already stripped) into `scratch`.
///
/// Returns the parsed header and a slice of `scratch` holding the payload.
pub fn decode_frame<'s>(
    encoded: &[u8],
    scratch: &'s mut [u8],
) -> Result<(FrameHeader, &'s [u8]), FrameError> {
    let raw_len = cobs::decode(encoded, scratch)?;
    decode_frame_raw(&scratch[..raw_len])
}

/// Parse a raw (already COBS-decoded) frame, verify its CRC, and return the
/// header plus payload slice.
pub fn decode_frame_raw(raw: &[u8]) -> Result<(FrameHeader, &[u8]), FrameError> {
    if raw.len() < HEADER_LEN + CRC_LEN {
        return Err(FrameError::TooShort);
    }
    let version = raw[0];
    if version != VERSION {
        return Err(FrameError::UnsupportedVersion(version));
    }
    let flags = Flags(raw[1]);
    let seq = u16::from(raw[2]) | (u16::from(raw[3]) << 8);
    let len = (u16::from(raw[4]) | (u16::from(raw[5]) << 8)) as usize;

    let expected_total = HEADER_LEN + len + CRC_LEN;
    if raw.len() != expected_total {
        return Err(FrameError::LengthMismatch);
    }

    let crc_at = HEADER_LEN + len;
    let received = u16::from(raw[crc_at]) | (u16::from(raw[crc_at + 1]) << 8);
    let computed = crc16_ccitt(&raw[..crc_at]);
    if received != computed {
        return Err(FrameError::CrcMismatch);
    }

    let payload = &raw[HEADER_LEN..crc_at];
    Ok((FrameHeader { version, flags, seq }, payload))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_stream() {
        let header = FrameHeader::new(Flags::none().with(Flags::RESPONSE_REQUESTED), 0x1234);
        let payload = b"\x00\x01\x02hello\x00world";

        let mut out = [0u8; MAX_ENCODED_FRAME_LEN];
        let n = encode_frame(&header, payload, &mut out).unwrap();

        // Last byte is the delimiter; nothing before it is a zero.
        assert_eq!(out[n - 1], DELIMITER);
        assert!(!out[..n - 1].contains(&0));

        let mut scratch = [0u8; MAX_RAW_FRAME_LEN];
        let (h, p) = decode_frame(&out[..n - 1], &mut scratch).unwrap();
        assert_eq!(h, header);
        assert_eq!(p, payload);
    }

    #[test]
    fn round_trip_raw_empty_payload() {
        let header = FrameHeader::new(Flags::none(), 7);
        let mut raw = [0u8; MAX_RAW_FRAME_LEN];
        let n = encode_frame_raw(&header, &[], &mut raw).unwrap();
        let (h, p) = decode_frame_raw(&raw[..n]).unwrap();
        assert_eq!(h, header);
        assert_eq!(p, &[] as &[u8]);
    }

    #[test]
    fn fragmentation_flags() {
        let f = Flags::none().with(Flags::FRAGMENT).with(Flags::LAST_FRAGMENT);
        assert!(f.has(Flags::FRAGMENT));
        assert!(f.has(Flags::LAST_FRAGMENT));
        assert!(!f.has(Flags::RESPONSE_REQUESTED));
    }

    #[test]
    fn crc_mismatch_detected() {
        let header = FrameHeader::new(Flags::none(), 1);
        let mut raw = [0u8; MAX_RAW_FRAME_LEN];
        let n = encode_frame_raw(&header, b"abc", &mut raw).unwrap();
        raw[HEADER_LEN] ^= 0xFF; // corrupt payload
        assert_eq!(decode_frame_raw(&raw[..n]), Err(FrameError::CrcMismatch));
    }

    #[test]
    fn length_mismatch_detected() {
        let header = FrameHeader::new(Flags::none(), 1);
        let mut raw = [0u8; MAX_RAW_FRAME_LEN];
        let n = encode_frame_raw(&header, b"abcd", &mut raw).unwrap();
        // Shrink the declared length without fixing the buffer size.
        raw[4] = 2;
        assert_eq!(decode_frame_raw(&raw[..n]), Err(FrameError::LengthMismatch));
    }

    #[test]
    fn payload_too_large() {
        let header = FrameHeader::new(Flags::none(), 0);
        let big = [0u8; MAX_PAYLOAD_LEN + 1];
        let mut out = [0u8; MAX_ENCODED_FRAME_LEN];
        assert_eq!(
            encode_frame(&header, &big, &mut out),
            Err(FrameError::PayloadTooLarge)
        );
    }

    #[test]
    fn unsupported_version() {
        let header = FrameHeader::new(Flags::none(), 0);
        let mut raw = [0u8; MAX_RAW_FRAME_LEN];
        let n = encode_frame_raw(&header, b"x", &mut raw).unwrap();
        raw[0] = 0xEE;
        // CRC will now also be wrong, but version is checked first.
        assert_eq!(
            decode_frame_raw(&raw[..n]),
            Err(FrameError::UnsupportedVersion(0xEE))
        );
    }

    #[test]
    fn too_short() {
        assert_eq!(decode_frame_raw(&[1, 2, 3]), Err(FrameError::TooShort));
    }

    #[test]
    fn max_payload_fits_encoded_buffer() {
        let header = FrameHeader::new(Flags::none(), 0xFFFF);
        let payload = [0xAAu8; MAX_PAYLOAD_LEN];
        let mut out = [0u8; MAX_ENCODED_FRAME_LEN];
        let n = encode_frame(&header, &payload, &mut out).unwrap();
        assert!(n <= MAX_ENCODED_FRAME_LEN);

        let mut scratch = [0u8; MAX_RAW_FRAME_LEN];
        let (_, p) = decode_frame(&out[..n - 1], &mut scratch).unwrap();
        assert_eq!(p, &payload);
    }
}
