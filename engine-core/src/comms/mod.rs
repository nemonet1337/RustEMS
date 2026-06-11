//! Device-side (no_std) TunerStudio binary protocol responder.
//!
//! This is the firmware counterpart to the host-side `rusefi-protocol` crate.
//! It frames/unframes packets and answers the TunerStudio commands needed for
//! PC tuning over a serial/UART link, without any heap allocation.
//!
//! Wire format (all multi-byte integers big-endian):
//!
//! ```text
//! [u16 payload_length][payload bytes][u32 CRC32 of payload]
//! ```
//!
//! A response payload is `[response_code][data...]`, where the code is
//! [`TS_RESPONSE_OK`] on success.

pub mod output;
pub use output::{OutputChannels, OUTPUT_CHANNELS_LEN};

pub mod control;
pub mod faults;
pub mod rdp;
pub mod telemetry;
pub use rdp::{
    build_request, build_request_empty, DeviceIdentity, RdpActions, RdpContext, RdpServer,
};

/// Framing overhead: 2-byte length prefix + 4-byte CRC32 suffix.
pub const FRAMING_OVERHEAD: usize = 6;

// ── Response codes ───────────────────────────────────────────────────────────
/// Command understood and executed.
pub const TS_RESPONSE_OK: u8 = 0x00;
/// Command understood but rejected (e.g. out-of-range request).
pub const TS_RESPONSE_REJECTED: u8 = 0x01;
/// Command byte not recognised.
pub const TS_RESPONSE_UNRECOGNISED: u8 = 0x04;

// ── Command bytes ────────────────────────────────────────────────────────────
/// Query protocol version (`F`).
pub const CMD_PROTOCOL: u8 = b'F';
/// Hello / get signature (`S`).
pub const CMD_HELLO: u8 = b'S';
/// Read a region of the configuration page (`R`).
pub const CMD_READ: u8 = b'R';
/// Write a chunk into the configuration page (`C`).
pub const CMD_WRITE: u8 = b'C';
/// Burn (commit) the configuration page (`B`).
pub const CMD_BURN: u8 = b'B';
/// Read live output channels (`O`).
pub const CMD_OUTPUT: u8 = b'O';
/// Get firmware version (`V`).
pub const CMD_VERSION: u8 = b'V';
/// CRC32 check of a configuration region (`k`).
pub const CMD_CRC: u8 = b'k';

/// Compute the IEEE 802.3 CRC-32 (reflected, matches the host `crc32fast`).
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Errors when decoding a framed packet from a byte stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Not enough bytes for a complete frame yet.
    Incomplete,
    /// The framed payload length exceeds the receive buffer.
    TooLarge,
    /// CRC32 of the payload did not match.
    CrcMismatch,
}

/// Decode one framed packet from the front of `buf`.
///
/// On success returns the payload slice and the total number of bytes consumed
/// (so the caller can drain its receive buffer).
pub fn decode_frame(buf: &[u8]) -> Result<(&[u8], usize), FrameError> {
    if buf.len() < FRAMING_OVERHEAD {
        return Err(FrameError::Incomplete);
    }
    let len = u16::from_be_bytes([buf[0], buf[1]]) as usize;
    let total = len + FRAMING_OVERHEAD;
    if buf.len() < total {
        return Err(FrameError::Incomplete);
    }
    let payload = &buf[2..2 + len];
    let received = u32::from_be_bytes([
        buf[2 + len],
        buf[2 + len + 1],
        buf[2 + len + 2],
        buf[2 + len + 3],
    ]);
    if crc32(payload) != received {
        return Err(FrameError::CrcMismatch);
    }
    Ok((payload, total))
}

/// Frame `payload` into `out`, returning the total packet length.
///
/// Returns `None` if `out` is too small.
pub fn encode_frame(payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let total = payload.len() + FRAMING_OVERHEAD;
    if out.len() < total || payload.len() > u16::MAX as usize {
        return None;
    }
    let len = payload.len() as u16;
    out[0..2].copy_from_slice(&len.to_be_bytes());
    out[2..2 + payload.len()].copy_from_slice(payload);
    let crc = crc32(payload);
    out[2 + payload.len()..total].copy_from_slice(&crc.to_be_bytes());
    Some(total)
}

/// Mutable view of the firmware's tunable state, presented to the protocol
/// handler. `config` is the flat tune page (read/write); `outputs` is a
/// snapshot of the live output channels.
pub struct TuneState<'a> {
    /// Firmware signature string (returned for hello).
    pub signature: &'a [u8],
    /// Firmware version string (returned for `V`).
    pub firmware_version: &'a [u8],
    /// Flat configuration page (the tune), read and written by TunerStudio.
    pub config: &'a mut [u8],
    /// Live output-channel snapshot (read-only).
    pub outputs: &'a [u8],
    /// Set to `true` when a burn command commits the page.
    pub burn_pending: bool,
}

#[inline]
fn be_u16(slice: &[u8], idx: usize) -> Option<usize> {
    let hi = *slice.get(idx)?;
    let lo = *slice.get(idx + 1)?;
    Some(u16::from_be_bytes([hi, lo]) as usize)
}

/// Build a single-byte response (just a response code) into `resp`.
fn code_only(resp: &mut [u8], code: u8) -> Option<usize> {
    *resp.get_mut(0)? = code;
    Some(1)
}

/// Build a `[OK][data]` response into `resp`. Returns the response length.
fn ok_with(resp: &mut [u8], data: &[u8]) -> Option<usize> {
    if resp.len() < 1 + data.len() {
        return None;
    }
    resp[0] = TS_RESPONSE_OK;
    resp[1..1 + data.len()].copy_from_slice(data);
    Some(1 + data.len())
}

/// Handle one request payload and write the *response payload* (including the
/// leading response code) into `resp`. Returns the response length, or `None`
/// if the response buffer is too small or nothing should be sent.
pub fn handle_request(req: &[u8], state: &mut TuneState, resp: &mut [u8]) -> Option<usize> {
    let cmd = *req.first()?;
    match cmd {
        CMD_HELLO => ok_with(resp, state.signature),
        CMD_VERSION => ok_with(resp, state.firmware_version),
        CMD_PROTOCOL => ok_with(resp, b"001"),

        CMD_OUTPUT => {
            // [O][offset:u16][length:u16]
            let offset = be_u16(req, 1)?;
            let length = be_u16(req, 3)?;
            match state.outputs.get(offset..offset + length) {
                Some(slice) => ok_with(resp, slice),
                None => code_only(resp, TS_RESPONSE_REJECTED),
            }
        }

        CMD_READ => {
            // [R][page:u16][offset:u16][length:u16]
            let offset = be_u16(req, 3)?;
            let length = be_u16(req, 5)?;
            match state.config.get(offset..offset + length) {
                Some(slice) => ok_with(resp, slice),
                None => code_only(resp, TS_RESPONSE_REJECTED),
            }
        }

        CMD_WRITE => {
            // [C][page:u16][offset:u16][data...]
            let offset = be_u16(req, 3)?;
            let data = req.get(5..)?;
            match state.config.get_mut(offset..offset + data.len()) {
                Some(dst) => {
                    dst.copy_from_slice(data);
                    code_only(resp, TS_RESPONSE_OK)
                }
                None => code_only(resp, TS_RESPONSE_REJECTED),
            }
        }

        CMD_BURN => {
            state.burn_pending = true;
            code_only(resp, TS_RESPONSE_OK)
        }

        CMD_CRC => {
            // [k][page:u16][offset:u16][length:u16] -> OK + u32 CRC (BE)
            let offset = be_u16(req, 3)?;
            let length = be_u16(req, 5)?;
            match state.config.get(offset..offset + length) {
                Some(slice) => ok_with(resp, &crc32(slice).to_be_bytes()),
                None => code_only(resp, TS_RESPONSE_REJECTED),
            }
        }

        _ => code_only(resp, TS_RESPONSE_UNRECOGNISED),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        // Standard CRC-32 check value for the ASCII string "123456789".
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn frame_round_trip() {
        let payload = b"hello world";
        let mut buf = [0u8; 64];
        let n = encode_frame(payload, &mut buf).unwrap();
        let (decoded, consumed) = decode_frame(&buf[..n]).unwrap();
        assert_eq!(decoded, payload);
        assert_eq!(consumed, n);
    }

    #[test]
    fn frame_incomplete_is_reported() {
        let mut buf = [0u8; 64];
        let n = encode_frame(b"abc", &mut buf).unwrap();
        assert_eq!(decode_frame(&buf[..n - 1]), Err(FrameError::Incomplete));
    }

    #[test]
    fn frame_crc_mismatch_detected() {
        let mut buf = [0u8; 64];
        let n = encode_frame(b"abc", &mut buf).unwrap();
        buf[2] ^= 0xFF; // corrupt a payload byte
        assert_eq!(decode_frame(&buf[..n]), Err(FrameError::CrcMismatch));
    }

    fn make_state<'a>(config: &'a mut [u8], outputs: &'a [u8]) -> TuneState<'a> {
        TuneState {
            signature: b"rusEFI RustEMS",
            firmware_version: b"2026.05",
            config,
            outputs,
            burn_pending: false,
        }
    }

    #[test]
    fn hello_returns_signature() {
        let mut cfg = [0u8; 8];
        let outs = [0u8; 4];
        let mut state = make_state(&mut cfg, &outs);
        let mut resp = [0u8; 64];
        let n = handle_request(&[CMD_HELLO], &mut state, &mut resp).unwrap();
        assert_eq!(resp[0], TS_RESPONSE_OK);
        assert_eq!(&resp[1..n], b"rusEFI RustEMS");
    }

    #[test]
    fn output_channels_returns_slice() {
        let mut cfg = [0u8; 4];
        let outs = [10u8, 20, 30, 40, 50];
        let mut state = make_state(&mut cfg, &outs);
        let mut resp = [0u8; 64];
        // [O][offset=1][length=3]
        let req = [CMD_OUTPUT, 0, 1, 0, 3];
        let n = handle_request(&req, &mut state, &mut resp).unwrap();
        assert_eq!(resp[0], TS_RESPONSE_OK);
        assert_eq!(&resp[1..n], &[20, 30, 40]);
    }

    #[test]
    fn write_then_read_and_crc() {
        let mut cfg = [0u8; 16];
        let outs = [0u8; 4];
        let mut state = make_state(&mut cfg, &outs);
        let mut resp = [0u8; 64];

        // Write [0xAA,0xBB] at offset 2: [C][page=0][offset=2][data]
        let req = [CMD_WRITE, 0, 0, 0, 2, 0xAA, 0xBB];
        let n = handle_request(&req, &mut state, &mut resp).unwrap();
        assert_eq!(&resp[..n], &[TS_RESPONSE_OK]);

        // Read 2 bytes at offset 2: [R][page=0][offset=2][length=2]
        let req = [CMD_READ, 0, 0, 0, 2, 0, 2];
        let n = handle_request(&req, &mut state, &mut resp).unwrap();
        assert_eq!(resp[0], TS_RESPONSE_OK);
        assert_eq!(&resp[1..n], &[0xAA, 0xBB]);

        // Burn marks the page committed.
        let _ = handle_request(&[CMD_BURN], &mut state, &mut resp).unwrap();
        assert!(state.burn_pending);
    }

    #[test]
    fn out_of_range_read_is_rejected_not_panicked() {
        let mut cfg = [0u8; 4];
        let outs = [0u8; 4];
        let mut state = make_state(&mut cfg, &outs);
        let mut resp = [0u8; 64];
        // Read offset 2 length 100 — out of range.
        let req = [CMD_READ, 0, 0, 0, 2, 0, 100];
        let n = handle_request(&req, &mut state, &mut resp).unwrap();
        assert_eq!(&resp[..n], &[TS_RESPONSE_REJECTED]);
    }

    #[test]
    fn unknown_command_is_rejected() {
        let mut cfg = [0u8; 4];
        let outs = [0u8; 4];
        let mut state = make_state(&mut cfg, &outs);
        let mut resp = [0u8; 64];
        let n = handle_request(&[b'Z'], &mut state, &mut resp).unwrap();
        assert_eq!(&resp[..n], &[TS_RESPONSE_UNRECOGNISED]);
    }
}
