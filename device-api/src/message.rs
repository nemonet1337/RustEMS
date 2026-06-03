//! Message layer carried in a frame payload.
//!
//! Layout: `KIND(u8) OP(u16 LE) BODY`. See `docs/api/03-message-protocol.md`.
//!
//! The body codec (CBOR for control, packed binary for telemetry) lives in
//! higher layers; this module only handles the message header plus the shared
//! [`Kind`], [`ErrorCode`], and [`ValueType`] enumerations and the opcode
//! catalog.

/// Size of the message header: KIND + OP(2).
pub const MESSAGE_HEADER_LEN: usize = 3;

/// Message direction/class.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    /// Host → device request.
    Request = 0,
    /// Device → host response, correlated by frame `seq`.
    Response = 1,
    /// Device → host asynchronous notification.
    Event = 2,
    /// Device → host streamed telemetry frame.
    Telemetry = 3,
}

impl Kind {
    /// Parse from the wire byte.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Kind::Request),
            1 => Some(Kind::Response),
            2 => Some(Kind::Event),
            3 => Some(Kind::Telemetry),
            _ => None,
        }
    }
}

/// Extract the category byte (high byte) of an opcode.
pub const fn op_category(op: u16) -> u8 {
    (op >> 8) as u8
}

/// Opcode catalog. Grouped by category in the high byte
/// (see `docs/api/03-message-protocol.md`).
pub mod op {
    // --- A. System (0x01) ---
    /// Identify device, capabilities, protocol/schema versions.
    pub const HELLO: u16 = 0x0101;
    /// Keepalive / liveness.
    pub const PING: u16 = 0x0102;
    /// Reboot the device.
    pub const REBOOT: u16 = 0x0103;
    /// Enter DFU/bootloader.
    pub const ENTER_BOOTLOADER: u16 = 0x0104;

    // --- B. Descriptor (0x02) ---
    /// Schema version/hash, counts, category list.
    pub const GET_SCHEMA_INFO: u16 = 0x0201;
    /// Paged parameter descriptors.
    pub const GET_PARAM_CATALOG: u16 = 0x0202;
    /// Paged table descriptors.
    pub const GET_TABLE_CATALOG: u16 = 0x0203;
    /// Paged telemetry channel descriptors.
    pub const GET_TELEMETRY_CATALOG: u16 = 0x0204;

    // --- C. Config (0x03) ---
    /// Read parameter values by id.
    pub const PARAM_GET: u16 = 0x0301;
    /// Write parameter values by id.
    pub const PARAM_SET: u16 = 0x0302;
    /// Read a full table (axes + cells).
    pub const TABLE_GET: u16 = 0x0303;
    /// Write a single table cell.
    pub const TABLE_SET_CELL: u16 = 0x0304;
    /// Write a table axis.
    pub const TABLE_SET_AXIS: u16 = 0x0305;
    /// Persist staged RAM config to flash.
    pub const CONFIG_SAVE: u16 = 0x0306;
    /// Discard staged RAM config (revert to flash).
    pub const CONFIG_DISCARD: u16 = 0x0307;
    /// Reset RAM config to defaults.
    pub const CONFIG_RESET_DEFAULTS: u16 = 0x0308;
    /// Report dirty/crc status.
    pub const CONFIG_STATUS: u16 = 0x0309;

    // --- D. Telemetry (0x04) ---
    /// Subscribe to a telemetry stream.
    pub const TELEM_SUBSCRIBE: u16 = 0x0401;
    /// Unsubscribe a telemetry stream.
    pub const TELEM_UNSUBSCRIBE: u16 = 0x0402;
    /// One-shot telemetry read.
    pub const TELEM_READ_ONCE: u16 = 0x0403;
    /// Pushed telemetry frame (Kind::Telemetry).
    pub const TELEM_FRAME: u16 = 0x04F0;

    // --- E. Control (0x05) ---
    /// Bench-test an actuator.
    pub const BENCH_TEST: u16 = 0x0501;
    /// Set a temporary control override.
    pub const SET_OVERRIDE: u16 = 0x0502;
    /// Clear a control override.
    pub const CLEAR_OVERRIDE: u16 = 0x0503;
    /// Run a calibration routine.
    pub const CALIBRATE: u16 = 0x0504;

    // --- F. Diagnostics (0x06) ---
    /// Read stored faults (DTCs).
    pub const GET_FAULTS: u16 = 0x0601;
    /// Clear faults by mask.
    pub const CLEAR_FAULTS: u16 = 0x0602;
    /// Pushed asynchronous event (Kind::Event).
    pub const EVENT: u16 = 0x06F0;
}

/// Standard error codes returned in a failed [`Kind::Response`] body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ErrorCode {
    /// Success.
    Ok = 0,
    /// Unknown opcode.
    UnknownOp = 1,
    /// Malformed request body.
    BadRequest = 2,
    /// Referenced id/channel does not exist.
    NotFound = 3,
    /// Value outside the allowed min/max.
    OutOfRange = 4,
    /// Parameter is read-only.
    ReadOnly = 5,
    /// Rejected because the engine is running.
    Busy = 6,
    /// Feature not supported on this board/build.
    NotSupported = 7,
    /// Fragment reassembly failed.
    Fragmentation = 8,
    /// Protocol/schema version mismatch.
    VersionMismatch = 9,
    /// Authentication required (reserved for future use).
    Unauthorized = 10,
}

impl ErrorCode {
    /// Parse from the wire value.
    pub const fn from_u16(v: u16) -> Option<Self> {
        match v {
            0 => Some(ErrorCode::Ok),
            1 => Some(ErrorCode::UnknownOp),
            2 => Some(ErrorCode::BadRequest),
            3 => Some(ErrorCode::NotFound),
            4 => Some(ErrorCode::OutOfRange),
            5 => Some(ErrorCode::ReadOnly),
            6 => Some(ErrorCode::Busy),
            7 => Some(ErrorCode::NotSupported),
            8 => Some(ErrorCode::Fragmentation),
            9 => Some(ErrorCode::VersionMismatch),
            10 => Some(ErrorCode::Unauthorized),
            _ => None,
        }
    }
}

/// Parameter / channel value types (see `docs/api/04-parameter-model.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ValueType {
    /// Unsigned 8-bit.
    U8 = 0,
    /// Signed 8-bit.
    I8 = 1,
    /// Unsigned 16-bit.
    U16 = 2,
    /// Signed 16-bit.
    I16 = 3,
    /// Unsigned 32-bit.
    U32 = 4,
    /// Signed 32-bit.
    I32 = 5,
    /// 32-bit float (default for physical quantities).
    F32 = 6,
    /// Boolean.
    Bool = 7,
    /// Enumeration (paired with labels in the descriptor).
    Enum = 8,
    /// Fixed-length string.
    Str = 9,
}

impl ValueType {
    /// Parse from the wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(ValueType::U8),
            1 => Some(ValueType::I8),
            2 => Some(ValueType::U16),
            3 => Some(ValueType::I16),
            4 => Some(ValueType::U32),
            5 => Some(ValueType::I32),
            6 => Some(ValueType::F32),
            7 => Some(ValueType::Bool),
            8 => Some(ValueType::Enum),
            9 => Some(ValueType::Str),
            _ => None,
        }
    }
}

/// Parsed message header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageHeader {
    /// Message class.
    pub kind: Kind,
    /// Opcode.
    pub op: u16,
}

/// Errors from message header encode/decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageError {
    /// Buffer too small to hold the header.
    BufferTooSmall,
    /// Unknown `KIND` byte.
    UnknownKind(u8),
}

/// Write `KIND OP` into the front of `buf`. Returns header length.
pub fn write_message_header(
    kind: Kind,
    op: u16,
    buf: &mut [u8],
) -> Result<usize, MessageError> {
    if buf.len() < MESSAGE_HEADER_LEN {
        return Err(MessageError::BufferTooSmall);
    }
    buf[0] = kind as u8;
    buf[1] = (op & 0xFF) as u8;
    buf[2] = (op >> 8) as u8;
    Ok(MESSAGE_HEADER_LEN)
}

/// Parse a message header from the front of `buf`, returning the header and the
/// remaining body slice.
pub fn read_message_header(buf: &[u8]) -> Result<(MessageHeader, &[u8]), MessageError> {
    if buf.len() < MESSAGE_HEADER_LEN {
        return Err(MessageError::BufferTooSmall);
    }
    let kind = Kind::from_u8(buf[0]).ok_or(MessageError::UnknownKind(buf[0]))?;
    let op = u16::from(buf[1]) | (u16::from(buf[2]) << 8);
    Ok((MessageHeader { kind, op }, &buf[MESSAGE_HEADER_LEN..]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trip() {
        let mut buf = [0u8; 32];
        let n = write_message_header(Kind::Request, op::HELLO, &mut buf).unwrap();
        assert_eq!(n, MESSAGE_HEADER_LEN);
        buf[n] = 0xAB; // body byte
        let (h, body) = read_message_header(&buf[..n + 1]).unwrap();
        assert_eq!(h.kind, Kind::Request);
        assert_eq!(h.op, op::HELLO);
        assert_eq!(body, &[0xAB]);
    }

    #[test]
    fn op_categories() {
        assert_eq!(op_category(op::HELLO), 0x01);
        assert_eq!(op_category(op::GET_PARAM_CATALOG), 0x02);
        assert_eq!(op_category(op::PARAM_SET), 0x03);
        assert_eq!(op_category(op::TELEM_FRAME), 0x04);
        assert_eq!(op_category(op::BENCH_TEST), 0x05);
        assert_eq!(op_category(op::EVENT), 0x06);
    }

    #[test]
    fn kind_round_trip() {
        for k in [Kind::Request, Kind::Response, Kind::Event, Kind::Telemetry] {
            assert_eq!(Kind::from_u8(k as u8), Some(k));
        }
        assert_eq!(Kind::from_u8(99), None);
    }

    #[test]
    fn error_code_round_trip() {
        for c in 0u16..=10 {
            let e = ErrorCode::from_u16(c).unwrap();
            assert_eq!(e as u16, c);
        }
        assert_eq!(ErrorCode::from_u16(999), None);
    }

    #[test]
    fn value_type_round_trip() {
        for v in 0u8..=9 {
            let t = ValueType::from_u8(v).unwrap();
            assert_eq!(t as u8, v);
        }
        assert_eq!(ValueType::from_u8(42), None);
    }

    #[test]
    fn unknown_kind_rejected() {
        let buf = [0x09u8, 0x01, 0x01];
        assert_eq!(read_message_header(&buf), Err(MessageError::UnknownKind(9)));
    }

    #[test]
    fn header_buffer_too_small() {
        let mut buf = [0u8; 2];
        assert_eq!(
            write_message_header(Kind::Request, op::HELLO, &mut buf),
            Err(MessageError::BufferTooSmall)
        );
    }
}
