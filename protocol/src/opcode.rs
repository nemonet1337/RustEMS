//! rusEFI binary protocol opcodes and typed command/response types.
//!
//! Opcodes are defined in `firmware/controllers/can/rusefi_can.h` and
//! `java_console/io/…/binaryprotocol/BinaryProtocolCommands.java`.

/// Get protocol version string
pub const TS_GET_PROTOCOL_VERSION_COMMAND_F: u8 = b'F';

/// Hello / identify command
pub const TS_HELLO_COMMAND: u8 = b'S';

/// Read memory page
pub const TS_READ_COMMAND: u8 = b'R';

/// Write memory page
pub const TS_CHUNK_WRITE_COMMAND: u8 = b'C';

/// Burn (persist) configuration to flash
pub const TS_BURN_COMMAND: u8 = b'B';

/// Request output channels (live data)
pub const TS_OUTPUT_COMMAND: u8 = b'O';

/// Execute a scripted action
pub const TS_EXECUTE: u8 = b'X';

/// Request a CRC of a page range
pub const TS_CRC_CHECK_COMMAND: u8 = b'k';

/// Get firmware signature/build info
pub const TS_GET_FIRMWARE_VERSION: u8 = b'V';

/// Response status: OK
pub const TS_RESPONSE_OK: u8 = 0x00;

/// Response status: Rejected (CRC mismatch etc.)
pub const TS_RESPONSE_REJECTED: u8 = 0x01;

/// Response status: Unrecognised command
pub const TS_RESPONSE_UNRECOGNISED_COMMAND: u8 = 0x04;

/// A typed command ready to be serialised into a packet payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `F` — request the protocol version string
    GetProtocolVersion,

    /// `S` — hello / identify
    Hello,

    /// `V` — request firmware version string
    GetFirmwareVersion,

    /// `B` — burn configuration to flash
    Burn,

    /// `R page offset length` — read `length` bytes from `page` at `offset`
    ReadPage { page: u16, offset: u16, length: u16 },

    /// `C page offset data` — write data chunk to `page` at `offset`
    WriteChunk {
        page: u16,
        offset: u16,
        data: Vec<u8>,
    },

    /// `O offset length` — request output channels starting at `offset`
    OutputChannels { offset: u16, length: u16 },

    /// `k page offset length` — request CRC32 of page range
    CrcCheck { page: u16, offset: u16, length: u16 },

    /// `X subcommand data...` — execute a scripted action
    ///
    /// Subcommands:
    /// - 0x01: Test check engine light
    /// - 0x02: Clear fatal error
    /// - 0x10: Disable fuel injection
    /// - 0x11: Enable fuel injection
    /// - 0x12: Disable ignition
    /// - 0x13: Enable ignition
    Execute { subcommand: u8, data: Vec<u8> },
}

impl Command {
    /// Serialise the command into a payload `Vec<u8>` suitable for [`encode_packet_vec`](crate::packet::encode_packet_vec).
    pub fn to_payload(&self) -> Vec<u8> {
        match self {
            Command::GetProtocolVersion => vec![TS_GET_PROTOCOL_VERSION_COMMAND_F],
            Command::Hello => vec![TS_HELLO_COMMAND],
            Command::GetFirmwareVersion => vec![TS_GET_FIRMWARE_VERSION],
            Command::Burn => vec![TS_BURN_COMMAND],

            Command::ReadPage {
                page,
                offset,
                length,
            } => {
                let mut v = vec![TS_READ_COMMAND];
                v.extend_from_slice(&page.to_be_bytes());
                v.extend_from_slice(&offset.to_be_bytes());
                v.extend_from_slice(&length.to_be_bytes());
                v
            }

            Command::WriteChunk { page, offset, data } => {
                let mut v = vec![TS_CHUNK_WRITE_COMMAND];
                v.extend_from_slice(&page.to_be_bytes());
                v.extend_from_slice(&offset.to_be_bytes());
                v.extend_from_slice(data);
                v
            }

            Command::OutputChannels { offset, length } => {
                let mut v = vec![TS_OUTPUT_COMMAND];
                v.extend_from_slice(&offset.to_be_bytes());
                v.extend_from_slice(&length.to_be_bytes());
                v
            }

            Command::CrcCheck {
                page,
                offset,
                length,
            } => {
                let mut v = vec![TS_CRC_CHECK_COMMAND];
                v.extend_from_slice(&page.to_be_bytes());
                v.extend_from_slice(&offset.to_be_bytes());
                v.extend_from_slice(&length.to_be_bytes());
                v
            }

            Command::Execute { subcommand, data } => {
                let mut v = vec![TS_EXECUTE, *subcommand];
                v.extend_from_slice(data);
                v
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_payload() {
        assert_eq!(Command::Hello.to_payload(), vec![TS_HELLO_COMMAND]);
    }

    #[test]
    fn read_page_payload() {
        let cmd = Command::ReadPage {
            page: 1,
            offset: 0x10,
            length: 0x20,
        };
        let payload = cmd.to_payload();
        assert_eq!(payload[0], TS_READ_COMMAND);
        assert_eq!(&payload[1..3], &[0x00, 0x01]); // page BE
        assert_eq!(&payload[3..5], &[0x00, 0x10]); // offset BE
        assert_eq!(&payload[5..7], &[0x00, 0x20]); // length BE
    }

    #[test]
    fn write_chunk_payload() {
        let data = vec![0xAA, 0xBB, 0xCC];
        let cmd = Command::WriteChunk {
            page: 0,
            offset: 4,
            data: data.clone(),
        };
        let payload = cmd.to_payload();
        assert_eq!(payload[0], TS_CHUNK_WRITE_COMMAND);
        assert_eq!(&payload[5..], data.as_slice());
    }

    #[test]
    fn execute_payload() {
        let data = vec![0x01, 0x02, 0x03];
        let cmd = Command::Execute {
            subcommand: 0x10,
            data: data.clone(),
        };
        let payload = cmd.to_payload();
        assert_eq!(payload[0], TS_EXECUTE);
        assert_eq!(payload[1], 0x10); // subcommand
        assert_eq!(&payload[2..], data.as_slice());
    }

    #[test]
    fn execute_clear_fatal() {
        let cmd = Command::Execute {
            subcommand: 0x02,
            data: vec![],
        };
        let payload = cmd.to_payload();
        assert_eq!(payload, vec![TS_EXECUTE, 0x02]);
    }
}
