//! CRC-16/CCITT-FALSE.
//!
//! Parameters: polynomial `0x1021`, init `0xFFFF`, no input/output reflection,
//! no final XOR. Check value for the ASCII string `"123456789"` is `0x29B1`.
//!
//! This is used by [`crate::frame`] to protect each frame. It replaces the
//! CRC32 of the legacy protocol to reduce overhead on small BLE MTUs.

const POLY: u16 = 0x1021;
const INIT: u16 = 0xFFFF;

/// Compute the CRC-16/CCITT-FALSE checksum over `data`.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = INIT;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ POLY
            } else {
                crc << 1
            };
            bit += 1;
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        // Canonical CRC-16/CCITT-FALSE check value.
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn empty_is_init() {
        assert_eq!(crc16_ccitt(&[]), INIT);
    }

    #[test]
    fn single_byte_changes() {
        assert_ne!(crc16_ccitt(&[0x00]), crc16_ccitt(&[0x01]));
    }

    #[test]
    fn order_matters() {
        assert_ne!(crc16_ccitt(&[0x01, 0x02]), crc16_ccitt(&[0x02, 0x01]));
    }
}
