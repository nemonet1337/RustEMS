//! Consistent Overhead Byte Stuffing (COBS).
//!
//! COBS removes all `0x00` bytes from a payload so that `0x00` can be used as
//! an unambiguous frame delimiter on byte-stream transports (USB CDC-ACM,
//! Bluetooth SPP). It is self-synchronizing: a receiver that joins mid-stream
//! recovers at the next `0x00`.
//!
//! These functions write into caller-provided buffers and never allocate or
//! panic; insufficient buffers return [`CobsError`].

/// Errors from COBS encode/decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CobsError {
    /// Output buffer is too small for the result.
    OutputTooSmall,
    /// Encoded input ended in the middle of a block.
    Truncated,
    /// A `0x00` byte appeared inside encoded data (must be the delimiter only).
    UnexpectedZero,
}

/// Maximum number of bytes [`encode`] can produce for `n` input bytes,
/// excluding the trailing delimiter.
pub const fn max_encoded_len(n: usize) -> usize {
    // One overhead byte, plus one extra per 254-byte run.
    n + n / 254 + 1
}

/// COBS-encode `input` into `output` (delimiter NOT appended).
///
/// Returns the number of bytes written. The caller typically appends a single
/// `0x00` delimiter afterwards.
pub fn encode(input: &[u8], output: &mut [u8]) -> Result<usize, CobsError> {
    if output.len() < max_encoded_len(input.len()) {
        return Err(CobsError::OutputTooSmall);
    }

    // `code_index` points at the slot reserved for the current run length.
    let mut code_index = 0usize;
    let mut write = 1usize;
    let mut code: u8 = 1;

    for &byte in input {
        if byte == 0 {
            set(output, code_index, code)?;
            code_index = write;
            write += 1;
            code = 1;
        } else {
            set(output, write, byte)?;
            write += 1;
            code += 1;
            if code == 0xFF {
                set(output, code_index, code)?;
                code_index = write;
                write += 1;
                code = 1;
            }
        }
    }
    set(output, code_index, code)?;
    Ok(write)
}

/// COBS-decode `input` (with the trailing `0x00` delimiter already removed)
/// into `output`. Returns the number of decoded bytes.
pub fn decode(input: &[u8], output: &mut [u8]) -> Result<usize, CobsError> {
    let mut read = 0usize;
    let mut write = 0usize;

    while read < input.len() {
        let code = input[read];
        if code == 0 {
            return Err(CobsError::UnexpectedZero);
        }
        read += 1;

        let block_len = (code - 1) as usize;
        let mut i = 0;
        while i < block_len {
            let byte = *input.get(read).ok_or(CobsError::Truncated)?;
            set(output, write, byte)?;
            read += 1;
            write += 1;
            i += 1;
        }

        // A run shorter than 0xFF implies an elided zero, unless we are at the end.
        if code != 0xFF && read < input.len() {
            set(output, write, 0)?;
            write += 1;
        }
    }
    Ok(write)
}

/// Bounds-checked write that maps an out-of-range index to [`CobsError`].
#[inline]
fn set(buf: &mut [u8], index: usize, value: u8) -> Result<(), CobsError> {
    *buf.get_mut(index).ok_or(CobsError::OutputTooSmall)? = value;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(data: &[u8]) {
        let mut enc = [0u8; 1024];
        let n = encode(data, &mut enc).unwrap();
        // Encoded data must never contain a zero (so the delimiter is unique).
        assert!(!enc[..n].contains(&0), "encoded data contains 0x00");
        let mut dec = [0u8; 1024];
        let m = decode(&enc[..n], &mut dec).unwrap();
        assert_eq!(&dec[..m], data);
    }

    #[test]
    fn round_trip_empty() {
        round_trip(&[]);
    }

    #[test]
    fn round_trip_no_zeros() {
        round_trip(&[1, 2, 3, 4, 5]);
    }

    #[test]
    fn round_trip_with_zeros() {
        round_trip(&[0, 0, 0]);
        round_trip(&[1, 0, 2, 0, 3]);
        round_trip(&[0, 1, 2, 0]);
    }

    #[test]
    fn round_trip_long_run() {
        // > 254 non-zero bytes forces a code overflow (0xFF) split.
        let data: [u8; 600] = core::array::from_fn(|i| (i % 255 + 1) as u8);
        round_trip(&data);
    }

    #[test]
    fn round_trip_long_run_with_zeros() {
        let mut data = [7u8; 600];
        data[300] = 0;
        data[599] = 0;
        round_trip(&data);
    }

    #[test]
    fn encode_output_too_small() {
        let data = [1, 2, 3];
        let mut out = [0u8; 2];
        assert_eq!(encode(&data, &mut out), Err(CobsError::OutputTooSmall));
    }

    #[test]
    fn decode_rejects_embedded_zero() {
        let mut out = [0u8; 16];
        assert_eq!(
            decode(&[2, 1, 0, 1], &mut out),
            Err(CobsError::UnexpectedZero)
        );
    }

    #[test]
    fn decode_truncated() {
        // code says 4 data bytes follow, but only 1 is present.
        let mut out = [0u8; 16];
        assert_eq!(decode(&[5, 1], &mut out), Err(CobsError::Truncated));
    }

    #[test]
    fn max_encoded_len_bound_holds() {
        for n in [0usize, 1, 253, 254, 255, 508, 600] {
            let data: heapless::Vec<u8, 600> = (0..n).map(|i| (i % 254 + 1) as u8).collect();
            let mut enc = [0u8; 700];
            let written = encode(&data, &mut enc).unwrap();
            assert!(written <= max_encoded_len(n), "n={n} written={written}");
        }
    }
}
