//! Fragment reassembly helper.

use crate::frame::{Flags, FrameHeader};
use crate::message::ErrorCode;

/// Helper to reassemble fragmented payloads over stream or packet transports.
#[derive(Debug, Default)]
pub struct Defragmenter<const N: usize> {
    buffer: heapless::Vec<u8, N>,
    current_seq: Option<u16>,
}

impl<const N: usize> Defragmenter<N> {
    /// Create a new defragmenter.
    pub const fn new() -> Self {
        Self {
            buffer: heapless::Vec::new(),
            current_seq: None,
        }
    }

    /// Reset the internal state, discarding any buffered fragments.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.current_seq = None;
    }

    /// Feed a frame into the defragmenter.
    ///
    /// - If the frame is not fragmented, it returns `Ok(Some(payload))` pointing directly to the input payload (bypass).
    /// - If it is a fragment, it buffers the payload.
    /// - If it is the last fragment, it appends it and returns `Ok(Some(&reassembled_payload))`.
    /// - Otherwise, returns `Ok(None)` indicating more fragments are expected.
    ///
    /// # Errors
    /// Returns `Err(ErrorCode::Fragmentation)` if:
    /// - The sequence number of a fragment does not match the active reassembly sequence.
    /// - The internal buffer overflows.
    pub fn feed<'a>(
        &'a mut self,
        header: &FrameHeader,
        payload: &'a [u8],
    ) -> Result<Option<&'a [u8]>, ErrorCode> {
        let is_fragment = header.flags.has(Flags::FRAGMENT);
        let is_last = header.flags.has(Flags::LAST_FRAGMENT);

        match (is_fragment, is_last) {
            (false, _) => {
                // Not fragmented. If we were in the middle of reassembling another sequence,
                // discard it (new unfragmented frame takes precedence/interrupts).
                self.reset();
                Ok(Some(payload))
            }
            (true, false) => {
                // First or intermediate fragment.
                if let Some(seq) = self.current_seq {
                    if seq != header.seq {
                        self.reset();
                        return Err(ErrorCode::Fragmentation);
                    }
                } else {
                    // Start of a new fragmented stream.
                    self.buffer.clear();
                    self.current_seq = Some(header.seq);
                }

                if self.buffer.extend_from_slice(payload).is_err() {
                    self.reset();
                    return Err(ErrorCode::Fragmentation);
                }
                Ok(None)
            }
            (true, true) => {
                // Last fragment.
                if let Some(seq) = self.current_seq {
                    if seq != header.seq {
                        self.reset();
                        return Err(ErrorCode::Fragmentation);
                    }
                } else {
                    // Received a final fragment without receiving any previous fragment.
                    // We allow it as a single final fragment, but reset first.
                    self.buffer.clear();
                }

                if self.buffer.extend_from_slice(payload).is_err() {
                    self.reset();
                    return Err(ErrorCode::Fragmentation);
                }

                self.current_seq = None; // Reset sequence tracking, but keep buffer contents intact.
                Ok(Some(&self.buffer))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unfragmented_bypass() {
        let mut defrag = Defragmenter::<1024>::new();
        let header = FrameHeader::new(Flags::none(), 123);
        let payload = b"hello";

        let result = defrag.feed(&header, payload).unwrap();
        assert_eq!(result, Some(&payload[..]));
    }

    #[test]
    fn test_reassembly() {
        let mut defrag = Defragmenter::<1024>::new();

        let h1 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 42);
        let h2 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 42);
        let h3 = FrameHeader::new(
            Flags::none()
                .with(Flags::FRAGMENT)
                .with(Flags::LAST_FRAGMENT),
            42,
        );

        assert_eq!(defrag.feed(&h1, b"hello ").unwrap(), None);
        assert_eq!(defrag.feed(&h2, b"world").unwrap(), None);
        let result = defrag.feed(&h3, b"!").unwrap();
        assert_eq!(result, Some(b"hello world!" as &[u8]));
    }

    #[test]
    fn test_sequence_mismatch() {
        let mut defrag = Defragmenter::<1024>::new();

        let h1 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 42);
        let h2 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 43); // mismatched sequence

        assert_eq!(defrag.feed(&h1, b"hello").unwrap(), None);
        assert_eq!(defrag.feed(&h2, b"world"), Err(ErrorCode::Fragmentation));
    }

    #[test]
    fn test_buffer_overflow() {
        let mut defrag = Defragmenter::<8>::new(); // small buffer

        let h1 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 42);
        let h2 = FrameHeader::new(Flags::none().with(Flags::FRAGMENT), 42);

        assert_eq!(defrag.feed(&h1, b"12345").unwrap(), None);
        assert_eq!(defrag.feed(&h2, b"67890"), Err(ErrorCode::Fragmentation));
    }
}
