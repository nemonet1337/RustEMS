//! Simulator UART — backed by a pair of `std::collections::VecDeque` buffers.
//!
//! Bytes written via `write_bytes()` are placed in the TX buffer; an external
//! test harness can drain the TX buffer or feed the RX buffer independently.

use rusefi_core::hal::UartPort;

const BUF_DEPTH: usize = 256;

/// Simulated UART port.
pub struct SimUartPort {
    /// Bytes the firmware has transmitted.
    pub tx_buf: std::collections::VecDeque<u8>,
    /// Bytes available for the firmware to receive.
    pub rx_buf: std::collections::VecDeque<u8>,
}

impl SimUartPort {
    /// Create a new simulated UART port with empty buffers.
    pub fn new() -> Self {
        Self {
            tx_buf: std::collections::VecDeque::with_capacity(BUF_DEPTH),
            rx_buf: std::collections::VecDeque::with_capacity(BUF_DEPTH),
        }
    }

    /// Feed bytes into the simulated RX path (as if received from wire).
    pub fn feed_rx(&mut self, data: &[u8]) {
        for &b in data {
            if self.rx_buf.len() < BUF_DEPTH {
                self.rx_buf.push_back(b);
            }
        }
    }

    /// Drain all bytes that the firmware has transmitted.
    pub fn drain_tx(&mut self) -> std::vec::Vec<u8> {
        self.tx_buf.drain(..).collect()
    }
}

impl Default for SimUartPort {
    fn default() -> Self {
        Self::new()
    }
}

impl UartPort for SimUartPort {
    fn write_bytes(&mut self, buf: &[u8]) -> usize {
        let mut written = 0;
        for &b in buf {
            if self.tx_buf.len() < BUF_DEPTH {
                self.tx_buf.push_back(b);
                written += 1;
            } else {
                break;
            }
        }
        written
    }

    fn read_bytes(&mut self, buf: &mut [u8]) -> usize {
        let mut read = 0;
        for slot in buf.iter_mut() {
            match self.rx_buf.pop_front() {
                Some(b) => { *slot = b; read += 1; }
                None    => break,
            }
        }
        read
    }
}
