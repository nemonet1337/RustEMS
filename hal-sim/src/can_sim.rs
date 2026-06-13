//! Simulator CAN bus — in-process FIFO backed by a heapless queue.
//!
//! Frames written via `transmit()` are echoed into the RX FIFO so the
//! control loop can read them back. Useful for loopback testing.

use rusefi_core::hal::{CanBus, CanFrame};

const FIFO_DEPTH: usize = 16;

/// Simulated CAN bus with a software loopback RX FIFO.
pub struct SimCanBus {
    rx: std::collections::VecDeque<CanFrame>,
}

impl SimCanBus {
    /// Create a new simulated CAN bus.
    pub fn new() -> Self {
        Self {
            rx: std::collections::VecDeque::with_capacity(FIFO_DEPTH),
        }
    }

    /// Inject a frame directly into the RX FIFO (simulates incoming traffic).
    pub fn inject(&mut self, frame: CanFrame) {
        if self.rx.len() < FIFO_DEPTH {
            self.rx.push_back(frame);
        }
    }
}

impl Default for SimCanBus {
    fn default() -> Self {
        Self::new()
    }
}

impl CanBus for SimCanBus {
    fn transmit(&mut self, frame: &CanFrame) -> bool {
        // Loopback: echo TX into RX
        self.inject(*frame);
        true
    }

    fn receive(&mut self) -> Option<CanFrame> {
        self.rx.pop_front()
    }
}
