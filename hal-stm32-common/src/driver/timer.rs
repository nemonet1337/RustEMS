//! Generic STM32F4/7 system timer implementation.
//!
//! This is a stub implementation for the trait-only crate.
//! The actual implementation should be in board-specific crates.

use rusefi_core::hal::{SystemTimer, TimerCallback};

/// Generic system timer (stub).
pub struct Stm32SystemTimer;

impl Stm32SystemTimer {
    /// Create a new system timer.
    pub fn new() -> Self {
        Self
    }
}

impl SystemTimer for Stm32SystemTimer {
    fn now_us(&self) -> u64 {
        unimplemented!()
    }

    fn schedule_us(&mut self, delay_us: u64, callback: TimerCallback) {
        unimplemented!()
    }
}
