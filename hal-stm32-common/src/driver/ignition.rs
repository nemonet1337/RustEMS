//! Generic STM32F4/7 ignition output implementation.
//!
//! This is a stub implementation for the trait-only crate.
//! The actual implementation should be in board-specific crates.

use rusefi_core::hal::IgnitionOutput;

/// Generic ignition output (stub).
pub struct Stm32IgnitionOutput;

impl Stm32IgnitionOutput {
    /// Create a new ignition output.
    pub fn new() -> Self {
        Self
    }
}

impl IgnitionOutput for Stm32IgnitionOutput {
    fn coil_charge(&mut self, _cylinder: u8) {
        // stub implementation
    }

    fn coil_fire(&mut self, _cylinder: u8) {
        // stub implementation
    }
}
