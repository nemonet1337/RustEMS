//! Generic STM32F4/7 trigger input implementation.
//!
//! This is a stub implementation for the trait-only crate.
//! The actual implementation should be in board-specific crates.

use rusefi_core::hal::TriggerInput;

/// Generic trigger input (stub).
pub struct Stm32TriggerInput;

impl Stm32TriggerInput {
    /// Create a new trigger input.
    pub fn new() -> Self {
        Self
    }
}

impl TriggerInput for Stm32TriggerInput {
    fn read_crank_timestamp(&mut self) -> Option<u64> {
        None
    }

    fn read_cam_timestamp(&mut self) -> Option<u64> {
        None
    }
}
