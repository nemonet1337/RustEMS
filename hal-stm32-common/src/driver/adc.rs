//! Generic STM32F4/7 ADC input implementation.
//!
//! This is a stub implementation for the trait-only crate.
//! The actual implementation should be in board-specific crates.

use rusefi_core::hal::AdcInput;
use rusefi_core::sensors::AdcChannel;

/// Generic ADC input (stub).
pub struct Stm32AdcInput;

impl Stm32AdcInput {
    /// Create a new ADC input.
    pub fn new() -> Self {
        Self
    }
}

impl AdcInput for Stm32AdcInput {
    fn read_raw(&mut self, _channel: AdcChannel) -> u16 {
        0
    }
}
