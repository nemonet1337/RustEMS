//! Generic STM32F4/7 injector output implementation.
//!
//! This is a stub implementation for the trait-only crate.
//! The actual implementation should be in board-specific crates.

use rusefi_core::hal::InjectorOutput;

/// Generic injector output (stub).
pub struct Stm32InjectorOutput<const N: usize>;

impl<const N: usize> Stm32InjectorOutput<N> {
    /// Create a new injector output.
    pub fn new() -> Self {
        Self
    }
}

impl<const N: usize> InjectorOutput for Stm32InjectorOutput<N> {
    fn open(&mut self, _cylinder: u8) {}

    fn close(&mut self, _cylinder: u8) {}
}
