//! Generic STM32F4/7 CAN bus driver.
//!
//! This module provides the trait definition and common types for CAN bus
//! implementations. Board-specific implementations are provided by
//! board-specific crates (e.g., `rusefi-hal-stm32f4`, `rusefi-hal-stm32f7`).

use rusefi_core::hal::{CanBus, CanFrame};

/// CAN bus configuration.
#[derive(Clone, Copy, Debug)]
pub struct CanConfig {
    /// Bit rate in kbps (e.g., 500 for 500 kbps).
    pub bitrate_kbps: u32,
    /// Enable silent mode (listen-only, no transmission).
    pub silent_mode: bool,
    /// Enable automatic retransmission.
    pub auto_retransmit: bool,
}

impl Default for CanConfig {
    fn default() -> Self {
        Self {
            bitrate_kbps: 500,
            silent_mode: false,
            auto_retransmit: true,
        }
    }
}

/// Generic STM32 CAN bus implementation (trait-only).
///
/// Board-specific crates should implement this trait for their hardware.
pub trait Stm32Can: CanBus {
    /// Initialize the CAN peripheral with the given configuration.
    fn init(&mut self, config: CanConfig) -> Result<(), CanError>;

    /// Enable or disable the CAN peripheral.
    fn set_enabled(&mut self, enabled: bool);
}

/// CAN bus errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanError {
    /// Hardware error (bus-off, etc.).
    HardwareError,
    /// Configuration error (invalid bit rate, etc.).
    ConfigError,
    /// TX mailbox full.
    TxFull,
    /// RX FIFO empty.
    RxEmpty,
}
