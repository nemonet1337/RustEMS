//! CAN bus management and OBD-II protocol implementation.
//!
//! This module provides:
//! - OBD-II standard PID support for engine monitoring
//! - CAN frame filtering and routing
//! - Vendor-specific message support (TunerStudio integration)

pub mod dash;
pub mod obd2;

use crate::hal::CanBus;

/// CAN bus manager.
pub struct CanManager {
    enabled: bool,
    tx_count: u32,
    rx_count: u32,
}

impl CanManager {
    /// Create a new CAN manager.
    pub fn new() -> Self {
        Self {
            enabled: false,
            tx_count: 0,
            rx_count: 0,
        }
    }

    /// Enable CAN communication.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable CAN communication.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Check if CAN is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Process received frames and update statistics.
    pub fn process_rx<CB: CanBus>(&mut self, can: &mut CB) {
        while let Some(_frame) = can.receive() {
            self.rx_count += 1;
        }
    }

    /// Get transmission statistics.
    pub fn tx_count(&self) -> u32 {
        self.tx_count
    }

    /// Get reception statistics.
    pub fn rx_count(&self) -> u32 {
        self.rx_count
    }
}

impl Default for CanManager {
    fn default() -> Self {
        Self::new()
    }
}
