//! Board trait and pin-set traits for STM32F4/7 HAL.
//!
//! This module defines the interface between board-specific pin configurations
//! and the generic driver implementations.

use rusefi_core::sensors::AdcChannel;

// ============================================================================
// Pin Set Traits
// ============================================================================

/// ADC pin set trait - maps sensor channels to physical pins.
pub trait AdcPinSet {
    /// Read the raw ADC value for the given channel.
    fn read(&self, channel: AdcChannel) -> u16;
}

/// Ignition pin set trait - provides coil output pins.
pub trait IgnitionPinSet {
    /// Set the coil output for the given cylinder.
    fn set_coil(&mut self, cylinder: u8, state: bool);
}

/// Trigger pin set trait - provides crank and cam input pins.
pub trait TriggerPinSet {
    /// Get the crank pin value.
    fn crank_pin(&self) -> bool;
    /// Get the cam pin value.
    fn cam_pin(&self) -> bool;
}

/// CAN pin set trait - provides CAN peripheral and pins.
pub trait CanPinSet {
    /// The CAN peripheral type (CAN1 or CAN2).
    type Peripheral;
    /// The CAN bus type.
    type Can;
    /// Create a CAN bus instance from the pins and peripheral.
    fn into_can(self, can: Self::Peripheral) -> Self::Can;
}

/// SD card pin set trait - provides SDMMC pins.
pub trait SdCardPinSet {
    // SDMMC pin configuration (stub for now)
}

// ============================================================================
// Board Trait
// ============================================================================

/// Board configuration trait - defines hardware capabilities and pin mapping.
pub trait Board {
    /// ADC pin set for this board.
    type AdcPins: AdcPinSet;
    /// Ignition pin set for this board.
    type IgnitionPins: IgnitionPinSet;
    /// Trigger pin set for this board.
    type TriggerPins: TriggerPinSet;
    /// CAN pin set for this board.
    type CanPins: CanPinSet;
    /// SD card pin set for this board.
    type SdCardPins: SdCardPinSet;

    // ── Output counts ─────────────────────────────────────────────────────────────
    /// Number of cylinders this board supports.
    const CYLINDER_COUNT: u8;
    /// Number of injector outputs.
    const INJECTOR_COUNT: u8;
    /// Number of ignition coil outputs.
    const IGNITION_COUNT: u8;
    /// Number of high-side 12V outputs.
    const HS_OUTPUT_COUNT: u8;
    /// Number of additional low-side / H-bridge outputs.
    const LS_EXTRA_OUTPUT_COUNT: u8;

    // ── Input counts ─────────────────────────────────────────────────────────────
    /// Number of general-purpose ADC inputs.
    const ADC_GP_COUNT: u8;
    /// Number of thermistor ADC inputs.
    const ADC_THERM_COUNT: u8;
    /// Number of Hall effect inputs.
    const HALL_INPUT_COUNT: u8;
    /// Number of VR (variable reluctance) sensor inputs.
    const VR_INPUT_COUNT: u8;

    // ── Bus counts ──────────────────────────────────────────────────────────────
    /// Number of CAN buses (1 or 2).
    const CAN_COUNT: u8;

    // ── Built-in features (bool) ────────────────────────────────────────────────
    /// Has internal wideband O2 controller.
    const HAS_INTERNAL_WBO: bool;
    /// Has dual wideband O2 support.
    const HAS_DUAL_WBO: bool;
    /// Has internal knock sensor controller.
    const HAS_INTERNAL_KNOCK: bool;
    /// Has dual knock sensor support.
    const HAS_DUAL_KNOCK: bool;
    /// Has dual electronic throttle body support.
    const HAS_DUAL_ETB: bool;
    /// Has internal barometric pressure sensor.
    const HAS_INTERNAL_BARO: bool;
    /// Has SD card support.
    const HAS_SDCARD: bool;
    /// Has Bluetooth module.
    const HAS_BLUETOOTH: bool;
    /// Has flex fuel sensor support.
    const HAS_FLEX_FUEL: bool;
}

// ============================================================================
// Generic CanBus Implementation
// ============================================================================

/// Generic CAN bus (stub).
pub struct GenericCanBus;

impl GenericCanBus {
    /// Create a new CAN bus.
    pub fn new() -> Self {
        Self
    }
}
