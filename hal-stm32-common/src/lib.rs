//! STM32F4/7 common HAL traits — board-agnostic interface definitions.
//!
//! This crate provides trait definitions for STM32F4/7-based boards.
//! Board-specific implementations are provided by board-specific crates.
//!
//! # Architecture
//!
//! - `board` module: `Board` trait and pin-set traits
//!
//! Concrete peripheral drivers (ADC, ignition, injector, trigger, CAN, timer)
//! are implemented in each board-specific crate (`hal-nano`, `hal-huge`,
//! `hal-proteus`, `hal-uaefi`, `hal-microrusefi`).
//!
//! # Example
//!
//! ```rust,ignore
//! use rusefi_hal_stm32_common::board::Board;
//!
//! // Board-specific implementation
//! pub struct MyBoard;
//! impl Board for MyBoard { /* ... */ }
//! ```

#![no_std]

pub mod board;

// Re-export common types for convenience
pub use board::{Board, AdcPinSet, IgnitionPinSet, TriggerPinSet, CanPinSet, SdCardPinSet};
