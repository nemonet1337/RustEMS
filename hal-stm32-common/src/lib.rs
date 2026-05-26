//! STM32F4/7 common HAL traits — board-agnostic interface definitions.
//!
//! This crate provides trait definitions for STM32F4/7-based boards.
//! Board-specific implementations are provided by board-specific crates.
//!
//! # Architecture
//!
//! - `board` module: `Board` trait and pin-set traits
//! - `driver` module: Generic driver implementations (stub for now)
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
pub mod driver;

// Re-export common types for convenience
pub use board::{Board, AdcPinSet, IgnitionPinSet, TriggerPinSet, CanPinSet, SdCardPinSet};
