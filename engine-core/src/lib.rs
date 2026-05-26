//! # rusefi-core
//!
//! `no_std` control logic for the rusEFI Rust firmware implementation.
//!
//! ## Feature flags
//!
//! | Feature      | Description                        |
//! |-------------|------------------------------------|
//! | `cyl-1`     | Single-cylinder engine             |
//! | `cyl-2`     | Two-cylinder engine                |
//! | `cyl-3`     | Three-cylinder engine              |
//! | `cyl-4`     | Four-cylinder engine               |
//! | `fuel-carb` | Carburetor (no injector control)   |
//! | `fuel-fi`   | Fuel injection enabled             |
//!
//! Exactly one `cyl-N` and one `fuel-*` feature must be enabled.

#![no_std]
#![deny(missing_docs)]

// Compile-time enforcement: at least one fuel mode must be selected.
#[cfg(not(any(feature = "fuel-carb", feature = "fuel-fi")))]
compile_error!("Exactly one of fuel-carb or fuel-fi features must be enabled");

#[cfg(all(feature = "fuel-carb", feature = "fuel-fi"))]
compile_error!("Only one of fuel-carb or fuel-fi may be enabled at a time");

pub mod actuators;
pub mod bootloader;
pub mod can;
pub mod config;
pub mod engine_cycle;
pub mod hal;
pub mod ignition;
pub mod knock;
pub mod maps;
pub mod outputs;
pub mod protection;
pub mod sensors;
pub mod shutdown;
pub mod start_stop;
pub mod storage;
pub mod tcu;
pub mod trigger;

#[cfg(feature = "fuel-fi")]
pub mod fuel;
