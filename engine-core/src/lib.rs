//! # rusefi-core
//!
//! `no_std` control logic for the rusEFI Rust firmware implementation.
//!
//! ## Feature flags
//!
//! ### Cylinder count (exactly one required)
//!
//! | Feature   | Cylinders |
//! |-----------|-----------|
//! | `cyl-1`   | 1         |
//! | `cyl-2`   | 2         |
//! | `cyl-3`   | 3         |
//! | `cyl-4`   | 4         |
//! | `cyl-5`   | 5         |
//! | `cyl-6`   | 6         |
//! | `cyl-8`   | 8         |
//! | `cyl-10`  | 10        |
//! | `cyl-12`  | 12        |
//!
//! ### Fuel delivery (exactly one required)
//!
//! | Feature      | Description                      |
//! |-------------|----------------------------------|
//! | `fuel-carb` | Carburetor (no injector control) |
//! | `fuel-fi`   | Fuel injection enabled           |
//!
//! ### Vehicle profile aliases (convenience combinations)
//!
//! `bike-{N}cyl-{carb|fi}` — e.g. `bike-4cyl-fi` enables `cyl-4` + `fuel-fi`.
//!
//! [`MAX_CYLINDERS`] is 12; `firing_order` and all multi-cylinder controllers
//! are sized to this limit at compile time.

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
pub mod comms;
pub mod config;
pub mod engine_cycle;
pub mod hal;
pub mod ignition;
pub mod knock;
pub mod maps;
pub mod outputs;
pub mod params;
pub mod protection;
pub mod sensors;
pub mod shutdown;
pub mod start_stop;
pub mod storage;
pub mod tcu;
pub mod trigger;

#[cfg(feature = "fuel-fi")]
pub mod fuel;
