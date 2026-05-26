//! Lookup table support for ignition, fuel, and sensor correction maps.

pub mod interpolation;

pub use interpolation::{interpolate1d, interpolate2d, lerp};
