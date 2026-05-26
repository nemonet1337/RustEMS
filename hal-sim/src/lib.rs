//! PC simulator HAL implementations of the `rusefi-core` HAL traits.
//!
//! This crate provides `std`-based (heap-capable) implementations suitable
//! for PC-side simulation and testing.  It is **not** `no_std`.

pub mod trigger_sim;
pub mod ignition_sim;
pub mod adc_sim;
pub mod timer_sim;
pub mod can_sim;
pub mod uart_sim;
#[cfg(feature = "fuel-fi")]
pub mod injector_sim;

pub use trigger_sim::SimTriggerInput;
pub use ignition_sim::SimIgnitionOutput;
pub use adc_sim::SimAdcInput;
pub use timer_sim::SimSystemTimer;
pub use can_sim::SimCanBus;
pub use uart_sim::SimUartPort;
#[cfg(feature = "fuel-fi")]
pub use injector_sim::SimInjectorOutput;
