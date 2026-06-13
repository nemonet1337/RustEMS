//! Actuator control modules — PID-based control for engine accessories.
//!
//! This module implements control algorithms for:
//! - Idle speed control via IAC valve
//! - Boost control via wastegate solenoid
//! - Variable valve timing (VVT) via cam phasers
//! - Electronic throttle body (ETB / drive-by-wire)
//! - Generic PWM output and auxiliary PID control

pub mod boost;
pub mod etb;
pub mod idle;
pub mod pwm;
pub mod vvt;

pub use boost::{BoostConfig, BoostController, BoostState};
pub use etb::{EtbConfig, EtbController, EtbFault, EtbOutput};
pub use idle::{IdleConfig, IdleController, IdleState};
pub use pwm::{AuxPidConfig, AuxPidController};
pub use vvt::{DualVvtController, VvtController, VvtMode, VvtOutputConfig};
