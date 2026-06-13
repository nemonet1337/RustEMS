//! Nano Ignition Output Implementation
//!
//! Controls 2 ignition coils (smart/dumb, active-low outputs).
//! PE14 = IGN1, PE13 = IGN2

use embassy_stm32::peripherals::{PE13, PE14};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};
use rusefi_core::hal::IgnitionOutput;

/// Nano Ignition Output driver for 2 cylinders.
pub struct Stm32IgnitionOutput {
    coil1: Output<'static>,
    coil2: Output<'static>,
}

impl Stm32IgnitionOutput {
    /// Create a new ignition output driver.
    pub fn new(pe14: Peri<'static, PE14>, pe13: Peri<'static, PE13>) -> Self {
        let coil1 = Output::new(pe14, Level::High, Speed::High);
        let coil2 = Output::new(pe13, Level::High, Speed::High);
        Self { coil1, coil2 }
    }

    /// Set coil state (true = charging/low, false = idle/high).
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::Low } else { Level::High };
        // Two physical coils; cylinders are grouped for wasted-spark / batch
        // operation (4-cylinder batch maps onto the two coils).
        match (cylinder as usize) % 2 {
            0 => self.coil1.set_level(level),
            _ => self.coil2.set_level(level),
        }
    }
}

impl IgnitionOutput for Stm32IgnitionOutput {
    fn coil_charge(&mut self, cylinder: u8) {
        self.set_coil(cylinder, true);
        defmt::trace!("Coil {} charging", cylinder);
    }

    fn coil_fire(&mut self, cylinder: u8) {
        self.set_coil(cylinder, false);
        defmt::trace!("Coil {} fired", cylinder);
    }
}
