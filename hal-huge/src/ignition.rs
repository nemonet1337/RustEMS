//! Huge Ignition Output Implementation
//!
//! Controls 4 ignition outputs used by the current firmware entrypoint.

use rusefi_core::hal::IgnitionOutput;
use embassy_stm32::{Peri, gpio::{Level, Output, Speed}};
use embassy_stm32::peripherals::{PE11, PE12, PE13, PE14};

/// Maximum ignition channels for Huge.
pub const IGN_COUNT: usize = 4;

/// Huge Ignition Output driver for up to 4 cylinders.
pub struct Stm32IgnitionOutput {
    coil1: Output<'static>,
    coil2: Output<'static>,
    coil3: Output<'static>,
    coil4: Output<'static>,
}

impl Stm32IgnitionOutput {
    /// Create a new ignition output driver.
    pub fn new(
        pe14: Peri<'static, PE14>,
        pe13: Peri<'static, PE13>,
        pe12: Peri<'static, PE12>,
        pe11: Peri<'static, PE11>,
    ) -> Self {
        let coil1 = Output::new(pe14, Level::High, Speed::High);
        let coil2 = Output::new(pe13, Level::High, Speed::High);
        let coil3 = Output::new(pe12, Level::High, Speed::High);
        let coil4 = Output::new(pe11, Level::High, Speed::High);

        Self { coil1, coil2, coil3, coil4 }
    }

    /// Set coil state (true = charging/low, false = idle/high).
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::Low } else { Level::High };
        match cylinder {
            0 => self.coil1.set_level(level),
            1 => self.coil2.set_level(level),
            2 => self.coil3.set_level(level),
            3 => self.coil4.set_level(level),
            _ => defmt::warn!("Invalid cylinder {} for Huge ignition", cylinder),
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
