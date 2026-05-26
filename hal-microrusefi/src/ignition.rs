//! microRusEFI Ignition Output Implementation
//!
//! Controls 4 ignition coils (active-low outputs).
//! PE14 = IGN1, PE13 = IGN2, PE12 = IGN3, PE11 = IGN4

use rusefi_core::hal::IgnitionOutput;
use embassy_stm32::{Peri, gpio::{Level, Output, Speed}};
use embassy_stm32::peripherals::{PE11, PE12, PE13, PE14};

/// microRusEFI Ignition Output driver for 4 cylinders.
pub struct Stm32IgnitionOutput {
    coil1: Output<'static>,
    coil2: Output<'static>,
    coil3: Output<'static>,
    coil4: Output<'static>,
}

impl Stm32IgnitionOutput {
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

    fn set_coil(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::Low } else { Level::High };
        match cylinder {
            0 => self.coil1.set_level(level),
            1 => self.coil2.set_level(level),
            2 => self.coil3.set_level(level),
            3 => self.coil4.set_level(level),
            _ => defmt::warn!("Invalid cylinder {} for ignition", cylinder),
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
