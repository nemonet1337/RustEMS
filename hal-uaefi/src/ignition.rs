//! UAEFI Ignition Output Implementation
//!
//! Controls 6 ignition coils (Smart ignition, active-low outputs).
//! PE14 = IGN1, PE13 = IGN2, PE12 = IGN3, PE11 = IGN4, PE10 = IGN5, PE9 = IGN6

use embassy_stm32::peripherals::{PE10, PE11, PE12, PE13, PE14, PE9};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};
use rusefi_core::hal::IgnitionOutput;

/// UAEFI Ignition Output driver for 6 cylinders.
pub struct Stm32IgnitionOutput {
    coil1: Output<'static>,
    coil2: Output<'static>,
    coil3: Output<'static>,
    coil4: Output<'static>,
    coil5: Output<'static>,
    coil6: Output<'static>,
}

impl Stm32IgnitionOutput {
    /// Create a new ignition output driver.
    pub fn new(
        pe14: Peri<'static, PE14>,
        pe13: Peri<'static, PE13>,
        pe12: Peri<'static, PE12>,
        pe11: Peri<'static, PE11>,
        pe10: Peri<'static, PE10>,
        pe9: Peri<'static, PE9>,
    ) -> Self {
        Self {
            coil1: Output::new(pe14, Level::High, Speed::High),
            coil2: Output::new(pe13, Level::High, Speed::High),
            coil3: Output::new(pe12, Level::High, Speed::High),
            coil4: Output::new(pe11, Level::High, Speed::High),
            coil5: Output::new(pe10, Level::High, Speed::High),
            coil6: Output::new(pe9, Level::High, Speed::High),
        }
    }

    /// Set coil state (true = charging/low, false = idle/high).
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::Low } else { Level::High };
        match cylinder {
            0 => self.coil1.set_level(level),
            1 => self.coil2.set_level(level),
            2 => self.coil3.set_level(level),
            3 => self.coil4.set_level(level),
            4 => self.coil5.set_level(level),
            5 => self.coil6.set_level(level),
            _ => defmt::warn!("Invalid cylinder {} for UAEFI ignition (max 6)", cylinder),
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
