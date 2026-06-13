//! Nano Injector Output Implementation
//!
//! The Nano is a 1–2 cylinder board with 2 low-side injector channels. For
//! 4-cylinder batch operation the cylinders are grouped onto the two physical
//! channels (`cylinder % 2`), matching the real Hellen Nano hardware
//! (PB9 = INJ1, PB8 = INJ2). Injectors idle low and are driven high while open.

use embassy_stm32::peripherals::{PB8, PB9};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};
use rusefi_core::hal::InjectorOutput;

/// Physical injector channels on the Nano.
pub const INJ_COUNT: usize = 2;

pub struct Stm32InjectorOutput {
    inj1: Output<'static>,
    inj2: Output<'static>,
}

impl Stm32InjectorOutput {
    pub fn new(pb9: Peri<'static, PB9>, pb8: Peri<'static, PB8>) -> Self {
        Self {
            inj1: Output::new(pb9, Level::Low, Speed::High),
            inj2: Output::new(pb8, Level::Low, Speed::High),
        }
    }

    fn set_injector(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::High } else { Level::Low };
        // Two physical channels; cylinders are grouped for batch injection.
        match (cylinder as usize) % INJ_COUNT {
            0 => self.inj1.set_level(level),
            _ => self.inj2.set_level(level),
        }
    }
}

impl InjectorOutput for Stm32InjectorOutput {
    fn open(&mut self, cylinder: u8) {
        self.set_injector(cylinder, true);
        defmt::trace!("Injector {} opened", cylinder);
    }

    fn close(&mut self, cylinder: u8) {
        self.set_injector(cylinder, false);
        defmt::trace!("Injector {} closed", cylinder);
    }
}
