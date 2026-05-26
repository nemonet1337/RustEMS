//! Proteus Injector Output Implementation
//!
//! Controls 4 injector outputs used by the current firmware entrypoint.

use rusefi_core::hal::InjectorOutput;
use embassy_stm32::{Peri, gpio::{Level, Output, Speed}};
use embassy_stm32::peripherals::{PB8, PB9, PD14, PD15};

/// Maximum injector channels for Proteus.
pub const INJ_COUNT: usize = 4;

/// Proteus Injector Output driver for up to 4 channels.
pub struct Stm32InjectorOutput {
    inj1: Output<'static>,
    inj2: Output<'static>,
    inj3: Output<'static>,
    inj4: Output<'static>,
}

impl Stm32InjectorOutput {
    pub fn new(
        pb9:  Peri<'static, PB9>,
        pb8:  Peri<'static, PB8>,
        pd15: Peri<'static, PD15>,
        pd14: Peri<'static, PD14>,
    ) -> Self {
        Self {
            inj1: Output::new(pb9,  Level::Low, Speed::High),
            inj2: Output::new(pb8,  Level::Low, Speed::High),
            inj3: Output::new(pd15, Level::Low, Speed::High),
            inj4: Output::new(pd14, Level::Low, Speed::High),
        }
    }

    fn set_injector(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::High } else { Level::Low };
        match cylinder {
            0 => self.inj1.set_level(level),
            1 => self.inj2.set_level(level),
            2 => self.inj3.set_level(level),
            3 => self.inj4.set_level(level),
            _ => defmt::warn!("Invalid cylinder {} for Proteus injector", cylinder),
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
