//! UAEFI Injector Output Implementation
//!
//! Controls 6 fuel injectors.
//! PB9=INJ1, PB8=INJ2, PD15=INJ3, PD14=INJ4, PD13=INJ5, PD12=INJ6

use embassy_stm32::peripherals::{PB8, PB9, PD12, PD13, PD14, PD15};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};
use rusefi_core::hal::InjectorOutput;

/// UAEFI Injector Output driver for 6 channels.
pub struct Stm32InjectorOutput {
    inj1: Output<'static>,
    inj2: Output<'static>,
    inj3: Output<'static>,
    inj4: Output<'static>,
    inj5: Output<'static>,
    inj6: Output<'static>,
}

impl Stm32InjectorOutput {
    /// Create a new 6-channel injector output driver.
    pub fn new(
        pb9: Peri<'static, PB9>,
        pb8: Peri<'static, PB8>,
        pd15: Peri<'static, PD15>,
        pd14: Peri<'static, PD14>,
        pd13: Peri<'static, PD13>,
        pd12: Peri<'static, PD12>,
    ) -> Self {
        Self {
            inj1: Output::new(pb9, Level::Low, Speed::High),
            inj2: Output::new(pb8, Level::Low, Speed::High),
            inj3: Output::new(pd15, Level::Low, Speed::High),
            inj4: Output::new(pd14, Level::Low, Speed::High),
            inj5: Output::new(pd13, Level::Low, Speed::High),
            inj6: Output::new(pd12, Level::Low, Speed::High),
        }
    }

    /// Set injector state (true = open/high, false = closed/low).
    fn set_injector(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::High } else { Level::Low };
        match cylinder {
            0 => self.inj1.set_level(level),
            1 => self.inj2.set_level(level),
            2 => self.inj3.set_level(level),
            3 => self.inj4.set_level(level),
            4 => self.inj5.set_level(level),
            5 => self.inj6.set_level(level),
            _ => defmt::warn!("Invalid cylinder {} for UAEFI injector (max 6)", cylinder),
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
