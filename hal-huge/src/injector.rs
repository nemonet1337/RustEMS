//! Huge Injector Output Implementation
//!
//! Drives up to 12 injectors (one per cylinder) for full sequential injection
//! on the 12-cylinder-capable Huge board. Injectors idle low and are driven
//! high while open.
//!
//! Pin assignment (PF0..PF11) is nominal and must be matched to the production
//! schematic; it avoids the ADC (PA/PC), trigger (PA5/PA8), CAN (PD0/PD1) and
//! ignition (PE4..PE15) pins used elsewhere on the board.

use rusefi_core::config::MAX_CYLINDERS;
use rusefi_core::hal::InjectorOutput;
use embassy_stm32::{Peri, gpio::{Level, Output, Speed}};
use embassy_stm32::peripherals::{
    PF0, PF1, PF10, PF11, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9,
};
use heapless::Vec;

/// Maximum injector channels for Huge (12-cylinder capable).
pub const INJ_COUNT: usize = 12;

/// Huge Injector Output driver for up to 12 cylinders.
pub struct Stm32InjectorOutput {
    injectors: Vec<Output<'static>, MAX_CYLINDERS>,
}

impl Stm32InjectorOutput {
    /// Create a new injector output driver wired to 12 injector GPIOs.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pf0: Peri<'static, PF0>,
        pf1: Peri<'static, PF1>,
        pf2: Peri<'static, PF2>,
        pf3: Peri<'static, PF3>,
        pf4: Peri<'static, PF4>,
        pf5: Peri<'static, PF5>,
        pf6: Peri<'static, PF6>,
        pf7: Peri<'static, PF7>,
        pf8: Peri<'static, PF8>,
        pf9: Peri<'static, PF9>,
        pf10: Peri<'static, PF10>,
        pf11: Peri<'static, PF11>,
    ) -> Self {
        let mut injectors = Vec::new();
        let _ = injectors.push(Output::new(pf0, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf1, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf2, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf3, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf4, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf5, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf6, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf7, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf8, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf9, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf10, Level::Low, Speed::High));
        let _ = injectors.push(Output::new(pf11, Level::Low, Speed::High));
        Self { injectors }
    }

    fn set_injector(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::High } else { Level::Low };
        if let Some(inj) = self.injectors.get_mut(cylinder as usize) {
            inj.set_level(level);
        } else {
            defmt::warn!("Invalid cylinder {} for Huge injector", cylinder);
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
