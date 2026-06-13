//! Huge Ignition Output Implementation
//!
//! Drives up to 12 ignition coils (one per cylinder) — the Huge board is rated
//! for up to 12 cylinders. Coils idle high and are pulled low while charging.
//!
//! Pin assignment (PE4..PE15) is nominal and must be matched to the production
//! schematic; it is chosen to avoid the ADC (PA/PC), trigger (PA5/PA8) and
//! CAN (PD0/PD1) pins used elsewhere on the board.

use embassy_stm32::peripherals::{
    PE10, PE11, PE12, PE13, PE14, PE15, PE4, PE5, PE6, PE7, PE8, PE9,
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};
use heapless::Vec;
use rusefi_core::config::MAX_CYLINDERS;
use rusefi_core::hal::IgnitionOutput;

/// Maximum ignition channels for Huge (12-cylinder capable).
pub const IGN_COUNT: usize = 12;

/// Huge Ignition Output driver for up to 12 cylinders.
pub struct Stm32IgnitionOutput {
    coils: Vec<Output<'static>, MAX_CYLINDERS>,
}

impl Stm32IgnitionOutput {
    /// Create a new ignition output driver wired to 12 coil GPIOs.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pe4: Peri<'static, PE4>,
        pe5: Peri<'static, PE5>,
        pe6: Peri<'static, PE6>,
        pe7: Peri<'static, PE7>,
        pe8: Peri<'static, PE8>,
        pe9: Peri<'static, PE9>,
        pe10: Peri<'static, PE10>,
        pe11: Peri<'static, PE11>,
        pe12: Peri<'static, PE12>,
        pe13: Peri<'static, PE13>,
        pe14: Peri<'static, PE14>,
        pe15: Peri<'static, PE15>,
    ) -> Self {
        let mut coils = Vec::new();
        // Coils idle high (open); charging pulls the gate low.
        let _ = coils.push(Output::new(pe4, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe5, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe6, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe7, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe8, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe9, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe10, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe11, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe12, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe13, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe14, Level::High, Speed::High));
        let _ = coils.push(Output::new(pe15, Level::High, Speed::High));
        Self { coils }
    }

    /// Set coil state (true = charging/low, false = idle/high).
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        let level = if state { Level::Low } else { Level::High };
        if let Some(coil) = self.coils.get_mut(cylinder as usize) {
            coil.set_level(level);
        } else {
            defmt::warn!("Invalid cylinder {} for Huge ignition", cylinder);
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
