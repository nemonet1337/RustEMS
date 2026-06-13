//! Nano board HAL — STM32F407 board-specific implementation.
//!
//! Smallest rusEFI board, optimised for single/twin-cylinder engines.
//! Superseal 26-pin connector.

#![no_std]

use embassy_stm32::can::{
    Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::peripherals::{CAN1, PA0, PA1, PC0, PC1, PC3, PD0, PD1};
use embassy_stm32::{bind_interrupts, Peri};
use rusefi_core::sensors::AdcChannel;
use rusefi_hal_stm32_common::board::{
    AdcPinSet, Board, CanPinSet, IgnitionPinSet, SdCardPinSet, TriggerPinSet,
};

// ============================================================================
// Driver Modules
// ============================================================================

pub mod adc;
pub mod can;
pub mod ignition;
#[cfg(feature = "fuel-fi")]
pub mod injector;
pub mod timer;
pub mod trigger;
pub mod uart;

// ============================================================================
// Pin Sets
// ============================================================================

/// ADC pin set for Nano board.
#[allow(dead_code)]
pub struct NanoAdcPins {
    clt: Peri<'static, PA0>,
    iat: Peri<'static, PA1>,
    map: Peri<'static, PC0>,
    vbatt: Peri<'static, PC1>,
    tps: Peri<'static, PC3>,
}

impl NanoAdcPins {
    pub fn new(
        clt: Peri<'static, PA0>,
        iat: Peri<'static, PA1>,
        map: Peri<'static, PC0>,
        vbatt: Peri<'static, PC1>,
        tps: Peri<'static, PC3>,
    ) -> Self {
        Self {
            clt,
            iat,
            map,
            vbatt,
            tps,
        }
    }
}

impl AdcPinSet for NanoAdcPins {
    fn read(&self, _channel: AdcChannel) -> u16 {
        0 // Actual reads through adc::Stm32AdcInput
    }
}

/// Ignition pin set for Nano board (2 cylinders).
pub struct NanoIgnitionPins {
    coils: [bool; 2],
}

impl NanoIgnitionPins {
    pub fn new() -> Self {
        Self { coils: [false; 2] }
    }
}

impl Default for NanoIgnitionPins {
    fn default() -> Self {
        Self::new()
    }
}

impl IgnitionPinSet for NanoIgnitionPins {
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        if (cylinder as usize) < self.coils.len() {
            self.coils[cylinder as usize] = state;
        }
    }
}

/// Trigger pin set for Nano board.
pub struct NanoTriggerPins {
    crank_value: bool,
    cam_value: bool,
}

impl NanoTriggerPins {
    pub fn new() -> Self {
        Self {
            crank_value: false,
            cam_value: false,
        }
    }
}

impl Default for NanoTriggerPins {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerPinSet for NanoTriggerPins {
    fn crank_pin(&self) -> bool {
        self.crank_value
    }

    fn cam_pin(&self) -> bool {
        self.cam_value
    }
}

/// CAN pin set for Nano board (CAN1).
pub struct NanoCanPins {
    rx: Peri<'static, PD0>,
    tx: Peri<'static, PD1>,
}

impl NanoCanPins {
    pub fn new(rx: Peri<'static, PD0>, tx: Peri<'static, PD1>) -> Self {
        Self { rx, tx }
    }
}

bind_interrupts!(struct CanIrqs {
    CAN1_TX  => TxInterruptHandler<CAN1>;
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
});

impl CanPinSet for NanoCanPins {
    type Peripheral = Peri<'static, CAN1>;
    type Can = embassy_stm32::can::Can<'static>;

    fn into_can(self, can: Peri<'static, CAN1>) -> Self::Can {
        Can::new(can, self.rx, self.tx, CanIrqs)
    }
}

/// SD card pin set for Nano board (stub).
pub struct NanoSdCardPins;

impl SdCardPinSet for NanoSdCardPins {}

// ============================================================================
// Board Implementation
// ============================================================================

/// Nano board configuration.
pub struct NanoBoard;

impl Board for NanoBoard {
    type AdcPins = NanoAdcPins;
    type IgnitionPins = NanoIgnitionPins;
    type TriggerPins = NanoTriggerPins;
    type CanPins = NanoCanPins;
    type SdCardPins = NanoSdCardPins;

    const CYLINDER_COUNT: u8 = 2;
    const INJECTOR_COUNT: u8 = 8;
    const IGNITION_COUNT: u8 = 2;
    const HS_OUTPUT_COUNT: u8 = 0;
    const LS_EXTRA_OUTPUT_COUNT: u8 = 0;

    const ADC_GP_COUNT: u8 = 6;
    const ADC_THERM_COUNT: u8 = 0;
    const HALL_INPUT_COUNT: u8 = 2;
    const VR_INPUT_COUNT: u8 = 1;

    const CAN_COUNT: u8 = 1;

    const HAS_INTERNAL_WBO: bool = false;
    const HAS_DUAL_WBO: bool = false;
    const HAS_INTERNAL_KNOCK: bool = false;
    const HAS_DUAL_KNOCK: bool = false;
    const HAS_DUAL_ETB: bool = false;
    const HAS_INTERNAL_BARO: bool = false;
    const HAS_SDCARD: bool = true;
    const HAS_BLUETOOTH: bool = false;
    const HAS_FLEX_FUEL: bool = true;
}
