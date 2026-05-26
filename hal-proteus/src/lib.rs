//! Proteus board HAL — STM32F767 (F7) / STM32F407 (F4) board-specific implementation.
//!
//! High-output ECU for up to 12 cylinders, with IP68 waterproofing.
//! TE Ampseal 93-pin connector, 135x82.5mm 4-layer PCB.

#![no_std]

use embassy_stm32::{Peri, bind_interrupts};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::peripherals::{CAN1, PA0, PA1, PC0, PC1, PC3, PD0, PD1};
use rusefi_core::sensors::AdcChannel;
use rusefi_hal_stm32_common::board::{AdcPinSet, Board, CanPinSet, IgnitionPinSet, SdCardPinSet, TriggerPinSet};

// ============================================================================
// Driver Modules
// ============================================================================

pub mod adc;
pub mod can;
pub mod ignition;
pub mod injector;
pub mod timer;
pub mod trigger;

// ============================================================================
// Pin Sets
// ============================================================================

/// ADC pin set for Proteus board (12 GP + 4 therm via ADC1 and ADC2/3).
pub struct ProteusAdcPins {
    clt:   Peri<'static, PA0>,
    iat:   Peri<'static, PA1>,
    map:   Peri<'static, PC0>,
    vbatt: Peri<'static, PC1>,
    tps:   Peri<'static, PC3>,
}

impl ProteusAdcPins {
    pub fn new(
        clt:   Peri<'static, PA0>,
        iat:   Peri<'static, PA1>,
        map:   Peri<'static, PC0>,
        vbatt: Peri<'static, PC1>,
        tps:   Peri<'static, PC3>,
    ) -> Self {
        Self { clt, iat, map, vbatt, tps }
    }
}

impl AdcPinSet for ProteusAdcPins {
    fn read(&self, _channel: AdcChannel) -> u16 {
        0 // Actual reads through adc::Stm32AdcInput
    }
}

/// Ignition pin set for Proteus board (12 cylinders, 5V/100mA logic level).
pub struct ProteusIgnitionPins {
    coils: [bool; 12],
}

impl ProteusIgnitionPins {
    pub fn new() -> Self {
        Self { coils: [false; 12] }
    }
}

impl Default for ProteusIgnitionPins {
    fn default() -> Self {
        Self::new()
    }
}

impl IgnitionPinSet for ProteusIgnitionPins {
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        if (cylinder as usize) < self.coils.len() {
            self.coils[cylinder as usize] = state;
        }
    }
}

/// Trigger pin set for Proteus board (up to 6 Hall + 2 VR).
pub struct ProteusTriggerPins {
    crank_value: bool,
    cam_value: bool,
}

impl ProteusTriggerPins {
    pub fn new() -> Self {
        Self {
            crank_value: false,
            cam_value: false,
        }
    }
}

impl Default for ProteusTriggerPins {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerPinSet for ProteusTriggerPins {
    fn crank_pin(&self) -> bool {
        self.crank_value
    }

    fn cam_pin(&self) -> bool {
        self.cam_value
    }
}

/// CAN1 pin set for Proteus board.
pub struct ProteusCan1Pins {
    rx: Peri<'static, PD0>,
    tx: Peri<'static, PD1>,
}

impl ProteusCan1Pins {
    pub fn new(rx: Peri<'static, PD0>, tx: Peri<'static, PD1>) -> Self {
        Self { rx, tx }
    }
}

bind_interrupts!(struct Can1Irqs {
    CAN1_TX  => TxInterruptHandler<CAN1>;
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
});

impl CanPinSet for ProteusCan1Pins {
    type Peripheral = Peri<'static, CAN1>;
    type Can = embassy_stm32::can::Can<'static>;

    fn into_can(self, can: Peri<'static, CAN1>) -> Self::Can {
        Can::new(can, self.rx, self.tx, Can1Irqs)
    }
}

/// SD card pin set for Proteus board (stub).
pub struct ProteusSdCardPins;

impl SdCardPinSet for ProteusSdCardPins {}

// ============================================================================
// Board Implementation
// ============================================================================

/// Proteus board configuration.
pub struct ProteusBoard;

impl Board for ProteusBoard {
    type AdcPins = ProteusAdcPins;
    type IgnitionPins = ProteusIgnitionPins;
    type TriggerPins = ProteusTriggerPins;
    type CanPins = ProteusCan1Pins;
    type SdCardPins = ProteusSdCardPins;

    const CYLINDER_COUNT: u8 = 12;
    const INJECTOR_COUNT: u8 = 16;
    const IGNITION_COUNT: u8 = 12;
    const HS_OUTPUT_COUNT: u8 = 4;
    const LS_EXTRA_OUTPUT_COUNT: u8 = 8;

    const ADC_GP_COUNT: u8 = 12;
    const ADC_THERM_COUNT: u8 = 4;
    const HALL_INPUT_COUNT: u8 = 6;
    const VR_INPUT_COUNT: u8 = 2;

    const CAN_COUNT: u8 = 2;

    const HAS_INTERNAL_WBO: bool = true;
    const HAS_DUAL_WBO: bool = false;
    const HAS_INTERNAL_KNOCK: bool = true;
    const HAS_DUAL_KNOCK: bool = false;
    const HAS_DUAL_ETB: bool = true;
    const HAS_INTERNAL_BARO: bool = true;
    const HAS_SDCARD: bool = true;
    const HAS_BLUETOOTH: bool = false;
    const HAS_FLEX_FUEL: bool = true;
}
