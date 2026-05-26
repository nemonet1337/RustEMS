//! rusEFI Huge HAL — STM32F4 board-specific implementation
//!
//! Top-tier ECU for up to 12 cylinders with dual WBO/knock and Bluetooth.
//! Superseal 120-pin connector, optional waterproofing.

#![no_std]

use embassy_stm32::{Peri, bind_interrupts};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::peripherals::{CAN1, PA0, PA1, PC0, PC1, PC3, PD0, PD1};
use rusefi_core::sensors::AdcChannel;
use rusefi_hal_stm32_common::board::{Board, AdcPinSet, IgnitionPinSet, TriggerPinSet, CanPinSet, SdCardPinSet};

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

/// Huge board ADC pins
pub struct HugeAdcPins {
    clt: Peri<'static, PA0>,
    iat: Peri<'static, PA1>,
    map: Peri<'static, PC0>,
    vbatt: Peri<'static, PC1>,
    tps: Peri<'static, PC3>,
}

impl HugeAdcPins {
    pub fn new(
        clt: Peri<'static, PA0>,
        iat: Peri<'static, PA1>,
        map: Peri<'static, PC0>,
        vbatt: Peri<'static, PC1>,
        tps: Peri<'static, PC3>,
    ) -> Self {
        Self { clt, iat, map, vbatt, tps }
    }
}

impl AdcPinSet for HugeAdcPins {
    fn read(&self, _channel: AdcChannel) -> u16 {
        0 // Actual reads through adc::Stm32AdcInput
    }
}

/// Huge board ignition pins (up to 12 cylinders)
pub struct HugeIgnitionPins {
    coils: [bool; 12],
}

impl HugeIgnitionPins {
    pub fn new() -> Self {
        Self { coils: [false; 12] }
    }
}

impl Default for HugeIgnitionPins {
    fn default() -> Self {
        Self::new()
    }
}

impl IgnitionPinSet for HugeIgnitionPins {
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        if (cylinder as usize) < self.coils.len() {
            self.coils[cylinder as usize] = state;
        }
    }
}

/// Huge board trigger pins
pub struct HugeTriggerPins {
    crank_value: bool,
    cam_value: bool,
}

impl HugeTriggerPins {
    pub fn new() -> Self {
        Self {
            crank_value: false,
            cam_value: false,
        }
    }
}

impl Default for HugeTriggerPins {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerPinSet for HugeTriggerPins {
    fn crank_pin(&self) -> bool {
        self.crank_value
    }

    fn cam_pin(&self) -> bool {
        self.cam_value
    }
}

/// Huge board CAN pins (dual CAN)
pub struct HugeCanPins {
    can1_rx: Peri<'static, PD0>,
    can1_tx: Peri<'static, PD1>,
}

impl HugeCanPins {
    pub fn new(can1_rx: Peri<'static, PD0>, can1_tx: Peri<'static, PD1>) -> Self {
        Self { can1_rx, can1_tx }
    }
}

bind_interrupts!(struct CanIrqs {
    CAN1_TX  => TxInterruptHandler<CAN1>;
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
});

impl CanPinSet for HugeCanPins {
    type Peripheral = Peri<'static, CAN1>;
    type Can = embassy_stm32::can::Can<'static>;

    fn into_can(self, can: Peri<'static, CAN1>) -> Self::Can {
        Can::new(can, self.can1_rx, self.can1_tx, CanIrqs)
    }
}

/// SD card pin set for Huge board (stub - not yet implemented).
pub struct HugeSdCardPins;

impl SdCardPinSet for HugeSdCardPins {}

// ─── Board implementation ─────────────────────────────────────────────────

/// Huge board implementation
pub struct HugeBoard;

impl Board for HugeBoard {
    type AdcPins = HugeAdcPins;
    type IgnitionPins = HugeIgnitionPins;
    type TriggerPins = HugeTriggerPins;
    type CanPins = HugeCanPins;
    type SdCardPins = HugeSdCardPins;

    const CYLINDER_COUNT: u8 = 12;
    const INJECTOR_COUNT: u8 = 12;
    const IGNITION_COUNT: u8 = 12;
    const HS_OUTPUT_COUNT: u8 = 0;
    const LS_EXTRA_OUTPUT_COUNT: u8 = 4;

    const ADC_GP_COUNT: u8 = 13;
    const ADC_THERM_COUNT: u8 = 2;
    const HALL_INPUT_COUNT: u8 = 5;
    const VR_INPUT_COUNT: u8 = 3;

    const CAN_COUNT: u8 = 2;

    const HAS_INTERNAL_WBO: bool = false;
    const HAS_DUAL_WBO: bool = true;
    const HAS_INTERNAL_KNOCK: bool = false;
    const HAS_DUAL_KNOCK: bool = true;
    const HAS_DUAL_ETB: bool = true;
    const HAS_INTERNAL_BARO: bool = true;
    const HAS_SDCARD: bool = true;
    const HAS_BLUETOOTH: bool = true;
    const HAS_FLEX_FUEL: bool = true;
}
