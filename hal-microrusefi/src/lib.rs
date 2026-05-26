//! microRusEFI HAL — STM32F407 board-specific implementation.
//!
//! Pin mapping (from microRusEFI board):
//! | Function | MCU pin | Note |
//! |----------|---------|------|
//! | Crank VR/Hall | PA8 (TIM1_CH1) | EXTI or input capture |
//! | Cam Hall | PA5 | EXTI |
//! | IGN 1 | PE14 | GPIO output |
//! | IGN 2 | PE13 | GPIO output |
//! | IGN 3 | PE12 | GPIO output |
//! | IGN 4 | PE11 | GPIO output |
//! | INJ 1 | PB9  | GPIO output |
//! | INJ 2 | PB8  | GPIO output |
//! | INJ 3 | PD15 | GPIO output |
//! | INJ 4 | PD14 | GPIO output |
//! | CLT    | PA0 (ADC1_IN0) | 2.7 kΩ pull-up to 5 V |
//! | IAT    | PA1 (ADC1_IN1) | 2.7 kΩ pull-up to 5 V |
//! | TPS    | PC3 (ADC1_IN13) | 0–5 V linear |
//! | MAP    | PC0 (ADC1_IN10) | 0–5 V linear |
//! | Vbatt  | PC1 (ADC1_IN11) | Divider ÷5.7 |
//! | CAN1_TX  | PD1     | AF9 |
//! | CAN1_RX  | PD0     | AF9 |
//! | USART3_TX | PD8    | AF7 |
//! | USART3_RX | PD9    | AF7 |

#![no_std]

use embassy_stm32::{Peri, bind_interrupts};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::peripherals::{CAN1, PA0, PA1, PC0, PC1, PC3, PD0, PD1};
use rusefi_core::sensors::AdcChannel;
use rusefi_hal_stm32_common::board::{Board, AdcPinSet, IgnitionPinSet, TriggerPinSet, CanPinSet, SdCardPinSet};

// ============================================================================
// microRusEFI Pin Sets
// ============================================================================

/// ADC pin set for microRusEFI.
pub struct MicroRusEFIAdcPins {
    clt: Peri<'static, PA0>,
    iat: Peri<'static, PA1>,
    map: Peri<'static, PC0>,
    vbatt: Peri<'static, PC1>,
    tps: Peri<'static, PC3>,
}

impl MicroRusEFIAdcPins {
    /// Create a new ADC pin set.
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

impl AdcPinSet for MicroRusEFIAdcPins {
    fn read(&self, _channel: AdcChannel) -> u16 {
        0
    }
}

/// Ignition pin set for microRusEFI (4 cylinders).
pub struct MicroRusEFIIgnitionPins {
    coils: [bool; 4],
}

impl MicroRusEFIIgnitionPins {
    /// Create a new ignition pin set.
    pub fn new() -> Self {
        Self { coils: [false; 4] }
    }
}

impl IgnitionPinSet for MicroRusEFIIgnitionPins {
    fn set_coil(&mut self, cylinder: u8, state: bool) {
        if (cylinder as usize) < self.coils.len() {
            self.coils[cylinder as usize] = state;
        }
    }
}

/// Trigger pin set for microRusEFI.
pub struct MicroRusEFITriggerPins {
    crank_value: bool,
    cam_value: bool,
}

impl MicroRusEFITriggerPins {
    /// Create a new trigger pin set.
    pub fn new() -> Self {
        Self { crank_value: false, cam_value: false }
    }
}

impl TriggerPinSet for MicroRusEFITriggerPins {
    fn crank_pin(&self) -> bool {
        self.crank_value
    }

    fn cam_pin(&self) -> bool {
        self.cam_value
    }
}

/// CAN pin set for microRusEFI (CAN1 only).
pub struct MicroRusEFICanPins {
    rx: Peri<'static, PD0>,
    tx: Peri<'static, PD1>,
}

impl MicroRusEFICanPins {
    /// Create a new CAN pin set.
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

impl CanPinSet for MicroRusEFICanPins {
    type Peripheral = Peri<'static, CAN1>;
    type Can = embassy_stm32::can::Can<'static>;

    fn into_can(self, can: Peri<'static, CAN1>) -> Self::Can {
        Can::new(can, self.rx, self.tx, CanIrqs)
    }
}

/// SD card pin set for microRusEFI (stub - not yet implemented).
pub struct MicroRusEFISdCardPins;

impl SdCardPinSet for MicroRusEFISdCardPins {}

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
// Board Implementation
// ============================================================================

/// microRusEFI board configuration.
pub struct MicroRusEFIBoard;

impl Board for MicroRusEFIBoard {
    type AdcPins = MicroRusEFIAdcPins;
    type IgnitionPins = MicroRusEFIIgnitionPins;
    type TriggerPins = MicroRusEFITriggerPins;
    type CanPins = MicroRusEFICanPins;
    type SdCardPins = MicroRusEFISdCardPins;

    const CYLINDER_COUNT: u8 = 4;
    const INJECTOR_COUNT: u8 = 4;
    const IGNITION_COUNT: u8 = 4;
    const HS_OUTPUT_COUNT: u8 = 0;
    const LS_EXTRA_OUTPUT_COUNT: u8 = 0;

    const ADC_GP_COUNT: u8 = 10;
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
