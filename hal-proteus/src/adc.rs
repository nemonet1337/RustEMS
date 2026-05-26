//! Proteus ADC Input Implementation (STM32F7/F4)
//!
//! Uses ADC1/ADC2/ADC3 with up to 16 analogue channels (12 GP + 4 therm).
//! PA0=CLT, PA1=IAT, PC0=MAP, PC1=Vbatt, PC3=TPS

use rusefi_core::hal::AdcInput;
use rusefi_core::sensors::AdcChannel;
use embassy_stm32::{Peri, adc::{Adc, SampleTime}};
use embassy_stm32::peripherals::{ADC1, PA0, PA1, PC0, PC1, PC3};

/// Proteus ADC Input driver.
pub struct Stm32AdcInput {
    adc: Adc<'static, ADC1>,
    clt_pin:   Peri<'static, PA0>,
    iat_pin:   Peri<'static, PA1>,
    map_pin:   Peri<'static, PC0>,
    vbatt_pin: Peri<'static, PC1>,
    tps_pin:   Peri<'static, PC3>,
}

impl Stm32AdcInput {
    pub fn new(
        adc1: Peri<'static, ADC1>,
        pa0:  Peri<'static, PA0>,
        pa1:  Peri<'static, PA1>,
        pc3:  Peri<'static, PC3>,
        pc0:  Peri<'static, PC0>,
        pc1:  Peri<'static, PC1>,
    ) -> Self {
        let adc = Adc::new(adc1);
        Self {
            adc,
            clt_pin: pa0,
            iat_pin: pa1,
            map_pin: pc0,
            vbatt_pin: pc1,
            tps_pin: pc3,
        }
    }
}

impl AdcInput for Stm32AdcInput {
    fn read_raw(&mut self, channel: AdcChannel) -> u16 {
        match channel {
            AdcChannel::Clt   => self.adc.blocking_read(&mut self.clt_pin,   SampleTime::CYCLES480),
            AdcChannel::Iat   => self.adc.blocking_read(&mut self.iat_pin,   SampleTime::CYCLES480),
            AdcChannel::Tps   => self.adc.blocking_read(&mut self.tps_pin,   SampleTime::CYCLES480),
            AdcChannel::Map   => self.adc.blocking_read(&mut self.map_pin,   SampleTime::CYCLES480),
            AdcChannel::Vbatt => self.adc.blocking_read(&mut self.vbatt_pin, SampleTime::CYCLES480),
            _ => 0, // Additional channels via ADC2/3 in future
        }
    }
}
