//! Simulator ADC input: returns configurable fixed values per channel.

use rusefi_core::hal::AdcInput;
use rusefi_core::sensors::AdcChannel;

/// Simulator implementation of [`AdcInput`].
///
/// Each channel returns a fixed 12-bit ADC value that can be set at runtime.
pub struct SimAdcInput {
    clt_raw: u16,
    iat_raw: u16,
    tps_raw: u16,
    map_raw: u16,
    vbatt_raw: u16,
    maf_raw: u16,
    fuel_level_raw: u16,
    oil_pressure_raw: u16,
    lambda1_raw: u16,
    lambda2_raw: u16,
}

impl SimAdcInput {
    /// Create with default mid-scale values.
    pub fn new() -> Self {
        Self {
            clt_raw: 1200,          // ~80°C on a typical thermistor
            iat_raw: 1600,          // ~25°C intake air
            tps_raw: 200,           // ~5% throttle (nearly closed)
            map_raw: 2000,          // ~50 kPa at idle
            vbatt_raw: 1800,        // ~14.5 V
            maf_raw: 2048,          // mid-scale
            fuel_level_raw: 2048,   // ~50%
            oil_pressure_raw: 2048, // mid-scale
            lambda1_raw: 1250,      // ~2.5V → lambda ~1.0 (wideband stoich)
            lambda2_raw: 1250,
        }
    }

    pub fn set_clt(&mut self, raw: u16) {
        self.clt_raw = raw;
    }
    pub fn set_iat(&mut self, raw: u16) {
        self.iat_raw = raw;
    }
    pub fn set_tps(&mut self, raw: u16) {
        self.tps_raw = raw;
    }
    pub fn set_map(&mut self, raw: u16) {
        self.map_raw = raw;
    }
    pub fn set_vbatt(&mut self, raw: u16) {
        self.vbatt_raw = raw;
    }
    pub fn set_maf(&mut self, raw: u16) {
        self.maf_raw = raw;
    }
    pub fn set_fuel_level(&mut self, raw: u16) {
        self.fuel_level_raw = raw;
    }
    pub fn set_oil_pressure(&mut self, raw: u16) {
        self.oil_pressure_raw = raw;
    }
    pub fn set_lambda1(&mut self, raw: u16) {
        self.lambda1_raw = raw;
    }
    pub fn set_lambda2(&mut self, raw: u16) {
        self.lambda2_raw = raw;
    }
}

impl Default for SimAdcInput {
    fn default() -> Self {
        Self::new()
    }
}

impl AdcInput for SimAdcInput {
    fn read_raw(&mut self, channel: AdcChannel) -> u16 {
        match channel {
            AdcChannel::Clt => self.clt_raw,
            AdcChannel::Iat => self.iat_raw,
            AdcChannel::Tps => self.tps_raw,
            AdcChannel::Map => self.map_raw,
            AdcChannel::Vbatt => self.vbatt_raw,
            AdcChannel::Maf => self.maf_raw,
            AdcChannel::FuelLevel => self.fuel_level_raw,
            AdcChannel::OilPressure => self.oil_pressure_raw,
            AdcChannel::Lambda1 => self.lambda1_raw,
            AdcChannel::Lambda2 => self.lambda2_raw,
        }
    }
}
