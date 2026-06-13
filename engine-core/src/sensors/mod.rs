//! Sensor conversion — raw ADC counts → physical units.
//!
//! Implements:
//! - Steinhart-Hart thermistor equation (`thermistor_func.cpp`)
//! - Linear conversion for TPS, MAP, etc. (`linear_func.cpp`)
//! Sensor processing — ADC conversion, thermistors, linear sensors, filtering.

use libm::{logf, powf};

pub mod heater;
pub use heater::{HeaterConfig, HeaterController, HeaterPhase};

/// Result type for sensor readings with range validation.
///
/// Similar to `Option<f32>` but distinguishes between:
/// - Valid reading in normal range
/// - Reading below minimum (open circuit / sensor disconnected)
/// - Reading above maximum (shorted to power / sensor fault)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorResult<T> {
    /// Valid sensor reading within expected range.
    Ok(T),
    /// Reading below minimum valid value (likely open circuit).
    TooLow,
    /// Reading above maximum valid value (likely shorted to power).
    TooHigh,
    /// No reading available (sensor not configured or disabled).
    None,
}

impl<T> SensorResult<T> {
    /// Returns true if the result is valid (Ok variant).
    pub fn is_valid(&self) -> bool {
        matches!(self, SensorResult::Ok(_))
    }

    /// Returns the value if valid, otherwise returns None.
    pub fn value(&self) -> Option<&T> {
        match self {
            SensorResult::Ok(v) => Some(v),
            _ => None,
        }
    }

    /// Converts to Option<T>, discarding error information.
    pub fn ok(self) -> Option<T> {
        match self {
            SensorResult::Ok(v) => Some(v),
            _ => None,
        }
    }

    /// Maps a `SensorResult<T>` to `SensorResult<U>` by applying a function.
    pub fn map<U, F: FnOnce(T) -> U>(self, f: F) -> SensorResult<U> {
        match self {
            SensorResult::Ok(v) => SensorResult::Ok(f(v)),
            SensorResult::TooLow => SensorResult::TooLow,
            SensorResult::TooHigh => SensorResult::TooHigh,
            SensorResult::None => SensorResult::None,
        }
    }
}

impl SensorResult<f32> {
    /// Returns the value or a default if not valid.
    pub fn unwrap_or(self, default: f32) -> f32 {
        match self {
            SensorResult::Ok(v) => v,
            _ => default,
        }
    }
}

/// ADC channel identifiers for microRusEFI (STM32F407, 3.3 V reference).
///
/// Pin mapping: `docs/sensor-map.md` § 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdcChannel {
    /// PA0 / EFI_ADC_0 — Coolant temperature (CLT).
    Clt,
    /// PA1 / EFI_ADC_1 — Intake air temperature (IAT).
    Iat,
    /// PC3 / EFI_ADC_13 — Throttle position (TPS).
    Tps,
    /// PC0 / EFI_ADC_10 — Manifold absolute pressure (MAP).
    Map,
    /// PC1 / EFI_ADC_11 — Battery voltage.
    Vbatt,
    /// PA2 / EFI_ADC_2 — Mass Air Flow (MAF) sensor.
    Maf,
    /// PA3 / EFI_ADC_3 — Fuel level sensor.
    FuelLevel,
    /// PA4 / EFI_ADC_4 — Oil pressure sensor.
    OilPressure,
    /// PA5 / EFI_ADC_5 — Lambda 1 (primary O2) sensor.
    Lambda1,
    /// PA6 / EFI_ADC_6 — Lambda 2 (secondary O2) sensor.
    Lambda2,
}

/// All current sensor readings used by the control loop.
///
/// `None` values indicate sensor failure or absence.
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorData {
    /// Engine speed in RPM.
    pub rpm: Option<f32>,
    /// Engine load as a percentage (0–100 %).
    pub load_pct: Option<f32>,
    /// Coolant temperature in °C.
    pub clt_celsius: Option<f32>,
    /// Intake air temperature in °C.
    pub iat_celsius: Option<f32>,
    /// Throttle position in % (0 = fully closed, 100 = fully open).
    pub tps_pct: Option<f32>,
    /// Manifold absolute pressure in kPa.
    pub map_kpa: Option<f32>,
    /// Battery voltage in V.
    pub battery_volts: Option<f32>,
    /// Mass Air Flow sensor voltage (0-5V).
    pub maf_voltage: Option<f32>,
    /// Fuel level sensor (0-100% or raw value).
    pub fuel_level_pct: Option<f32>,
    /// Oil pressure sensor (kPa or psi).
    pub oil_pressure_kpa: Option<f32>,
    /// Lambda 1 (primary O2 sensor) voltage (0-5V).
    pub lambda1_voltage: Option<f32>,
    /// Lambda 2 (secondary O2 sensor) voltage (0-5V).
    pub lambda2_voltage: Option<f32>,
}

// ─────────────────────────────────────────────────────────────
// ADC raw → voltage
// ─────────────────────────────────────────────────────────────

/// Convert a 12-bit ADC reading to voltage.
///
/// microRusEFI uses a 3.3 V reference and 12-bit ADC.
#[inline]
pub fn adc_to_volts(raw: u16) -> f32 {
    (raw as f32) * (3.3 / 4096.0)
}

// ─────────────────────────────────────────────────────────────
// Steinhart-Hart thermistor
// ─────────────────────────────────────────────────────────────

/// Three-point calibration data for a thermistor.
///
/// `(resistance_ohm, temperature_celsius)` pairs at three different points.
#[derive(Clone, Copy, Debug)]
pub struct ThermistorCalibration {
    /// First calibration point `(resistance_ohm, temperature_celsius)`.
    pub p1: (f32, f32),
    /// Second calibration point `(resistance_ohm, temperature_celsius)`.
    pub p2: (f32, f32),
    /// Third calibration point `(resistance_ohm, temperature_celsius)`.
    pub p3: (f32, f32),
}

/// Steinhart-Hart coefficients computed from a three-point calibration.
#[derive(Clone, Copy, Debug)]
pub struct SteinhartHart {
    a: f32,
    b: f32,
    c: f32,
}

impl SteinhartHart {
    /// Compute Steinhart-Hart A, B, C coefficients from three calibration points.
    ///
    /// Reference: `firmware/controllers/sensors/converters/thermistor_func.cpp`
    pub fn from_calibration(cal: &ThermistorCalibration) -> Self {
        let (r1, t1_c) = cal.p1;
        let (r2, t2_c) = cal.p2;
        let (r3, t3_c) = cal.p3;

        let y1 = 1.0 / (t1_c + 273.15);
        let y2 = 1.0 / (t2_c + 273.15);
        let y3 = 1.0 / (t3_c + 273.15);

        let l1 = logf(r1);
        let l2 = logf(r2);
        let l3 = logf(r3);

        let u2 = (y2 - y1) / (l2 - l1);
        let u3 = (y3 - y1) / (l3 - l1);

        let c = (u3 - u2) / (l3 - l2) / (l1 + l2 + l3);
        let b = u2 - c * (l1 * l1 + l1 * l2 + l2 * l2);
        let a = y1 - (b + l1 * l1 * c) * l1;

        Self { a, b, c }
    }

    /// Convert a resistance (Ω) to temperature (°C).
    ///
    /// Returns `None` if the resistance is non-positive or conversion overflows.
    pub fn resistance_to_celsius(&self, resistance_ohm: f32) -> Option<f32> {
        if resistance_ohm <= 0.0 {
            return None;
        }
        let ln_r = logf(resistance_ohm);
        let inv_t = self.a + self.b * ln_r + self.c * powf(ln_r, 3.0);
        if inv_t <= 0.0 {
            return None;
        }
        Some((1.0 / inv_t) - 273.15)
    }
}

/// Convert a voltage across a resistor divider to thermistor resistance.
///
/// `bias_ohm` is the fixed pull-up (or pull-down) resistor value.
/// `supply_v` is the reference voltage (e.g. 5.0 V for sensor supply).
pub fn voltage_to_resistance(v_sensor: f32, supply_v: f32, bias_ohm: f32) -> Option<f32> {
    if v_sensor <= 0.0 || v_sensor >= supply_v {
        return None;
    }
    // Pull-up divider: R_thermistor = bias * V / (Vcc - V)
    Some(bias_ohm * v_sensor / (supply_v - v_sensor))
}

// ─────────────────────────────────────────────────────────────
// Linear conversion (TPS, MAP, etc.)
// ─────────────────────────────────────────────────────────────

/// Linear sensor conversion: voltage → physical quantity.
///
/// Clamps output to `[out_min, out_max]`.
#[derive(Clone, Copy, Debug)]
pub struct LinearSensor {
    a: f32,
    b: f32,
    /// Minimum valid output value; readings below this return `None`.
    pub out_min: f32,
    /// Maximum valid output value; readings above this return `None`.
    pub out_max: f32,
}

impl LinearSensor {
    /// Create from two calibration points `(v1, out1)` and `(v2, out2)`.
    pub fn from_two_points(v1: f32, out1: f32, v2: f32, out2: f32, out_min: f32, out_max: f32) -> Self {
        let a = (out2 - out1) / (v2 - v1);
        let b = out1 - a * v1;
        Self { a, b, out_min, out_max }
    }

    /// Convert a voltage to the physical quantity.
    ///
    /// Returns `None` if the result is outside `[out_min, out_max]`.
    pub fn convert(&self, voltage: f32) -> Option<f32> {
        let result = self.a * voltage + self.b;
        if result < self.out_min || result > self.out_max {
            None
        } else {
            Some(result)
        }
    }
}

// ─────────────────────────────────────────────────────────────
// IIR low-pass filter
// ─────────────────────────────────────────────────────────────

/// Exponential moving average (IIR first-order low-pass filter).
///
/// `alpha` in `(0, 1]`: higher = faster response, lower = more smoothing.
#[derive(Clone, Copy, Debug)]
pub struct IirFilter {
    alpha: f32,
    value: f32,
    initialized: bool,
}

impl IirFilter {
    /// Create a new filter with the given smoothing coefficient.
    pub const fn new(alpha: f32) -> Self {
        Self { alpha, value: 0.0, initialized: false }
    }

    /// Feed a new sample and return the filtered output.
    pub fn update(&mut self, sample: f32) -> f32 {
        if self.initialized {
            self.value = self.alpha * sample + (1.0 - self.alpha) * self.value;
        } else {
            self.value = sample;
            self.initialized = true;
        }
        self.value
    }

    /// Current filtered value.
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Reset to uninitialized state.
    pub fn reset(&mut self) {
        self.initialized = false;
    }
}

// ─────────────────────────────────────────────────────────────
// MAF sensor conversion
// ─────────────────────────────────────────────────────────────

/// MAF sensor type and calibration.
#[derive(Clone, Copy, Debug)]
pub enum MafSensorType {
    /// Voltage-based MAF (0-5V output proportional to airflow).
    Voltage,
    /// Frequency-based MAF (Hz output proportional to airflow).
    Frequency,
}

/// MAF sensor configuration.
#[derive(Clone, Copy, Debug)]
pub struct MafSensorConfig {
    /// Sensor type.
    pub sensor_type: MafSensorType,
    /// Minimum voltage (g/s = 0).
    pub min_voltage: f32,
    /// Maximum voltage (max g/s).
    pub max_voltage: f32,
    /// Maximum airflow rate in g/s.
    pub max_flow_g_per_s: f32,
}

impl Default for MafSensorConfig {
    fn default() -> Self {
        Self {
            sensor_type: MafSensorType::Voltage,
            min_voltage: 0.5,
            max_voltage: 4.5,
            max_flow_g_per_s: 250.0,
        }
    }
}

/// MAF sensor converter.
pub struct MafSensor {
    cfg: MafSensorConfig,
}

impl MafSensor {
    /// Create new MAF sensor with configuration.
    pub fn new(cfg: MafSensorConfig) -> Self {
        Self { cfg }
    }

    /// Convert voltage to airflow rate in g/s.
    ///
    /// Returns `None` if voltage is outside valid range.
    pub fn voltage_to_flow(&self, voltage: f32) -> Option<f32> {
        if voltage < self.cfg.min_voltage || voltage > self.cfg.max_voltage {
            return None;
        }
        let normalized = (voltage - self.cfg.min_voltage) / (self.cfg.max_voltage - self.cfg.min_voltage);
        Some(normalized * self.cfg.max_flow_g_per_s)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &MafSensorConfig {
        &self.cfg
    }
}

// ─────────────────────────────────────────────────────────────
// Fuel level sensor
// ─────────────────────────────────────────────────────────────

/// Fuel level sensor configuration.
#[derive(Clone, Copy, Debug)]
pub struct FuelLevelConfig {
    /// Voltage at empty tank.
    pub empty_voltage: f32,
    /// Voltage at full tank.
    pub full_voltage: f32,
    /// Minimum valid percentage.
    pub min_pct: f32,
    /// Maximum valid percentage.
    pub max_pct: f32,
}

impl Default for FuelLevelConfig {
    fn default() -> Self {
        Self {
            empty_voltage: 0.5,
            full_voltage: 4.5,
            min_pct: 0.0,
            max_pct: 100.0,
        }
    }
}

/// Fuel level sensor converter.
pub struct FuelLevelSensor {
    cfg: FuelLevelConfig,
}

impl FuelLevelSensor {
    /// Create new fuel level sensor with configuration.
    pub fn new(cfg: FuelLevelConfig) -> Self {
        Self { cfg }
    }

    /// Convert voltage to fuel level percentage.
    ///
    /// Returns `None` if voltage is outside valid range.
    pub fn voltage_to_pct(&self, voltage: f32) -> Option<f32> {
        if voltage < self.cfg.empty_voltage || voltage > self.cfg.full_voltage {
            return None;
        }
        let normalized = (voltage - self.cfg.empty_voltage) / (self.cfg.full_voltage - self.cfg.empty_voltage);
        let pct = normalized * 100.0;
        Some(pct.clamp(self.cfg.min_pct, self.cfg.max_pct))
    }

    /// Get configuration reference.
    pub fn config(&self) -> &FuelLevelConfig {
        &self.cfg
    }
}

// ─────────────────────────────────────────────────────────────
// Oil pressure sensor
// ─────────────────────────────────────────────────────────────

/// Oil pressure sensor configuration.
#[derive(Clone, Copy, Debug)]
pub struct OilPressureConfig {
    /// Voltage at 0 kPa.
    pub zero_voltage: f32,
    /// Voltage at maximum pressure.
    pub max_voltage: f32,
    /// Maximum pressure in kPa.
    pub max_pressure_kpa: f32,
}

impl Default for OilPressureConfig {
    fn default() -> Self {
        Self {
            zero_voltage: 0.5,
            max_voltage: 4.5,
            max_pressure_kpa: 700.0, // ~100 psi
        }
    }
}

/// Oil pressure sensor converter.
pub struct OilPressureSensor {
    cfg: OilPressureConfig,
}

impl OilPressureSensor {
    /// Create new oil pressure sensor with configuration.
    pub fn new(cfg: OilPressureConfig) -> Self {
        Self { cfg }
    }

    /// Convert voltage to oil pressure in kPa.
    ///
    /// Returns `None` if voltage is outside valid range.
    pub fn voltage_to_kpa(&self, voltage: f32) -> Option<f32> {
        if voltage < self.cfg.zero_voltage || voltage > self.cfg.max_voltage {
            return None;
        }
        let normalized = (voltage - self.cfg.zero_voltage) / (self.cfg.max_voltage - self.cfg.zero_voltage);
        Some(normalized * self.cfg.max_pressure_kpa)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &OilPressureConfig {
        &self.cfg
    }
}

// ─────────────────────────────────────────────────────────────
// Lambda (O2) sensor
// ─────────────────────────────────────────────────────────────

/// Lambda sensor type.
#[derive(Clone, Copy, Debug)]
pub enum LambdaSensorType {
    /// Narrowband (0-1V, switches at stoichiometric).
    Narrowband,
    /// Wideband (0-5V linear, 0V = lean, 5V = rich).
    Wideband,
}

/// Lambda sensor configuration.
#[derive(Clone, Copy, Debug)]
pub struct LambdaSensorConfig {
    /// Sensor type.
    pub sensor_type: LambdaSensorType,
    /// Voltage at lambda = 2.0 (lean).
    pub lean_voltage: f32,
    /// Voltage at lambda = 0.5 (rich).
    pub rich_voltage: f32,
}

impl Default for LambdaSensorConfig {
    fn default() -> Self {
        Self {
            sensor_type: LambdaSensorType::Wideband,
            lean_voltage: 0.0,
            rich_voltage: 5.0,
        }
    }
}

/// Lambda sensor converter.
pub struct LambdaSensor {
    cfg: LambdaSensorConfig,
}

impl LambdaSensor {
    /// Create new lambda sensor with configuration.
    pub fn new(cfg: LambdaSensorConfig) -> Self {
        Self { cfg }
    }

    /// Convert voltage to lambda value.
    ///
    /// For wideband: 0V = 2.0 (lean), 5V = 0.5 (rich), 2.5V = 1.0 (stoichiometric).
    /// For narrowband: returns approximate lambda based on switching.
    ///
    /// Returns `None` if voltage is outside valid range.
    pub fn voltage_to_lambda(&self, voltage: f32) -> Option<f32> {
        match self.cfg.sensor_type {
            LambdaSensorType::Wideband => {
                if voltage < 0.0 || voltage > 5.0 {
                    return None;
                }
                // Piecewise-linear mapping through the stoichiometric midpoint:
                //   lean_voltage  -> 2.0 (lean)
                //   stoich (mid)  -> 1.0 (stoichiometric)
                //   rich_voltage  -> 0.5 (rich)
                // The lean and rich sides have different slopes, so a single
                // straight line would mis-report mid-range mixtures.
                let lean_v = self.cfg.lean_voltage;
                let rich_v = self.cfg.rich_voltage;
                let stoich_v = (lean_v + rich_v) * 0.5;
                let lambda = if voltage <= stoich_v {
                    let span = stoich_v - lean_v;
                    if span.abs() < f32::EPSILON {
                        1.0
                    } else {
                        2.0 + (voltage - lean_v) / span * (1.0 - 2.0)
                    }
                } else {
                    let span = rich_v - stoich_v;
                    if span.abs() < f32::EPSILON {
                        1.0
                    } else {
                        1.0 + (voltage - stoich_v) / span * (0.5 - 1.0)
                    }
                };
                Some(lambda.clamp(0.5, 2.0))
            }
            LambdaSensorType::Narrowband => {
                // Narrowband is non-linear, approximate:
                // < 0.45V = rich, > 0.55V = lean, around 0.5V = stoichiometric
                if voltage < 0.0 || voltage > 1.0 {
                    return None;
                }
                if voltage < 0.45 {
                    Some(0.9) // Rich
                } else if voltage > 0.55 {
                    Some(1.1) // Lean
                } else {
                    Some(1.0) // Stoichiometric
                }
            }
        }
    }

    /// Convert voltage to AFR (Air-Fuel Ratio).
    ///
    /// # Arguments
    /// * `voltage` - Sensor voltage
    /// * `stoich_afr` - Stoichiometric AFR (14.7 for gasoline)
    ///
    /// # Returns
    /// AFR value, or `None` if conversion fails.
    pub fn voltage_to_afr(&self, voltage: f32, stoich_afr: f32) -> Option<f32> {
        self.voltage_to_lambda(voltage).map(|lambda| lambda * stoich_afr)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &LambdaSensorConfig {
        &self.cfg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn adc_to_volts_midscale() {
        // 2048 → ~1.65 V
        let v = adc_to_volts(2048);
        assert_relative_eq!(v, 1.65, epsilon = 0.01);
    }

    #[test]
    fn steinhart_hart_known_values() {
        // Rough NTC: -40°C = 100 kΩ, 20°C = 2.5 kΩ, 100°C = 177 Ω
        let cal = ThermistorCalibration {
            p1: (100_000.0, -40.0),
            p2: (2_500.0, 20.0),
            p3: (177.0, 100.0),
        };
        let sh = SteinhartHart::from_calibration(&cal);
        // Should reproduce calibration points within ±0.5 °C
        let t = sh.resistance_to_celsius(2_500.0).expect("valid");
        assert_relative_eq!(t, 20.0, epsilon = 0.5);
    }

    #[test]
    fn linear_tps_conversion() {
        // 0.5 V → 0 %, 4.5 V → 100 %
        let tps = LinearSensor::from_two_points(0.5, 0.0, 4.5, 100.0, 0.0, 100.0);
        assert_relative_eq!(tps.convert(0.5).unwrap(), 0.0);
        assert_relative_eq!(tps.convert(4.5).unwrap(), 100.0);
        assert_relative_eq!(tps.convert(2.5).unwrap(), 50.0);
        // Out of range
        assert!(tps.convert(0.0).is_none());
    }

    #[test]
    fn iir_filter_convergence() {
        let mut f = IirFilter::new(0.5);
        // First sample: initialises to sample value
        assert_relative_eq!(f.update(10.0), 10.0);
        // After many samples of 0: should converge to 0
        for _ in 0..20 {
            f.update(0.0);
        }
        assert!(f.value() < 0.01);
    }

    #[test]
    fn sensor_result_valid_value() {
        let r: SensorResult<f32> = SensorResult::Ok(25.0);
        assert!(r.is_valid());
        assert_eq!(r.value(), Some(&25.0));
        assert_eq!(r.ok(), Some(25.0));
        assert_relative_eq!(r.unwrap_or(0.0), 25.0);
    }

    #[test]
    fn sensor_result_too_low() {
        let r: SensorResult<f32> = SensorResult::TooLow;
        assert!(!r.is_valid());
        assert_eq!(r.value(), None);
        assert_eq!(r.ok(), None);
        assert_relative_eq!(r.unwrap_or(10.0), 10.0); // returns default
    }

    #[test]
    fn sensor_result_too_high() {
        let r: SensorResult<f32> = SensorResult::TooHigh;
        assert!(!r.is_valid());
        assert_eq!(r.ok(), None);
    }

    #[test]
    fn sensor_result_map() {
        let r: SensorResult<f32> = SensorResult::Ok(10.0);
        let mapped = r.map(|v| v * 2.0);
        assert_eq!(mapped.ok(), Some(20.0));

        let r2: SensorResult<f32> = SensorResult::TooLow;
        let mapped2 = r2.map(|v| v * 2.0);
        assert_eq!(mapped2, SensorResult::TooLow);
    }

    // MAF sensor tests
    #[test]
    fn maf_voltage_to_flow() {
        let cfg = MafSensorConfig::default();
        let maf = MafSensor::new(cfg);

        // Mid voltage = half flow
        let flow = maf.voltage_to_flow(2.5);
        assert!(flow.is_some());
        assert_relative_eq!(flow.unwrap(), 125.0, epsilon = 1.0); // ~125 g/s
    }

    #[test]
    fn maf_out_of_range() {
        let cfg = MafSensorConfig::default();
        let maf = MafSensor::new(cfg);

        assert!(maf.voltage_to_flow(0.2).is_none()); // Below min
        assert!(maf.voltage_to_flow(5.0).is_none()); // Above max
    }

    // Fuel level sensor tests
    #[test]
    fn fuel_level_voltage_to_pct() {
        let cfg = FuelLevelConfig::default();
        let fuel = FuelLevelSensor::new(cfg);

        // Empty
        assert_relative_eq!(fuel.voltage_to_pct(0.5).unwrap(), 0.0, epsilon = 0.1);
        // Half
        assert_relative_eq!(fuel.voltage_to_pct(2.5).unwrap(), 50.0, epsilon = 0.1);
        // Full
        assert_relative_eq!(fuel.voltage_to_pct(4.5).unwrap(), 100.0, epsilon = 0.1);
    }

    #[test]
    fn fuel_level_out_of_range() {
        let cfg = FuelLevelConfig::default();
        let fuel = FuelLevelSensor::new(cfg);

        assert!(fuel.voltage_to_pct(0.2).is_none());
        assert!(fuel.voltage_to_pct(5.0).is_none());
    }

    // Oil pressure sensor tests
    #[test]
    fn oil_pressure_voltage_to_kpa() {
        let cfg = OilPressureConfig::default();
        let oil = OilPressureSensor::new(cfg);

        // Zero pressure
        assert_relative_eq!(oil.voltage_to_kpa(0.5).unwrap(), 0.0, epsilon = 1.0);
        // Half pressure
        assert_relative_eq!(oil.voltage_to_kpa(2.5).unwrap(), 350.0, epsilon = 1.0);
        // Max pressure
        assert_relative_eq!(oil.voltage_to_kpa(4.5).unwrap(), 700.0, epsilon = 1.0);
    }

    #[test]
    fn oil_pressure_out_of_range() {
        let cfg = OilPressureConfig::default();
        let oil = OilPressureSensor::new(cfg);

        assert!(oil.voltage_to_kpa(0.2).is_none());
        assert!(oil.voltage_to_kpa(5.0).is_none());
    }

    // Lambda sensor tests
    #[test]
    fn lambda_wideband_voltage_to_lambda() {
        let cfg = LambdaSensorConfig {
            sensor_type: LambdaSensorType::Wideband,
            ..Default::default()
        };
        let lambda = LambdaSensor::new(cfg);

        // 0V = 2.0 (lean)
        assert_relative_eq!(lambda.voltage_to_lambda(0.0).unwrap(), 2.0, epsilon = 0.01);
        // 2.5V = 1.0 (stoichiometric)
        assert_relative_eq!(lambda.voltage_to_lambda(2.5).unwrap(), 1.0, epsilon = 0.01);
        // 5V = 0.5 (rich)
        assert_relative_eq!(lambda.voltage_to_lambda(5.0).unwrap(), 0.5, epsilon = 0.01);
    }

    #[test]
    fn lambda_wideband_voltage_to_afr() {
        let cfg = LambdaSensorConfig {
            sensor_type: LambdaSensorType::Wideband,
            ..Default::default()
        };
        let lambda = LambdaSensor::new(cfg);

        // 2.5V = 1.0 lambda = 14.7 AFR
        let afr = lambda.voltage_to_afr(2.5, 14.7);
        assert!(afr.is_some());
        assert_relative_eq!(afr.unwrap(), 14.7, epsilon = 0.1);
    }

    #[test]
    fn lambda_narrowband_approximate() {
        let cfg = LambdaSensorConfig {
            sensor_type: LambdaSensorType::Narrowband,
            ..Default::default()
        };
        let lambda = LambdaSensor::new(cfg);

        // Rich
        assert_relative_eq!(lambda.voltage_to_lambda(0.3).unwrap(), 0.9, epsilon = 0.01);
        // Stoichiometric
        assert_relative_eq!(lambda.voltage_to_lambda(0.5).unwrap(), 1.0, epsilon = 0.01);
        // Lean
        assert_relative_eq!(lambda.voltage_to_lambda(0.7).unwrap(), 1.1, epsilon = 0.01);
    }

    #[test]
    fn lambda_out_of_range() {
        let cfg = LambdaSensorConfig::default();
        let lambda = LambdaSensor::new(cfg);

        assert!(lambda.voltage_to_lambda(-0.1).is_none());
        assert!(lambda.voltage_to_lambda(6.0).is_none());
    }
}
