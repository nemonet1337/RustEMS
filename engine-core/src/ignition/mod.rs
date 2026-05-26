//! Ignition timing and dwell calculation.
//!
//! Derived from `firmware/controllers/algo/ignition/ignition_state.cpp`.

use crate::config::EngineConfig;
use crate::maps::interpolation::{interpolate1d, interpolate2d};
use crate::sensors::SensorData;

#[cfg(feature = "cyl-1")]
/// Number of cylinders — 1 (set by `cyl-1` feature).
pub const CYL_COUNT: u8 = 1;
#[cfg(feature = "cyl-2")]
/// Number of cylinders — 2 (set by `cyl-2` feature).
pub const CYL_COUNT: u8 = 2;
#[cfg(feature = "cyl-3")]
/// Number of cylinders — 3 (set by `cyl-3` feature).
pub const CYL_COUNT: u8 = 3;
#[cfg(feature = "cyl-4")]
/// Number of cylinders — 4 (set by `cyl-4` feature).
pub const CYL_COUNT: u8 = 4;
#[cfg(feature = "cyl-5")]
/// Number of cylinders — 5 (set by `cyl-5` feature).
pub const CYL_COUNT: u8 = 5;
#[cfg(feature = "cyl-6")]
/// Number of cylinders — 6 (set by `cyl-6` feature).
pub const CYL_COUNT: u8 = 6;
#[cfg(feature = "cyl-8")]
/// Number of cylinders — 8 (set by `cyl-8` feature).
pub const CYL_COUNT: u8 = 8;
#[cfg(feature = "cyl-10")]
/// Number of cylinders — 10 (set by `cyl-10` feature).
pub const CYL_COUNT: u8 = 10;
#[cfg(feature = "cyl-12")]
/// Number of cylinders — 12 (set by `cyl-12` feature).
pub const CYL_COUNT: u8 = 12;

// Compile-time assertion: exactly one cyl-N feature must be selected.
#[cfg(not(any(
    feature = "cyl-1",
    feature = "cyl-2",
    feature = "cyl-3",
    feature = "cyl-4",
    feature = "cyl-5",
    feature = "cyl-6",
    feature = "cyl-8",
    feature = "cyl-10",
    feature = "cyl-12"
)))]
compile_error!("Exactly one of cyl-1, cyl-2, cyl-3, cyl-4, cyl-5, cyl-6, cyl-8, cyl-10, cyl-12 features must be enabled");

#[cfg(all(feature = "cyl-1", feature = "cyl-2"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");
#[cfg(all(feature = "cyl-1", feature = "cyl-3"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");
#[cfg(all(feature = "cyl-1", feature = "cyl-4"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");
#[cfg(all(feature = "cyl-2", feature = "cyl-3"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");
#[cfg(all(feature = "cyl-2", feature = "cyl-4"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");
#[cfg(all(feature = "cyl-3", feature = "cyl-4"))]
compile_error!("Only one cylinder-count feature may be enabled at a time");

/// Maximum dwell time to prevent coil overheating (ms).
const MAX_DWELL_MS: f32 = 10.0;

/// Overdwell protection configuration.
#[derive(Clone, Copy, Debug)]
pub struct OverdwellConfig {
    /// Maximum allowed dwell time in milliseconds.
    pub max_dwell_ms: f32,
    /// Warning threshold at 80% of max dwell (for diagnostics).
    pub warning_threshold_pct: f32,
}

impl Default for OverdwellConfig {
    fn default() -> Self {
        Self {
            max_dwell_ms: MAX_DWELL_MS,
            warning_threshold_pct: 0.8,
        }
    }
}

/// Overdwell protection state machine per coil.
///
/// Tracks coil charging state and enforces maximum dwell time.
/// If the coil charges for too long, it will be force-fired to prevent overheating.
#[derive(Clone, Copy, Debug, Default)]
pub struct OverdwellController {
    cfg: OverdwellConfig,
    /// Timestamp when coil charging started (µs), None if not charging.
    charge_start_us: Option<u64>,
    /// True if coil is currently charging.
    charging: bool,
    /// Count of overdwell events (for diagnostics).
    overdwell_count: u32,
}

impl OverdwellController {
    /// Create a new overdwell controller with the given configuration.
    pub fn new(cfg: OverdwellConfig) -> Self {
        Self {
            cfg,
            charge_start_us: None,
            charging: false,
            overdwell_count: 0,
        }
    }

    /// Start coil charging.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds.
    pub fn start_charge(&mut self, now_us: u64) {
        self.charge_start_us = Some(now_us);
        self.charging = true;
    }

    /// Stop coil charging (normal spark fired).
    pub fn end_charge(&mut self) {
        self.charge_start_us = None;
        self.charging = false;
    }

    /// Check if coil should be force-fired due to overdwell.
    ///
    /// Call this periodically (e.g., from 1ms tick) to check for overdwell condition.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds.
    ///
    /// # Returns
    /// `true` if coil should be force-fired to prevent overheating.
    pub fn check_overdwell(&mut self, now_us: u64) -> bool {
        if !self.charging {
            return false;
        }

        let Some(start_us) = self.charge_start_us else {
            return false;
        };

        let elapsed_ms = (now_us - start_us) as f32 / 1000.0;
        if elapsed_ms >= self.cfg.max_dwell_ms {
            self.overdwell_count += 1;
            self.end_charge();
            true
        } else {
            false
        }
    }

    /// Get current dwell time in milliseconds (0 if not charging).
    pub fn current_dwell_ms(&self, now_us: u64) -> f32 {
        match self.charge_start_us {
            Some(start_us) if self.charging => {
                ((now_us - start_us) as f32 / 1000.0).min(self.cfg.max_dwell_ms)
            }
            _ => 0.0,
        }
    }

    /// Check if near overdwell threshold (for diagnostics/warning).
    pub fn is_near_overdwell(&self, now_us: u64) -> bool {
        if !self.charging {
            return false;
        }
        let Some(start_us) = self.charge_start_us else {
            return false;
        };
        let elapsed_ms = (now_us - start_us) as f32 / 1000.0;
        let warning_ms = self.cfg.max_dwell_ms * self.cfg.warning_threshold_pct;
        elapsed_ms >= warning_ms
    }

    /// True if coil is currently charging.
    pub fn is_charging(&self) -> bool {
        self.charging
    }

    /// Total overdwell events count (diagnostics).
    pub fn overdwell_count(&self) -> u32 {
        self.overdwell_count
    }

    /// Reset controller (e.g., on engine stall).
    pub fn reset(&mut self) {
        self.charge_start_us = None;
        self.charging = false;
        self.overdwell_count = 0;
    }
}

/// Computed ignition state for one engine cycle.
#[derive(Clone, Copy, Debug)]
pub struct IgnitionOutput {
    /// Ignition advance angle in degrees BTDC (Before TDC).
    pub advance_deg: f32,
    /// Coil dwell duration in milliseconds.
    pub dwell_ms: f32,
    /// Coil dwell expressed as crank angle degrees.
    pub dwell_deg: f32,
    /// Angle at which the coil begins charging (= TDC - advance - dwell_deg).
    pub charge_start_deg: f32,
    /// Angle at which the spark fires (= TDC - advance).
    pub spark_deg: f32,
    /// True if ignition is cut due to RPM limiter.
    pub rpm_limiter_active: bool,
}

/// RPM limiter configuration.
#[derive(Clone, Copy, Debug)]
pub struct RpmLimiterConfig {
    /// Hard RPM cutoff limit.
    pub hard_limit_rpm: f32,
    /// Recovery RPM (hysteresis: must drop below this to resume).
    pub recovery_rpm: f32,
    /// Whether soft spark cut is enabled (random misfire above limit).
    pub soft_cut_enabled: bool,
}

impl Default for RpmLimiterConfig {
    fn default() -> Self {
        Self {
            hard_limit_rpm: 7500.0,
            recovery_rpm: 7200.0,
            soft_cut_enabled: false,
        }
    }
}

impl RpmLimiterConfig {
    /// Create default limiter for a typical 4-cylinder engine.
    pub fn default_4cyl() -> Self {
        Self::default()
    }

    /// Create default limiter for a motorcycle engine (higher RPM).
    pub fn default_bike() -> Self {
        Self {
            hard_limit_rpm: 12000.0,
            recovery_rpm: 11500.0,
            soft_cut_enabled: false,
        }
    }
}

/// RPM limiter state machine.
#[derive(Clone, Copy, Debug, Default)]
pub struct RpmLimiter {
    cfg: RpmLimiterConfig,
    active: bool,
}

impl RpmLimiter {
    /// Create a new RPM limiter with the given configuration.
    pub fn new(cfg: RpmLimiterConfig) -> Self {
        Self { cfg, active: false }
    }

    /// Update limiter state and return whether ignition should be cut.
    pub fn update(&mut self, rpm: f32) -> bool {
        if self.active {
            // Already active: stay cut until below recovery RPM
            if rpm < self.cfg.recovery_rpm {
                self.active = false;
            }
        } else {
            // Not active: cut if above hard limit
            if rpm >= self.cfg.hard_limit_rpm {
                self.active = true;
            }
        }
        self.active
    }

    /// Current limiter state.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Reset limiter (e.g., on engine stall).
    pub fn reset(&mut self) {
        self.active = false;
    }
}

/// Compute ignition timing for one cylinder given engine operating conditions.
///
/// # Arguments
/// * `cfg`     — engine configuration (maps, dwell tables)
/// * `sensors` — current sensor readings
/// * `tdc_deg` — TDC angle for this cylinder within the 720° engine cycle
///
/// # Returns
/// `Some(IgnitionOutput)` if RPM is valid (> 0), `None` otherwise.
pub fn compute_ignition(
    cfg: &EngineConfig,
    sensors: &SensorData,
    tdc_deg: f32,
) -> Option<IgnitionOutput> {
    let rpm = sensors.rpm?;
    if rpm <= 0.0 {
        return None;
    }

    // ── advance angle ───────────────────────────────────────────────────────
    let load = sensors.load_pct.unwrap_or(0.0);

    let advance_deg = if rpm < cfg.cranking_rpm {
        // Cranking advance — single value from config
        cfg.cranking_timing_deg
    } else {
        // Running advance — look up in 2D table (load × RPM)
        interpolate2d(
            &cfg.ignition_table,
            &cfg.ignition_load_bins,
            load,
            &cfg.ignition_rpm_bins,
            rpm,
        )
    };

    // ── dwell ───────────────────────────────────────────────────────────────
    let dwell_ms = if rpm < cfg.cranking_rpm {
        cfg.cranking_dwell_ms
    } else {
        let base = interpolate1d(&cfg.dwell_rpm_bins, &cfg.dwell_ms_table, rpm);
        // Voltage correction
        let vcorr = if let Some(vbatt) = sensors.battery_volts {
            let c = interpolate1d(&cfg.dwell_voltage_bins, &cfg.dwell_voltage_corr, vbatt);
            if c < 0.1 { 1.0 } else { c }
        } else {
            1.0
        };
        base * vcorr
    };

    // ── overdwell protection ────────────────────────────────────────────────
    // Clamp dwell to prevent coil overheating
    let dwell_ms = dwell_ms.min(MAX_DWELL_MS);

    // ── angle conversion ────────────────────────────────────────────────────
    // ms per degree = 1000 / (rpm / 60 * 360) = 166.667 / rpm
    let dwell_deg = dwell_ms * rpm / 166.667;

    let spark_deg = tdc_deg - advance_deg;
    let charge_start_deg = spark_deg - dwell_deg;

    Some(IgnitionOutput {
        advance_deg,
        dwell_ms,
        dwell_deg,
        charge_start_deg,
        spark_deg,
        rpm_limiter_active: false,
    })
}

/// Returns the TDC angles (in degrees within the 720° engine cycle) for each
/// cylinder, given the firing order.
///
/// `firing_order[i]` is the 0-based cylinder index that fires at step `i`.
/// The first step fires at 0°, subsequent steps at equal intervals.
pub fn tdc_angles_from_firing_order(firing_order: &[u8]) -> heapless::Vec<f32, 4> {
    let n = firing_order.len();
    let interval = 720.0 / n as f32;
    let mut angles: heapless::Vec<f32, 4> = heapless::Vec::new();
    for step in 0..n {
        let _ = angles.push(step as f32 * interval);
    }
    angles
}

/// Engine load calculation strategy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadStrategy {
    /// Alpha-N: Use TPS as load (throttle-based).
    AlphaN,
    /// Speed Density: Use MAP + VE table (intake pressure-based).
    SpeedDensity,
    /// Simple: Use TPS percentage directly.
    Simple,
}

/// Configuration for Alpha-N load calculation.
#[derive(Clone, Copy, Debug)]
pub struct AlphaNConfig {
    /// RPM breakpoints for load table.
    pub rpm_bins: [f32; 8],
    /// TPS percentage breakpoints for load table.
    pub tps_bins: [f32; 8],
    /// Load table: load_pct values for each (RPM, TPS) point.
    pub load_table: [[f32; 8]; 8],
}

impl Default for AlphaNConfig {
    fn default() -> Self {
        Self {
            rpm_bins: [0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0],
            tps_bins: [0.0, 10.0, 25.0, 50.0, 75.0, 90.0, 100.0, 100.0],
            // Default: linear TPS to load mapping
            load_table: [
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 0 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 1000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 2000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 3000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 4000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 5000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 6000 RPM
                [0.0, 5.0, 12.5, 25.0, 37.5, 45.0, 50.0, 50.0], // 7000 RPM
            ],
        }
    }
}

impl AlphaNConfig {
    /// Create default configuration for motorcycle engine.
    pub fn default_bike() -> Self {
        Self {
            rpm_bins: [0.0, 2000.0, 4000.0, 6000.0, 8000.0, 10000.0, 12000.0, 14000.0],
            tps_bins: [0.0, 5.0, 15.0, 35.0, 60.0, 80.0, 100.0, 100.0],
            load_table: [
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
                [0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 75.0],
            ],
        }
    }
}

/// Alpha-N load calculator (TPS-based).
pub struct AlphaNCalculator {
    cfg: AlphaNConfig,
}

impl AlphaNCalculator {
    /// Create new Alpha-N calculator with configuration.
    pub fn new(cfg: AlphaNConfig) -> Self {
        Self { cfg }
    }

    /// Calculate engine load from TPS and RPM.
    ///
    /// # Arguments
    /// * `tps_pct` - Throttle position (0-100%)
    /// * `rpm` - Engine RPM
    ///
    /// # Returns
    /// Load percentage (0-100%)
    pub fn calculate_load(&self, tps_pct: f32, rpm: f32) -> f32 {
        interpolate2d(
            &self.cfg.load_table,
            &self.cfg.tps_bins,
            tps_pct.clamp(0.0, 100.0),
            &self.cfg.rpm_bins,
            rpm.clamp(0.0, 20000.0),
        )
        .clamp(0.0, 100.0)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &AlphaNConfig {
        &self.cfg
    }

    /// Update load table value at specific indices.
    pub fn set_load_value(&mut self, rpm_idx: usize, tps_idx: usize, value: f32) {
        if rpm_idx < 8 && tps_idx < 8 {
            self.cfg.load_table[rpm_idx][tps_idx] = value.clamp(0.0, 100.0);
        }
    }
}

/// Configuration for Speed Density load calculation.
#[derive(Clone, Copy, Debug)]
pub struct SpeedDensityConfig {
    /// RPM breakpoints for VE table.
    pub rpm_bins: [f32; 8],
    /// MAP (kPa) breakpoints for VE table.
    pub map_bins: [f32; 8],
    /// Volumetric Efficiency table (8x8).
    pub ve_table: [[f32; 8]; 8],
    /// Engine displacement in liters.
    pub displacement_l: f32,
    /// Ideal gas constant factor (simplified).
    pub air_temp_k: f32,
}

impl Default for SpeedDensityConfig {
    fn default() -> Self {
        Self {
            rpm_bins: [0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0],
            map_bins: [20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0],
            // Default VE table: typical engine curve
            ve_table: [
                [50.0, 55.0, 60.0, 65.0, 70.0, 72.0, 70.0, 68.0], // 0 RPM
                [55.0, 60.0, 68.0, 75.0, 85.0, 90.0, 88.0, 85.0], // 1000 RPM
                [58.0, 65.0, 75.0, 85.0, 95.0, 100.0, 98.0, 95.0], // 2000 RPM
                [60.0, 68.0, 78.0, 90.0, 100.0, 105.0, 103.0, 100.0], // 3000 RPM
                [62.0, 70.0, 80.0, 92.0, 102.0, 108.0, 106.0, 103.0], // 4000 RPM
                [60.0, 68.0, 78.0, 90.0, 100.0, 105.0, 103.0, 100.0], // 5000 RPM
                [58.0, 65.0, 75.0, 85.0, 95.0, 100.0, 98.0, 95.0], // 6000 RPM
                [55.0, 60.0, 70.0, 80.0, 90.0, 95.0, 93.0, 90.0], // 7000 RPM
            ],
            displacement_l: 2.0,
            air_temp_k: 298.0, // 25°C
        }
    }
}

/// Speed Density load calculator (MAP-based with VE table).
pub struct SpeedDensityCalculator {
    cfg: SpeedDensityConfig,
}

impl SpeedDensityCalculator {
    /// Create new Speed Density calculator with configuration.
    pub fn new(cfg: SpeedDensityConfig) -> Self {
        Self { cfg }
    }

    /// Calculate engine load from MAP, RPM, and IAT.
    ///
    /// # Arguments
    /// * `map_kpa` - Manifold Absolute Pressure in kPa (0-160)
    /// * `rpm` - Engine RPM
    /// * `iat_c` - Intake Air Temperature in Celsius (optional, uses default if None)
    ///
    /// # Returns
    /// Load percentage (0-100%) representing relative air mass
    pub fn calculate_load(&self, map_kpa: f32, rpm: f32, iat_c: Option<f32>) -> f32 {
        // Clamp inputs
        let map = map_kpa.clamp(0.0, 200.0);
        let rpm = rpm.clamp(0.0, 20000.0);

        // Look up VE from table
        let ve_pct = interpolate2d(
            &self.cfg.ve_table,
            &self.cfg.map_bins,
            map,
            &self.cfg.rpm_bins,
            rpm,
        )
        .clamp(0.0, 150.0);

        // Temperature correction (ideal gas law: PV = nRT)
        // Load is proportional to air mass, which is inversely proportional to temperature
        let temp_k = iat_c.map(|t| t + 273.15).unwrap_or(self.cfg.air_temp_k);
        let temp_factor = self.cfg.air_temp_k / temp_k.max(200.0);

        // Calculate normalized load
        // Load = (MAP / 100kPa) * VE * temp_factor * scaling
        let map_factor = map / 100.0;
        let load = map_factor * ve_pct * temp_factor;

        load.clamp(0.0, 100.0)
    }

    /// Calculate air mass flow rate (g/s) for fuel calculation.
    ///
    /// # Arguments
    /// * `map_kpa` - Manifold Absolute Pressure in kPa
    /// * `rpm` - Engine RPM
    /// * `iat_c` - Intake Air Temperature in Celsius
    ///
    /// # Returns
    /// Estimated air mass in grams per cylinder cycle
    pub fn calculate_air_mass(&self, map_kpa: f32, rpm: f32, iat_c: f32) -> f32 {
        let ve_pct = interpolate2d(
            &self.cfg.ve_table,
            &self.cfg.map_bins,
            map_kpa.clamp(0.0, 200.0),
            &self.cfg.rpm_bins,
            rpm.clamp(0.0, 20000.0),
        );

        // Simplified air mass calculation
        // m_air = (MAP * VE * displacement) / (R * T)
        let temp_k = (iat_c + 273.15).max(200.0);
        let r_air = 0.287; // kJ/(kg·K)

        // Convert to grams (simplified)
        let air_mass_g = (map_kpa * ve_pct * self.cfg.displacement_l) / (r_air * temp_k * 100.0);

        air_mass_g.max(0.0)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &SpeedDensityConfig {
        &self.cfg
    }

    /// Update VE table value at specific indices.
    pub fn set_ve_value(&mut self, rpm_idx: usize, map_idx: usize, value: f32) {
        if rpm_idx < 8 && map_idx < 8 {
            self.cfg.ve_table[rpm_idx][map_idx] = value.clamp(0.0, 150.0);
        }
    }
}

/// Unified load calculator that supports both Alpha-N and Speed Density.
pub struct LoadCalculator {
    strategy: LoadStrategy,
    alpha_n: Option<AlphaNCalculator>,
    speed_density: Option<SpeedDensityCalculator>,
}

impl LoadCalculator {
    /// Create calculator with Alpha-N strategy.
    pub fn new_alpha_n(cfg: AlphaNConfig) -> Self {
        Self {
            strategy: LoadStrategy::AlphaN,
            alpha_n: Some(AlphaNCalculator::new(cfg)),
            speed_density: None,
        }
    }

    /// Create calculator with Speed Density strategy.
    pub fn new_speed_density(cfg: SpeedDensityConfig) -> Self {
        Self {
            strategy: LoadStrategy::SpeedDensity,
            alpha_n: None,
            speed_density: Some(SpeedDensityCalculator::new(cfg)),
        }
    }

    /// Create simple calculator using direct TPS percentage.
    pub fn new_simple() -> Self {
        Self {
            strategy: LoadStrategy::Simple,
            alpha_n: None,
            speed_density: None,
        }
    }

    /// Calculate load based on selected strategy.
    ///
    /// # Arguments
    /// * `tps_pct` - TPS percentage (required for Alpha-N and Simple)
    /// * `map_kpa` - MAP in kPa (required for Speed Density)
    /// * `rpm` - Engine RPM (required for Alpha-N and Speed Density)
    /// * `iat_c` - Intake Air Temperature (optional, used by Speed Density)
    ///
    /// # Returns
    /// Load percentage (0-100%)
    pub fn calculate(
        &self,
        tps_pct: Option<f32>,
        map_kpa: Option<f32>,
        rpm: Option<f32>,
        iat_c: Option<f32>,
    ) -> f32 {
        match self.strategy {
            LoadStrategy::AlphaN => {
                if let (Some(tps), Some(r)) = (tps_pct, rpm) {
                    self.alpha_n
                        .as_ref()
                        .map(|calc| calc.calculate_load(tps, r))
                        .unwrap_or(tps)
                } else {
                    tps_pct.unwrap_or(0.0)
                }
            }
            LoadStrategy::SpeedDensity => {
                if let (Some(map), Some(r)) = (map_kpa, rpm) {
                    self.speed_density
                        .as_ref()
                        .map(|calc| calc.calculate_load(map, r, iat_c))
                        .unwrap_or(map)
                } else {
                    map_kpa.unwrap_or(0.0)
                }
            }
            LoadStrategy::Simple => tps_pct.unwrap_or(0.0).clamp(0.0, 100.0),
        }
    }

    /// Get current load strategy.
    pub fn strategy(&self) -> LoadStrategy {
        self.strategy
    }

    /// Switch to Alpha-N strategy.
    pub fn set_alpha_n(&mut self, cfg: AlphaNConfig) {
        self.strategy = LoadStrategy::AlphaN;
        self.alpha_n = Some(AlphaNCalculator::new(cfg));
    }

    /// Switch to Speed Density strategy.
    pub fn set_speed_density(&mut self, cfg: SpeedDensityConfig) {
        self.strategy = LoadStrategy::SpeedDensity;
        self.speed_density = Some(SpeedDensityCalculator::new(cfg));
    }
}

/// Launch control (2-step) configuration.
#[derive(Clone, Copy, Debug)]
pub struct LaunchControlConfig {
    /// Activation RPM threshold.
    pub launch_rpm: f32,
    /// RPM hysteresis for deactivation.
    pub release_rpm: f32,
    /// Minimum TPS to activate (prevents activation at idle).
    pub min_tps_pct: f32,
    /// Minimum vehicle speed to activate (optional, 0 = always active).
    pub min_speed_kph: f32,
    /// Retard timing during launch (degrees added to advance, usually negative).
    pub timing_retard_deg: f32,
    /// Whether to cut fuel during launch.
    pub fuel_cut_enabled: bool,
    /// Enable button input activation.
    pub button_activation_enabled: bool,
    /// Enable clutch switch activation.
    pub clutch_switch_activation_enabled: bool,
    /// Enable speed-based activation.
    pub speed_based_activation_enabled: bool,
}

impl Default for LaunchControlConfig {
    fn default() -> Self {
        Self {
            launch_rpm: 4000.0,
            release_rpm: 3800.0,
            min_tps_pct: 80.0,
            min_speed_kph: 0.0,
            timing_retard_deg: -10.0,
            fuel_cut_enabled: false,
            button_activation_enabled: false,
            clutch_switch_activation_enabled: false,
            speed_based_activation_enabled: false,
        }
    }
}

impl LaunchControlConfig {
    /// Create default for drag racing (aggressive).
    pub fn drag_racing() -> Self {
        Self {
            launch_rpm: 5000.0,
            release_rpm: 4800.0,
            min_tps_pct: 90.0,
            min_speed_kph: 0.0,
            timing_retard_deg: -20.0,
            fuel_cut_enabled: true,
            button_activation_enabled: true,
            clutch_switch_activation_enabled: false,
            speed_based_activation_enabled: false,
        }
    }

    /// Create default for street use (conservative).
    pub fn street() -> Self {
        Self {
            launch_rpm: 3500.0,
            release_rpm: 3300.0,
            min_tps_pct: 70.0,
            min_speed_kph: 0.0,
            timing_retard_deg: -5.0,
            fuel_cut_enabled: false,
            button_activation_enabled: false,
            clutch_switch_activation_enabled: true,
            speed_based_activation_enabled: false,
        }
    }
}

/// Launch control state machine.
#[derive(Clone, Copy, Debug)]
pub struct LaunchControl {
    cfg: LaunchControlConfig,
    active: bool,
    /// Current ignition timing offset during launch.
    timing_offset_deg: f32,
    /// Whether fuel should be cut.
    fuel_cut: bool,
}

impl LaunchControl {
    /// Create new launch control with configuration.
    pub fn new(cfg: LaunchControlConfig) -> Self {
        Self {
            cfg,
            active: false,
            timing_offset_deg: 0.0,
            fuel_cut: false,
        }
    }

    /// Update launch control state.
    ///
    /// # Arguments
    /// * `rpm` - Current engine RPM
    /// * `tps_pct` - Current TPS percentage
    /// * `speed_kph` - Current vehicle speed (optional)
    ///
    /// # Returns
    /// Modified ignition timing advance (in degrees BTDC)
    pub fn update(&mut self, rpm: f32, tps_pct: f32, speed_kph: Option<f32>) -> f32 {
        // Check activation conditions
        let speed_ok = speed_kph.map(|s| s >= self.cfg.min_speed_kph).unwrap_or(true);
        let tps_ok = tps_pct >= self.cfg.min_tps_pct;

        if self.active {
            // Currently active: stay active until RPM drops below release point
            if rpm < self.cfg.release_rpm {
                self.active = false;
                self.timing_offset_deg = 0.0;
                self.fuel_cut = false;
            }
        } else {
            // Not active: activate if all conditions met
            if rpm >= self.cfg.launch_rpm && tps_ok && speed_ok {
                self.active = true;
                self.timing_offset_deg = self.cfg.timing_retard_deg;
                self.fuel_cut = self.cfg.fuel_cut_enabled;
            }
        }

        self.timing_offset_deg
    }

    /// Check if launch control is currently active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Get current timing offset to apply to base advance.
    pub fn timing_offset(&self) -> f32 {
        if self.active {
            self.timing_offset_deg
        } else {
            0.0
        }
    }

    /// Check if fuel should be cut.
    pub fn fuel_cut(&self) -> bool {
        self.active && self.fuel_cut
    }

    /// Reset launch control (e.g., on engine stall).
    pub fn reset(&mut self) {
        self.active = false;
        self.timing_offset_deg = 0.0;
        self.fuel_cut = false;
    }

    /// Get configuration reference.
    pub fn config(&self) -> &LaunchControlConfig {
        &self.cfg
    }
}

/// Soft spark cut (random misfire for smooth RPM limiting).
pub struct SoftSparkCut {
    /// Probability of spark cut (0.0 - 1.0).
    cut_probability: f32,
    /// Random state for cut decisions.
    cut_counter: u32,
}

impl SoftSparkCut {
    /// Create new soft spark cut with given probability.
    pub fn new(cut_probability: f32) -> Self {
        Self {
            cut_probability: cut_probability.clamp(0.0, 1.0),
            cut_counter: 0,
        }
    }

    /// Determine if this spark should be cut.
    ///
    /// Uses a counter-based pseudo-random approach for no_std compatibility.
    ///
    /// # Returns
    /// `true` if spark should be cut this cycle.
    pub fn should_cut(&mut self) -> bool {
        self.cut_counter = self.cut_counter.wrapping_add(1);
        // Simple deterministic "random" based on counter
        let threshold = (self.cut_probability * 100.0) as u32;
        (self.cut_counter % 100) < threshold
    }

    /// Get current cut probability.
    pub fn probability(&self) -> f32 {
        self.cut_probability
    }

    /// Update cut probability.
    pub fn set_probability(&mut self, prob: f32) {
        self.cut_probability = prob.clamp(0.0, 1.0);
    }

    /// Reset counter.
    pub fn reset(&mut self) {
        self.cut_counter = 0;
    }
}

// ─── Multi-spark ──────────────────────────────────────────────────────────────

/// Multi-spark ignition configuration.
///
/// At low RPM, fires the coil multiple times per ignition event to improve
/// combustion quality and cold-start idle stability.
#[derive(Clone, Copy, Debug)]
pub struct MultiSparkConfig {
    /// Enable multi-spark mode.
    pub enabled: bool,
    /// Maximum number of sparks per ignition event (e.g. 3).
    pub max_sparks: u8,
    /// RPM above which multi-spark is disabled (e.g. 3000).
    pub rpm_threshold: f32,
    /// Time between consecutive sparks (milliseconds).
    pub spark_interval_ms: f32,
    /// Dwell time for each additional spark (milliseconds).
    pub dwell_per_spark_ms: f32,
}

impl Default for MultiSparkConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            max_sparks: 3,
            rpm_threshold: 3000.0,
            spark_interval_ms: 1.0,
            dwell_per_spark_ms: 1.0,
        }
    }
}

/// Multi-spark controller.
///
/// Calculates how many sparks to fire and the timing for each,
/// based on current RPM. At RPM = 0 the maximum number of sparks fires;
/// at RPM ≥ `rpm_threshold` only a single spark fires.
pub struct MultiSparkController {
    cfg: MultiSparkConfig,
}

impl MultiSparkController {
    /// Create a new multi-spark controller with the given configuration.
    pub fn new(cfg: MultiSparkConfig) -> Self {
        Self { cfg }
    }

    /// Calculate the number of sparks to fire at the given RPM.
    pub fn spark_count(&self, rpm: f32) -> u8 {
        if !self.cfg.enabled || rpm >= self.cfg.rpm_threshold {
            return 1;
        }
        if self.cfg.max_sparks <= 1 {
            return 1;
        }

        // Linear scale from max_sparks at 0 RPM to 1 spark at rpm_threshold
        let scale = 1.0 - (rpm / self.cfg.rpm_threshold).clamp(0.0, 1.0);
        let extra = libm::roundf(scale * (self.cfg.max_sparks as f32 - 1.0)) as u8;
        1 + extra
    }

    /// Total additional time required for multi-spark events (milliseconds).
    ///
    /// This must be accounted for when scheduling the ignition event relative
    /// to TDC — the first spark fires `total_duration_ms` before the intended
    /// spark angle.
    pub fn total_duration_ms(&self, rpm: f32) -> f32 {
        let count = self.spark_count(rpm);
        if count <= 1 {
            return 0.0;
        }
        let extra_sparks = (count - 1) as f32;
        extra_sparks * (self.cfg.spark_interval_ms + self.cfg.dwell_per_spark_ms)
    }

    /// Spark interval between consecutive events (milliseconds).
    pub fn spark_interval_ms(&self) -> f32 {
        self.cfg.spark_interval_ms
    }

    /// Dwell time per additional spark (milliseconds).
    pub fn dwell_per_spark_ms(&self) -> f32 {
        self.cfg.dwell_per_spark_ms
    }

    /// Whether multi-spark is enabled.
    pub fn is_enabled(&self) -> bool {
        self.cfg.enabled
    }

    /// Get configuration reference.
    pub fn config(&self) -> &MultiSparkConfig {
        &self.cfg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn tdc_angles_4cyl() {
        let order = [0u8, 2, 3, 1]; // 1-3-4-2 (0-based)
        let angles = tdc_angles_from_firing_order(&order);
        assert_eq!(angles.len(), 4);
        assert_relative_eq!(angles[0], 0.0);
        assert_relative_eq!(angles[1], 180.0);
        assert_relative_eq!(angles[2], 360.0);
        assert_relative_eq!(angles[3], 540.0);
    }

    #[cfg(feature = "cyl-4")]
    #[test]
    fn cylinder_count_feature() {
        assert_eq!(CYL_COUNT, 4);
    }

    #[cfg(feature = "cyl-1")]
    #[test]
    fn cylinder_count_1() {
        assert_eq!(CYL_COUNT, 1);
    }

    // RPM Limiter Tests
    #[test]
    fn rpm_limiter_hard_cut() {
        let cfg = RpmLimiterConfig {
            hard_limit_rpm: 7500.0,
            recovery_rpm: 7200.0,
            soft_cut_enabled: false,
        };
        let mut limiter = RpmLimiter::new(cfg);

        // Below limit - should not cut
        assert!(!limiter.update(7000.0));
        assert!(!limiter.is_active());

        // At hard limit - should cut
        assert!(limiter.update(7500.0));
        assert!(limiter.is_active());

        // Above limit - should stay cut
        assert!(limiter.update(7600.0));
        assert!(limiter.is_active());

        // Below recovery - should resume
        assert!(!limiter.update(7100.0));
        assert!(!limiter.is_active());
    }

    #[test]
    fn rpm_limiter_hysteresis() {
        let cfg = RpmLimiterConfig {
            hard_limit_rpm: 7500.0,
            recovery_rpm: 7200.0,
            soft_cut_enabled: false,
        };
        let mut limiter = RpmLimiter::new(cfg);

        // Trigger limiter
        limiter.update(7600.0);
        assert!(limiter.is_active());

        // Between recovery and hard limit - should stay cut
        assert!(limiter.update(7300.0));
        assert!(limiter.is_active());

        // Just above recovery - should stay cut
        assert!(limiter.update(7250.0));
        assert!(limiter.is_active());

        // Below recovery - should resume
        assert!(!limiter.update(7199.0));
        assert!(!limiter.is_active());
    }

    #[test]
    fn rpm_limiter_reset() {
        let cfg = RpmLimiterConfig::default_4cyl();
        let mut limiter = RpmLimiter::new(cfg);

        // Trigger and verify
        limiter.update(8000.0);
        assert!(limiter.is_active());

        // Reset
        limiter.reset();
        assert!(!limiter.is_active());
    }

    // Overdwell protection tests
    #[test]
    fn overdwell_basic_protection() {
        let cfg = OverdwellConfig::default();
        let mut ctrl = OverdwellController::new(cfg);

        assert!(!ctrl.is_charging());

        // Start charging
        ctrl.start_charge(0);
        assert!(ctrl.is_charging());

        // Before max dwell - should not trigger
        assert!(!ctrl.check_overdwell(5_000_000)); // 5ms
        assert!(ctrl.is_charging());

        // At max dwell - should trigger
        assert!(ctrl.check_overdwell(10_000_000)); // 10ms
        assert!(!ctrl.is_charging()); // Should have ended charge
        assert_eq!(ctrl.overdwell_count(), 1);
    }

    #[test]
    fn overdwell_normal_operation() {
        let cfg = OverdwellConfig::default();
        let mut ctrl = OverdwellController::new(cfg);

        // Normal charge cycle: 4ms
        ctrl.start_charge(0);
        assert!(ctrl.is_charging());

        // End charge normally before overdwell
        ctrl.end_charge();
        assert!(!ctrl.is_charging());
        assert_eq!(ctrl.overdwell_count(), 0);

        // Can start charging again
        ctrl.start_charge(10_000);
        assert!(ctrl.is_charging());
    }

    #[test]
    fn overdwell_current_dwell_time() {
        let cfg = OverdwellConfig::default();
        let mut ctrl = OverdwellController::new(cfg);

        ctrl.start_charge(0);

        // Check dwell time at different points
        assert_relative_eq!(ctrl.current_dwell_ms(1_000), 1.0, epsilon = 0.1);
        assert_relative_eq!(ctrl.current_dwell_ms(5_000), 5.0, epsilon = 0.1);

        // Should cap at max dwell
        assert_relative_eq!(ctrl.current_dwell_ms(15_000), 10.0, epsilon = 0.1);
    }

    #[test]
    fn overdwell_warning_threshold() {
        let cfg = OverdwellConfig {
            max_dwell_ms: 10.0,
            warning_threshold_pct: 0.8,
        };
        let mut ctrl = OverdwellController::new(cfg);

        ctrl.start_charge(0);

        // At 70% - no warning
        assert!(!ctrl.is_near_overdwell(7_000));

        // At 80% - warning
        assert!(ctrl.is_near_overdwell(8_000));

        // At 90% - warning
        assert!(ctrl.is_near_overdwell(9_000));
    }

    #[test]
    fn overdwell_not_charging() {
        let cfg = OverdwellConfig::default();
        let ctrl = OverdwellController::new(cfg);

        // All methods should return false/0 when not charging
        assert!(!ctrl.is_charging());
        assert_relative_eq!(ctrl.current_dwell_ms(10_000_000), 0.0, epsilon = 0.01);
        assert!(!ctrl.is_near_overdwell(10_000_000));
    }

    #[test]
    fn overdwell_reset() {
        let cfg = OverdwellConfig::default();
        let mut ctrl = OverdwellController::new(cfg);

        // Trigger overdwell
        ctrl.start_charge(0);
        ctrl.check_overdwell(15_000_000);
        assert_eq!(ctrl.overdwell_count(), 1);

        // Reset
        ctrl.reset();
        assert!(!ctrl.is_charging());
        assert_eq!(ctrl.overdwell_count(), 0);
    }

    #[test]
    fn overdwell_multiple_events() {
        let cfg = OverdwellConfig {
            max_dwell_ms: 5.0,
            ..Default::default()
        };
        let mut ctrl = OverdwellController::new(cfg);

        // Multiple overdwell events
        for i in 0..5 {
            ctrl.start_charge(i * 10_000_000);
            assert!(ctrl.check_overdwell((i + 1) * 10_000_000));
        }

        assert_eq!(ctrl.overdwell_count(), 5);
    }

    // Alpha-N load calculation tests
    #[test]
    fn alpha_n_basic_calculation() {
        let cfg = AlphaNConfig::default();
        let calc = AlphaNCalculator::new(cfg);

        // At idle: low TPS, low load
        let idle_load = calc.calculate_load(5.0, 800.0);
        assert!(idle_load > 0.0 && idle_load < 20.0);

        // At WOT: high TPS, high load
        let wot_load = calc.calculate_load(100.0, 3000.0);
        assert!(wot_load > 40.0 && wot_load <= 100.0);
    }

    #[test]
    fn alpha_n_tps_scaling() {
        let cfg = AlphaNConfig::default();
        let calc = AlphaNCalculator::new(cfg);

        let rpm = 3000.0;
        let load_0 = calc.calculate_load(0.0, rpm);
        let load_50 = calc.calculate_load(50.0, rpm);
        let load_100 = calc.calculate_load(100.0, rpm);

        // Load should increase with TPS
        assert!(load_0 < load_50);
        assert!(load_50 < load_100);
    }

    #[test]
    fn alpha_n_clamping() {
        let cfg = AlphaNConfig::default();
        let calc = AlphaNCalculator::new(cfg);

        // Should clamp to valid range
        let over_100 = calc.calculate_load(150.0, 5000.0);
        assert!(over_100 <= 100.0);

        let negative = calc.calculate_load(-10.0, 5000.0);
        assert!(negative >= 0.0);
    }

    #[test]
    fn alpha_n_table_update() {
        let cfg = AlphaNConfig::default();
        let mut calc = AlphaNCalculator::new(cfg);

        // Update table value
        calc.set_load_value(3, 3, 75.0);

        // Verify config is updated
        assert!((calc.config().load_table[3][3] - 75.0).abs() < 0.1);
    }

    // Speed Density load calculation tests
    #[test]
    fn speed_density_basic_calculation() {
        let cfg = SpeedDensityConfig::default();
        let calc = SpeedDensityCalculator::new(cfg);

        // At idle: low MAP, low load
        let idle_load = calc.calculate_load(35.0, 800.0, None);
        assert!(idle_load > 0.0 && idle_load < 40.0);

        // At WOT: high MAP, high load
        let wot_load = calc.calculate_load(100.0, 3000.0, None);
        assert!(wot_load > 60.0 && wot_load <= 100.0);
    }

    #[test]
    fn speed_density_temperature_correction() {
        let cfg = SpeedDensityConfig::default();
        let calc = SpeedDensityCalculator::new(cfg);

        let map = 80.0;
        let rpm = 3000.0;

        // Hot air (higher temp = lower load for same MAP)
        let hot_load = calc.calculate_load(map, rpm, Some(50.0));

        // Cold air (lower temp = higher load for same MAP)
        let cold_load = calc.calculate_load(map, rpm, Some(10.0));

        // Cold load should be higher than hot load
        assert!(cold_load > hot_load);
    }

    #[test]
    fn speed_density_air_mass_calculation() {
        let cfg = SpeedDensityConfig::default();
        let calc = SpeedDensityCalculator::new(cfg);

        // Calculate air mass
        let air_mass = calc.calculate_air_mass(100.0, 3000.0, 25.0);
        assert!(air_mass > 0.0);

        // Higher RPM = more air
        let air_mass_6000 = calc.calculate_air_mass(100.0, 6000.0, 25.0);
        assert!(air_mass_6000 > air_mass);
    }

    #[test]
    fn speed_density_ve_table_update() {
        let cfg = SpeedDensityConfig::default();
        let mut calc = SpeedDensityCalculator::new(cfg);

        // Update VE table value
        calc.set_ve_value(4, 4, 110.0);

        // Verify config is updated
        assert!((calc.config().ve_table[4][4] - 110.0).abs() < 0.1);
    }

    // Load calculator unified tests
    #[test]
    fn load_calculator_alpha_n() {
        let calc = LoadCalculator::new_alpha_n(AlphaNConfig::default());

        assert_eq!(calc.strategy(), LoadStrategy::AlphaN);

        let load = calc.calculate(Some(50.0), None, Some(3000.0), None);
        assert!(load > 0.0 && load <= 100.0);
    }

    #[test]
    fn load_calculator_speed_density() {
        let calc = LoadCalculator::new_speed_density(SpeedDensityConfig::default());

        assert_eq!(calc.strategy(), LoadStrategy::SpeedDensity);

        let load = calc.calculate(None, Some(80.0), Some(3000.0), Some(25.0));
        assert!(load > 0.0 && load <= 100.0);
    }

    #[test]
    fn load_calculator_simple() {
        let calc = LoadCalculator::new_simple();

        assert_eq!(calc.strategy(), LoadStrategy::Simple);

        let load = calc.calculate(Some(75.0), None, None, None);
        assert_relative_eq!(load, 75.0, epsilon = 0.1);
    }

    #[test]
    fn load_calculator_strategy_switch() {
        let mut calc = LoadCalculator::new_simple();

        // Switch to Alpha-N
        calc.set_alpha_n(AlphaNConfig::default());
        assert_eq!(calc.strategy(), LoadStrategy::AlphaN);

        // Switch to Speed Density
        calc.set_speed_density(SpeedDensityConfig::default());
        assert_eq!(calc.strategy(), LoadStrategy::SpeedDensity);
    }

    // Launch control tests
    #[test]
    fn launch_control_activation() {
        let cfg = LaunchControlConfig::default();
        let mut lc = LaunchControl::new(cfg);

        assert!(!lc.is_active());

        // Below launch RPM - should not activate
        lc.update(3500.0, 90.0, None);
        assert!(!lc.is_active());

        // At launch RPM with high TPS - should activate
        lc.update(4000.0, 85.0, None);
        assert!(lc.is_active());
    }

    #[test]
    fn launch_control_hysteresis() {
        let cfg = LaunchControlConfig::default();
        let mut lc = LaunchControl::new(cfg);

        // Activate
        lc.update(4000.0, 85.0, None);
        assert!(lc.is_active());

        // Below launch but above release - should stay active
        lc.update(3900.0, 85.0, None);
        assert!(lc.is_active());

        // Below release - should deactivate
        lc.update(3700.0, 85.0, None);
        assert!(!lc.is_active());
    }

    #[test]
    fn launch_control_tps_threshold() {
        let cfg = LaunchControlConfig::default();
        let mut lc = LaunchControl::new(cfg);

        // At launch RPM but low TPS - should not activate
        lc.update(4000.0, 50.0, None);
        assert!(!lc.is_active());

        // High TPS - should activate
        lc.update(4000.0, 85.0, None);
        assert!(lc.is_active());
    }

    #[test]
    fn launch_control_timing_retard() {
        let cfg = LaunchControlConfig {
            timing_retard_deg: -15.0,
            ..Default::default()
        };
        let mut lc = LaunchControl::new(cfg);

        // Before activation - no timing offset
        assert_relative_eq!(lc.timing_offset(), 0.0, epsilon = 0.1);

        // After activation - timing retarded
        lc.update(4000.0, 85.0, None);
        assert!(lc.is_active());
        assert_relative_eq!(lc.timing_offset(), -15.0, epsilon = 0.1);
    }

    #[test]
    fn launch_control_fuel_cut() {
        let cfg = LaunchControlConfig {
            fuel_cut_enabled: true,
            ..Default::default()
        };
        let mut lc = LaunchControl::new(cfg);

        // Before activation - no fuel cut
        assert!(!lc.fuel_cut());

        // After activation - fuel should be cut
        lc.update(4000.0, 85.0, None);
        assert!(lc.fuel_cut());
    }

    #[test]
    fn launch_control_button_activation() {
        let cfg = LaunchControlConfig::default();
        assert!(!cfg.button_activation_enabled);
        
        let cfg = LaunchControlConfig::drag_racing();
        assert!(cfg.button_activation_enabled);
    }

    #[test]
    fn launch_control_clutch_switch_activation() {
        let cfg = LaunchControlConfig::default();
        assert!(!cfg.clutch_switch_activation_enabled);
        
        let cfg = LaunchControlConfig::street();
        assert!(cfg.clutch_switch_activation_enabled);
    }

    #[test]
    fn launch_control_speed_based_activation() {
        let cfg = LaunchControlConfig::default();
        assert!(!cfg.speed_based_activation_enabled);
        
        let mut cfg = LaunchControlConfig::default();
        cfg.speed_based_activation_enabled = true;
        assert!(cfg.speed_based_activation_enabled);
    }

    #[test]
    fn launch_control_speed_threshold() {
        let cfg = LaunchControlConfig {
            min_speed_kph: 10.0,
            ..Default::default()
        };
        let mut lc = LaunchControl::new(cfg);

        // High RPM, high TPS, but low speed - should not activate
        lc.update(4000.0, 85.0, Some(5.0));
        assert!(!lc.is_active());

        // Above speed threshold - should activate
        lc.update(4000.0, 85.0, Some(15.0));
        assert!(lc.is_active());
    }

    #[test]
    fn launch_control_reset() {
        let mut lc = LaunchControl::new(LaunchControlConfig::default());

        // Activate and verify
        lc.update(4000.0, 85.0, None);
        assert!(lc.is_active());

        // Reset
        lc.reset();
        assert!(!lc.is_active());
        assert_relative_eq!(lc.timing_offset(), 0.0, epsilon = 0.1);
    }

    // Soft spark cut tests
    #[test]
    fn soft_spark_cut_basic() {
        let mut cut = SoftSparkCut::new(0.5); // 50% cut probability

        // Should cut approximately 50% of sparks over many cycles
        let mut cut_count = 0;
        for _ in 0..1000 {
            if cut.should_cut() {
                cut_count += 1;
            }
        }

        // Should be roughly 50% (450-550 range)
        assert!(cut_count > 450 && cut_count < 550);
    }

    #[test]
    fn soft_spark_cut_zero_probability() {
        let mut cut = SoftSparkCut::new(0.0);

        // Should never cut
        for _ in 0..100 {
            assert!(!cut.should_cut());
        }
    }

    #[test]
    fn soft_spark_cut_full_probability() {
        let mut cut = SoftSparkCut::new(1.0);

        // Should always cut
        for _ in 0..100 {
            assert!(cut.should_cut());
        }
    }

    #[test]
    fn soft_spark_cut_probability_update() {
        let mut cut = SoftSparkCut::new(0.0);

        // Initially never cuts
        assert!(!cut.should_cut());

        // Update to always cut
        cut.set_probability(1.0);
        assert!(cut.should_cut());
    }

    #[test]
    fn soft_spark_cut_clamping() {
        // Should clamp probability to valid range
        let cut_low = SoftSparkCut::new(-0.5);
        assert_eq!(cut_low.probability(), 0.0);

        let cut_high = SoftSparkCut::new(1.5);
        assert_eq!(cut_high.probability(), 1.0);
    }

    // MultiSpark tests
    #[test]
    fn multi_spark_disabled_returns_one() {
        let cfg = MultiSparkConfig { enabled: false, ..MultiSparkConfig::default() };
        let ctrl = MultiSparkController::new(cfg);
        assert_eq!(ctrl.spark_count(500.0), 1);
    }

    #[test]
    fn multi_spark_max_at_zero_rpm() {
        let cfg = MultiSparkConfig::default();
        let ctrl = MultiSparkController::new(cfg);
        assert_eq!(ctrl.spark_count(0.0), cfg.max_sparks);
    }

    #[test]
    fn multi_spark_one_above_threshold() {
        let cfg = MultiSparkConfig::default();
        let ctrl = MultiSparkController::new(cfg);
        assert_eq!(ctrl.spark_count(cfg.rpm_threshold + 100.0), 1);
    }

    #[test]
    fn multi_spark_scales_linearly() {
        let cfg = MultiSparkConfig { max_sparks: 3, rpm_threshold: 2000.0, ..MultiSparkConfig::default() };
        let ctrl = MultiSparkController::new(cfg);
        // At half threshold, expect 2 sparks (midpoint between 1 and 3)
        let count = ctrl.spark_count(1000.0);
        assert!(count >= 1 && count <= 3);
    }

    #[test]
    fn multi_spark_total_duration() {
        let cfg = MultiSparkConfig {
            max_sparks: 3,
            spark_interval_ms: 1.0,
            dwell_per_spark_ms: 1.0,
            rpm_threshold: 3000.0,
            ..MultiSparkConfig::default()
        };
        let ctrl = MultiSparkController::new(cfg);
        // 3 sparks: 2 extra intervals * (1.0 + 1.0) = 4 ms
        let dur = ctrl.total_duration_ms(0.0);
        assert!((dur - 4.0).abs() < 0.1);
    }
}
