//! Fuel injection calculation.
//!
//! Only compiled when the `fuel-fi` feature is enabled.
//! Derived from:
//! - `firmware/controllers/algo/fuel/fuel_computer.cpp`
//! - `firmware/controllers/algo/fuel/injector_model.cpp`

#![cfg(feature = "fuel-fi")]

pub mod ltft;
pub mod wall_wetting;

use crate::config::EngineConfig;
use crate::maps::interpolation::{interpolate1d, interpolate2d};
use crate::sensors::SensorData;

/// Standard atmosphere pressure (kPa).
const STD_ATMOSPHERE_KPA: f32 = 101.325;

/// Fuel density for gasoline (g/cc = g/mL).
const FUEL_DENSITY_G_PER_CC: f32 = 0.755;

/// Convert injector flow from cc/min to g/s.
#[inline]
fn cc_per_min_to_g_per_s(cc_per_min: f32) -> f32 {
    cc_per_min * FUEL_DENSITY_G_PER_CC / 60.0
}

/// Fuel injection result for one cylinder firing event.
#[derive(Clone, Copy, Debug)]
pub struct InjectionOutput {
    /// Fuel mass to inject in grams.
    pub fuel_mass_g: f32,
    /// Injector pulse width in milliseconds (open time + deadtime).
    pub pulse_ms: f32,
    /// Injector open time in milliseconds (excludes deadtime).
    pub open_ms: f32,
    /// Target lambda (λ) ratio.
    pub target_lambda: f32,
    /// True if cranking enrichment is being applied.
    pub cranking_enrichment_active: bool,
}

/// Cranking fuel enrichment configuration.
#[derive(Clone, Copy, Debug)]
pub struct CrankingConfig {
    /// RPM threshold below which cranking fuel mode is active.
    pub cranking_rpm: f32,
    /// Base fuel mass during cranking (mg, converted to g internally).
    pub cranking_fuel_mg: f32,
    /// Duration of post-cranking taper in engine cycles.
    pub taper_cycles: u32,
    /// Final multiplier after taper completes (e.g., 1.0 = no enrichment).
    pub final_multiplier: f32,
}

impl Default for CrankingConfig {
    fn default() -> Self {
        Self {
            cranking_rpm: 400.0,
            cranking_fuel_mg: 15.0, // 15 mg per injection during cranking
            taper_cycles: 50,       // taper over 50 engine cycles
            final_multiplier: 1.0,
        }
    }
}

/// Alpha-N fuel calculation configuration (TPS-based).
#[derive(Clone, Copy, Debug)]
pub struct AlphaNFuelConfig {
    /// RPM breakpoints for fuel table.
    pub rpm_bins: [f32; 8],
    /// TPS percentage breakpoints for fuel table.
    pub tps_bins: [f32; 8],
    /// Fuel mass table: mg per injection for each (RPM, TPS) point.
    pub fuel_table_mg: [[f32; 8]; 8],
    /// Displacement per cylinder in cc.
    pub displacement_cc_per_cyl: f32,
}

impl Default for AlphaNFuelConfig {
    fn default() -> Self {
        Self {
            rpm_bins: [0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0],
            tps_bins: [0.0, 10.0, 25.0, 50.0, 75.0, 90.0, 100.0, 100.0],
            fuel_table_mg: [
                [2.0, 3.0, 5.0, 8.0, 12.0, 15.0, 18.0, 18.0], // 0 RPM
                [2.0, 3.0, 5.0, 8.0, 12.0, 15.0, 18.0, 18.0], // 1000 RPM
                [2.5, 4.0, 7.0, 12.0, 18.0, 22.0, 28.0, 28.0], // 2000 RPM
                [3.0, 5.0, 9.0, 16.0, 24.0, 30.0, 38.0, 38.0], // 3000 RPM
                [3.5, 6.0, 11.0, 20.0, 30.0, 38.0, 48.0, 48.0], // 4000 RPM
                [4.0, 7.0, 13.0, 24.0, 36.0, 46.0, 58.0, 58.0], // 5000 RPM
                [4.5, 8.0, 15.0, 28.0, 42.0, 54.0, 68.0, 68.0], // 6000 RPM
                [5.0, 9.0, 17.0, 32.0, 48.0, 62.0, 78.0, 78.0], // 7000 RPM
            ],
            displacement_cc_per_cyl: 375.0, // 2.0L 4-cylinder
        }
    }
}

/// Alpha-N fuel calculator (TPS-based).
pub struct AlphaNFuelCalculator {
    cfg: AlphaNFuelConfig,
}

impl AlphaNFuelCalculator {
    /// Create new Alpha-N fuel calculator with configuration.
    pub fn new(cfg: AlphaNFuelConfig) -> Self {
        Self { cfg }
    }

    /// Calculate fuel mass from TPS and RPM.
    ///
    /// # Arguments
    /// * `tps_pct` - Throttle position (0-100%)
    /// * `rpm` - Engine RPM
    ///
    /// # Returns
    /// Fuel mass in milligrams
    pub fn calculate_fuel_mg(&self, tps_pct: f32, rpm: f32) -> f32 {
        interpolate2d(
            &self.cfg.fuel_table_mg,
            &self.cfg.tps_bins,
            tps_pct.clamp(0.0, 100.0),
            &self.cfg.rpm_bins,
            rpm.clamp(0.0, 20000.0),
        )
        .max(0.0)
    }

    /// Get configuration reference.
    pub fn config(&self) -> &AlphaNFuelConfig {
        &self.cfg
    }

    /// Update fuel table value at specific indices.
    pub fn set_fuel_value(&mut self, rpm_idx: usize, tps_idx: usize, value_mg: f32) {
        if rpm_idx < 8 && tps_idx < 8 {
            self.cfg.fuel_table_mg[rpm_idx][tps_idx] = value_mg.max(0.0);
        }
    }
}

/// MAF fuel calculation configuration.
#[derive(Clone, Copy, Debug)]
pub struct MafFuelConfig {
    /// MAF sensor calibration: voltage to airflow rate (g/s per volt).
    pub voltage_to_g_per_s: f32,
    /// Minimum MAF voltage (for zero flow).
    pub min_voltage: f32,
    /// Maximum MAF voltage.
    pub max_voltage: f32,
}

impl Default for MafFuelConfig {
    fn default() -> Self {
        Self {
            voltage_to_g_per_s: 50.0, // 50 g/s per volt (typical)
            min_voltage: 0.5,
            max_voltage: 5.0,
        }
    }
}

/// MAF fuel calculator (mass air flow sensor-based).
pub struct MafFuelCalculator {
    cfg: MafFuelConfig,
}

impl MafFuelCalculator {
    /// Create new MAF fuel calculator with configuration.
    pub fn new(cfg: MafFuelConfig) -> Self {
        Self { cfg }
    }

    /// Calculate air mass flow from MAF voltage.
    ///
    /// # Arguments
    /// * `maf_voltage` - MAF sensor voltage (0-5V)
    ///
    /// # Returns
    /// Air mass flow rate in g/s
    pub fn calculate_air_flow_g_per_s(&self, maf_voltage: f32) -> f32 {
        let clamped = maf_voltage.clamp(self.cfg.min_voltage, self.cfg.max_voltage);
        (clamped - self.cfg.min_voltage) * self.cfg.voltage_to_g_per_s
    }

    /// Calculate fuel mass per cylinder from MAF air flow.
    ///
    /// # Arguments
    /// * `maf_voltage` - MAF sensor voltage (0-5V)
    /// * `rpm` - Engine RPM
    /// * `cylinders` - Number of cylinders
    /// * `stoich_afr` - Stoichiometric AFR (14.7 for gasoline)
    /// * `lambda` - Target lambda (1.0 = stoichiometric)
    ///
    /// # Returns
    /// Fuel mass per cylinder in grams
    pub fn calculate_fuel_g_per_cyl(
        &self,
        maf_voltage: f32,
        rpm: f32,
        cylinders: u8,
        stoich_afr: f32,
        lambda: f32,
    ) -> f32 {
        let air_flow_g_per_s = self.calculate_air_flow_g_per_s(maf_voltage);
        if air_flow_g_per_s <= 0.0 || rpm <= 0.0 {
            return 0.0;
        }

        // Calculate events per second (firing events per cylinder)
        let events_per_sec = rpm / 120.0; // 4-stroke: 2 revolutions per cycle
        let air_per_event_g = air_flow_g_per_s / events_per_sec;
        let air_per_cyl_g = air_per_event_g / cylinders as f32;

        // Calculate fuel from air
        let afr = stoich_afr * lambda;
        air_per_cyl_g / afr
    }

    /// Get configuration reference.
    pub fn config(&self) -> &MafFuelConfig {
        &self.cfg
    }

    /// Update calibration.
    pub fn set_calibration(&mut self, voltage_to_g_per_s: f32) {
        self.cfg.voltage_to_g_per_s = voltage_to_g_per_s.max(0.0);
    }
}

/// Acceleration enrichment configuration.
#[derive(Clone, Copy, Debug)]
pub struct AccelEnrichmentConfig {
    /// TPS rate of change threshold (%/s) to trigger enrichment.
    pub tps_rate_threshold: f32,
    /// Maximum enrichment multiplier.
    pub max_multiplier: f32,
    /// Enrichment decay rate (multiplier per second).
    pub decay_rate: f32,
    /// Duration of enrichment after trigger (seconds).
    pub duration_s: f32,
}

impl Default for AccelEnrichmentConfig {
    fn default() -> Self {
        Self {
            tps_rate_threshold: 50.0, // 50%/s
            max_multiplier: 1.5,
            decay_rate: 2.0, // Decay to 1.0 over 0.25s
            duration_s: 0.5,
        }
    }
}

/// Acceleration enrichment controller.
pub struct AccelEnrichmentController {
    cfg: AccelEnrichmentConfig,
    /// Current enrichment multiplier.
    current_multiplier: f32,
    /// Timestamp when enrichment was triggered (µs).
    trigger_time_us: Option<u64>,
    /// Last TPS value for rate calculation.
    last_tps: f32,
    /// Last timestamp for rate calculation (µs).
    last_time_us: u64,
    /// Whether a previous sample has been recorded (so a timestamp of 0 is valid).
    has_last: bool,
}

impl AccelEnrichmentController {
    /// Create new acceleration enrichment controller with configuration.
    pub fn new(cfg: AccelEnrichmentConfig) -> Self {
        Self {
            cfg,
            current_multiplier: 1.0,
            trigger_time_us: None,
            last_tps: 0.0,
            last_time_us: 0,
            has_last: false,
        }
    }

    /// Update enrichment state.
    ///
    /// # Arguments
    /// * `tps_pct` - Current TPS percentage
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current fuel multiplier (>= 1.0 during enrichment)
    pub fn update(&mut self, tps_pct: f32, now_us: u64) -> f32 {
        // Calculate TPS rate of change
        let tps_rate = if self.has_last && now_us > self.last_time_us {
            let dt_s = (now_us - self.last_time_us) as f32 / 1_000_000.0;
            (tps_pct - self.last_tps) / dt_s
        } else {
            0.0
        };

        self.last_tps = tps_pct;
        self.last_time_us = now_us;
        self.has_last = true;

        // Check for acceleration trigger
        if tps_rate > self.cfg.tps_rate_threshold {
            self.trigger_time_us = Some(now_us);
            self.current_multiplier = self.cfg.max_multiplier;
        }

        // Decay enrichment
        if let Some(trigger_us) = self.trigger_time_us {
            let elapsed_s = (now_us - trigger_us) as f32 / 1_000_000.0;
            if elapsed_s > self.cfg.duration_s {
                self.current_multiplier = 1.0;
                self.trigger_time_us = None;
            } else {
                // Exponential decay using approximation
                let decay = self.cfg.decay_rate * elapsed_s;
                // Approximate exp(-x) using 1 / (1 + x + x^2/2)
                let x = decay;
                let exp_neg_x = if x < 0.1 {
                    1.0 - x // Taylor series for small x
                } else {
                    1.0 / (1.0 + x + x * x / 2.0)
                };
                self.current_multiplier = (self.cfg.max_multiplier - 1.0) * exp_neg_x + 1.0;
            }
        }

        self.current_multiplier.max(1.0)
    }

    /// Current enrichment multiplier for display/logging.
    pub fn current_multiplier(&self) -> f32 {
        self.current_multiplier.max(1.0)
    }

    /// Check if enrichment is currently active.
    pub fn is_active(&self) -> bool {
        self.current_multiplier > 1.01
    }

    /// Reset to inactive state.
    pub fn reset(&mut self) {
        self.current_multiplier = 1.0;
        self.trigger_time_us = None;
        self.last_tps = 0.0;
        self.last_time_us = 0;
        self.has_last = false;
    }

    /// Get configuration reference.
    pub fn config(&self) -> &AccelEnrichmentConfig {
        &self.cfg
    }
}

/// Injector small pulse (non-linear) correction configuration.
#[derive(Clone, Copy, Debug)]
pub struct SmallPulseConfig {
    /// Minimum pulse width for correction (ms).
    pub min_pulse_ms: f32,
    /// Correction factor at minimum pulse (additional time multiplier).
    pub min_correction: f32,
    /// Pulse width at which correction is no longer applied (ms).
    pub max_pulse_ms: f32,
}

impl Default for SmallPulseConfig {
    fn default() -> Self {
        Self {
            min_pulse_ms: 0.5,
            min_correction: 2.0,
            max_pulse_ms: 2.0,
        }
    }
}

/// Small pulse correction for injector non-linearity at low pulse widths.
pub struct SmallPulseCorrection {
    cfg: SmallPulseConfig,
}

impl SmallPulseCorrection {
    /// Create new small pulse correction with configuration.
    pub fn new(cfg: SmallPulseConfig) -> Self {
        Self { cfg }
    }

    /// Apply correction to a pulse width.
    ///
    /// # Arguments
    /// * `pulse_ms` - Original pulse width in milliseconds
    ///
    /// # Returns
    /// Corrected pulse width in milliseconds
    pub fn correct(&self, pulse_ms: f32) -> f32 {
        if pulse_ms < self.cfg.min_pulse_ms {
            // Below minimum: apply maximum correction
            pulse_ms * self.cfg.min_correction
        } else if pulse_ms > self.cfg.max_pulse_ms {
            // Above maximum: no correction
            pulse_ms
        } else {
            // Linear interpolation between min and max
            let t = (pulse_ms - self.cfg.min_pulse_ms) / (self.cfg.max_pulse_ms - self.cfg.min_pulse_ms);
            let correction = self.cfg.min_correction + (1.0 - self.cfg.min_correction) * t;
            pulse_ms * correction
        }
    }

    /// Get configuration reference.
    pub fn config(&self) -> &SmallPulseConfig {
        &self.cfg
    }
}

/// Batch injection (2-wire mode) configuration.
#[derive(Clone, Copy, Debug)]
pub struct BatchInjectionConfig {
    /// Enable batch injection mode.
    pub enabled: bool,
    /// Number of injection events per cycle (1 = all at once, 2 = split).
    pub events_per_cycle: u8,
    /// Injection timing offset in degrees (for split injection).
    pub offset_deg: f32,
}

impl Default for BatchInjectionConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            events_per_cycle: 1,
            offset_deg: 0.0,
        }
    }
}

/// Batch injection controller for 2-wire mode.
pub struct BatchInjectionController {
    cfg: BatchInjectionConfig,
}

impl BatchInjectionController {
    /// Create new batch injection controller with configuration.
    pub fn new(cfg: BatchInjectionConfig) -> Self {
        Self { cfg }
    }

    /// Calculate injection timing for batch mode.
    ///
    /// # Arguments
    /// * `cylinder_index` - Cylinder index (0-based)
    /// * `total_cylinders` - Total number of cylinders
    /// * `base_timing_deg` - Base injection timing in degrees
    ///
    /// # Returns
    /// Injection timing in degrees for the specified cylinder
    pub fn calculate_timing(&self, cylinder_index: usize, total_cylinders: u8, base_timing_deg: f32) -> f32 {
        let _ = cylinder_index;
        let _ = total_cylinders;
        
        if !self.cfg.enabled {
            return base_timing_deg;
        }

        if self.cfg.events_per_cycle == 1 {
            // Single event: all cylinders fire at same time
            base_timing_deg
        } else {
            // Split injection: offset by configured amount
            base_timing_deg + self.cfg.offset_deg
        }
    }

    /// Check if batch mode is enabled.
    pub fn is_enabled(&self) -> bool {
        self.cfg.enabled
    }

    /// Get configuration reference.
    pub fn config(&self) -> &BatchInjectionConfig {
        &self.cfg
    }

    /// Update configuration.
    pub fn set_config(&mut self, cfg: BatchInjectionConfig) {
        self.cfg = cfg;
    }
}

/// Closed loop fuel correction pause configuration (after DFCO).
#[derive(Clone, Copy, Debug)]
pub struct ClosedLoopPauseConfig {
    /// Pause duration after DFCO exit (seconds).
    pub pause_duration_s: f32,
    /// Enable pause after DFCO.
    pub enabled: bool,
}

impl Default for ClosedLoopPauseConfig {
    fn default() -> Self {
        Self {
            pause_duration_s: 2.0,
            enabled: true,
        }
    }
}

/// Closed loop fuel correction pause controller (after DFCO).
pub struct ClosedLoopPauseController {
    cfg: ClosedLoopPauseConfig,
    /// Timestamp when DFCO exited (µs).
    dfco_exit_time_us: Option<u64>,
    /// Current pause state.
    is_paused: bool,
}

impl ClosedLoopPauseController {
    /// Create new closed loop pause controller with configuration.
    pub fn new(cfg: ClosedLoopPauseConfig) -> Self {
        Self {
            cfg,
            dfco_exit_time_us: None,
            is_paused: false,
        }
    }

    /// Notify that DFCO has been exited.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    pub fn on_dfco_exit(&mut self, now_us: u64) {
        if self.cfg.enabled {
            self.dfco_exit_time_us = Some(now_us);
            self.is_paused = true;
        }
    }

    /// Update pause state.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    /// * `dfco_active` - Whether DFCO is currently active
    ///
    /// # Returns
    /// `true` if closed loop should be paused
    pub fn update(&mut self, now_us: u64, dfco_active: bool) -> bool {
        if dfco_active {
            // DFCO active: pause closed loop
            self.is_paused = true;
            return true;
        }

        if !self.cfg.enabled {
            self.is_paused = false;
            return false;
        }

        if let Some(exit_time_us) = self.dfco_exit_time_us {
            let elapsed_s = (now_us - exit_time_us) as f32 / 1_000_000.0;
            if elapsed_s > self.cfg.pause_duration_s {
                self.is_paused = false;
                self.dfco_exit_time_us = None;
            }
        } else {
            self.is_paused = false;
        }

        self.is_paused
    }

    /// Check if closed loop is currently paused.
    pub fn is_paused(&self) -> bool {
        self.is_paused
    }

    /// Reset to unpaused state.
    pub fn reset(&mut self) {
        self.dfco_exit_time_us = None;
        self.is_paused = false;
    }

    /// Get configuration reference.
    pub fn config(&self) -> &ClosedLoopPauseConfig {
        &self.cfg
    }
}

/// Deceleration Fuel Cut Off (DFCO) configuration.
#[derive(Clone, Copy, Debug)]
pub struct DfcoConfig {
    /// Minimum RPM to enable DFCO (below this, fuel is restored).
    pub min_rpm: f32,
    /// Maximum MAP to enable DFCO (below this threshold = decelerating).
    pub max_map_kpa: f32,
    /// Minimum TPS to enable DFCO (TPS must be near 0%).
    pub max_tps_pct: f32,
    /// Delay before engaging DFCO after conditions are met (seconds).
    pub delay_secs: f32,
    /// RPM threshold below which DFCO is disabled (engine will stall).
    pub cutoff_restore_rpm: f32,
}

impl Default for DfcoConfig {
    fn default() -> Self {
        Self {
            min_rpm: 1500.0,
            max_map_kpa: 35.0,
            max_tps_pct: 1.0,
            delay_secs: 0.5,
            cutoff_restore_rpm: 1200.0,
        }
    }
}

/// DFCO state machine.
#[derive(Clone, Copy, Debug, Default)]
pub struct DfcoController {
    cfg: DfcoConfig,
    active: bool,
    /// Timestamp when DFCO conditions were first met (µs).
    condition_met_us: Option<u64>,
}

impl DfcoController {
    /// Create a new DFCO controller with the given configuration.
    pub fn new(cfg: DfcoConfig) -> Self {
        Self {
            cfg,
            active: false,
            condition_met_us: None,
        }
    }

    /// Update DFCO state and return whether fuel should be cut.
    ///
    /// # Arguments
    /// * `rpm` - Current engine RPM
    /// * `map_kpa` - Manifold absolute pressure in kPa
    /// * `tps_pct` - Throttle position in percent (0-100)
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if fuel injection should be cut (DFCO active).
    pub fn update(&mut self, rpm: f32, map_kpa: f32, tps_pct: f32, now_us: u64) -> bool {
        // Check if DFCO conditions are met
        let conditions_met = rpm > self.cfg.min_rpm
            && map_kpa < self.cfg.max_map_kpa
            && tps_pct < self.cfg.max_tps_pct;

        if self.active {
            // Already active: check if we should restore fuel
            // Restore if RPM dropped below cutoff threshold, or conditions no longer met
            let should_restore = rpm < self.cfg.cutoff_restore_rpm || !conditions_met;
            if should_restore {
                self.active = false;
                self.condition_met_us = None;
            }
        } else {
            // Not active: check if we should engage
            if conditions_met {
                if let Some(start_us) = self.condition_met_us {
                    // Conditions have been met for some time
                    let elapsed_s = (now_us - start_us) as f32 / 1_000_000.0;
                    if elapsed_s >= self.cfg.delay_secs {
                        self.active = true;
                    }
                } else {
                    // First time conditions are met
                    self.condition_met_us = Some(now_us);
                }
            } else {
                // Conditions not met, reset timer
                self.condition_met_us = None;
            }
        }

        self.active
    }

    /// Current DFCO state for display/logging.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Reset DFCO controller (e.g., on engine stall).
    pub fn reset(&mut self) {
        self.active = false;
        self.condition_met_us = None;
    }
}

/// Post-cranking enrichment taper state.
#[derive(Clone, Copy, Debug)]
pub struct CrankingTaper {
    cfg: CrankingConfig,
    cycles_since_start: u32,
    active: bool,
}

impl CrankingTaper {
    /// Create a new taper controller with the given configuration.
    pub fn new(cfg: CrankingConfig) -> Self {
        Self {
            cfg,
            cycles_since_start: 0,
            active: false,
        }
    }

    /// Call when engine starts (RPM transitions from 0 to >0).
    pub fn on_engine_start(&mut self) {
        self.cycles_since_start = 0;
        self.active = true;
    }

    /// Call at each engine cycle to advance the taper.
    /// Returns the current fuel multiplier (>= 1.0 during enrichment).
    pub fn update(&mut self, rpm: f32) -> f32 {
        if !self.active {
            return 1.0;
        }

        if rpm <= 0.0 {
            // Engine stopped, reset
            self.active = false;
            self.cycles_since_start = 0;
            return 1.0;
        }

        if rpm < self.cfg.cranking_rpm {
            // Still cranking: use full enrichment
            return self.cfg.cranking_fuel_mg / 10.0; // Scale mg to ~multiplier range
        }

        // Taper phase
        self.cycles_since_start += 1;

        if self.cycles_since_start >= self.cfg.taper_cycles {
            // Taper complete
            self.active = false;
            return self.cfg.final_multiplier;
        }

        // Linear interpolation from cranking multiplier to final multiplier
        let t = self.cycles_since_start as f32 / self.cfg.taper_cycles as f32;
        let start_mult = self.cfg.cranking_fuel_mg / 10.0;
        start_mult + (self.cfg.final_multiplier - start_mult) * t
    }

    /// Current enrichment multiplier for display/logging.
    pub fn current_multiplier(&self) -> f32 {
        if !self.active {
            return 1.0;
        }
        let t = (self.cycles_since_start as f32 / self.cfg.taper_cycles as f32).min(1.0);
        let start_mult = self.cfg.cranking_fuel_mg / 10.0;
        start_mult + (self.cfg.final_multiplier - start_mult) * t
    }

    /// Reset to inactive state.
    pub fn reset(&mut self) {
        self.active = false;
        self.cycles_since_start = 0;
    }
}

/// Compute the fuel injection pulse for one cylinder firing event.
///
/// # Arguments
/// * `cfg`       — engine configuration
/// * `sensors`   — current sensor readings
/// * `airmass_g` — estimated air mass in the cylinder (grams)
///
/// # Returns
/// `Some(InjectionOutput)` if all required sensors are valid, `None` otherwise.
pub fn compute_injection(
    cfg: &EngineConfig,
    sensors: &SensorData,
    airmass_g: f32,
) -> Option<InjectionOutput> {
    let rpm = sensors.rpm?;
    let load = sensors.load_pct.unwrap_or(0.0);

    // ── target lambda ────────────────────────────────────────────────────────
    let lambda = interpolate2d(
        &cfg.lambda_table,
        &cfg.lambda_load_bins,
        load,
        &cfg.lambda_rpm_bins,
        rpm,
    );

    // ── stoichiometric ratio ─────────────────────────────────────────────────
    let stoich = cfg.stoich_ratio_primary; // 14.7 for gasoline

    // ── required fuel mass ───────────────────────────────────────────────────
    let afr = stoich * lambda;
    let fuel_mass_g = airmass_g / afr;

    if fuel_mass_g <= 0.0 {
        return None;
    }

    // ── injector flow rate ───────────────────────────────────────────────────
    let flow_g_per_s = cc_per_min_to_g_per_s(cfg.injector_flow_cc_per_min);

    // ── deadtime (battery voltage correction) ────────────────────────────────
    let vbatt = sensors.battery_volts.unwrap_or(12.0);
    let deadtime_ms = interpolate2d(
        &cfg.injector_deadtime_table,
        &cfg.injector_deadtime_pressure_bins,
        STD_ATMOSPHERE_KPA, // simplified: no fuel pressure sensor
        &cfg.injector_deadtime_voltage_bins,
        vbatt,
    );

    // ── pulse width ──────────────────────────────────────────────────────────
    // open_ms = fuel_mass / flow_rate * 1000  (ms)
    let open_ms = fuel_mass_g / flow_g_per_s * 1000.0;
    let pulse_ms = open_ms + deadtime_ms;

    Some(InjectionOutput {
        fuel_mass_g,
        pulse_ms,
        open_ms,
        target_lambda: lambda,
        cranking_enrichment_active: false,
    })
}

/// Simplified air mass estimate from MAP and displacement.
///
/// Uses the ideal gas law approximation:
/// `airmass ≈ displacement_cc / cylinders * MAP_kPa / 101.325 * volumetric_efficiency`
pub fn estimate_airmass_g(
    map_kpa: f32,
    displacement_cc_per_cyl: f32,
    volumetric_efficiency: f32,
) -> f32 {
    // Air density at standard conditions: ~1.293 g/L = 1.293e-3 g/cc
    const AIR_DENSITY_G_PER_CC: f32 = 1.293e-3;
    displacement_cc_per_cyl * (map_kpa / STD_ATMOSPHERE_KPA) * volumetric_efficiency * AIR_DENSITY_G_PER_CC
}

/// Speed Density air mass calculation using VE table and temperature corrections.
///
/// # Formula
/// ```text
/// airmass = displacement * (MAP / 101.325) * VE(RPM, MAP) * IAT_correction * CLT_correction * air_density_std
/// ```
///
/// # Arguments
/// * `cfg` - Engine configuration containing VE table and correction tables
/// * `rpm` - Current engine RPM
/// * `map_kpa` - Manifold absolute pressure in kPa
/// * `iat_c` - Intake air temperature in °C
/// * `clt_c` - Coolant temperature in °C
/// * `displacement_cc_per_cyl` - Displacement per cylinder in cc
///
/// # Returns
/// Estimated air mass in grams, or `None` if required sensors are invalid.
pub fn calculate_airmass_speed_density(
    cfg: &EngineConfig,
    rpm: f32,
    map_kpa: f32,
    iat_c: f32,
    clt_c: f32,
    displacement_cc_per_cyl: f32,
) -> Option<f32> {
    // Get VE from table using RPM and MAP (load)
    let ve = interpolate2d(
        &cfg.ve_table,
        &cfg.ve_load_bins,
        map_kpa,
        &cfg.ve_rpm_bins,
        rpm,
    );

    // IAT correction: cold air is denser, hot air is less dense
    // Formula: correction = 273.15 / (IAT + 273.15) * iat_fuel_corr
    // First apply the density correction from ideal gas law
    const STD_TEMP_K: f32 = 273.15; // 0°C in Kelvin
    let iat_k = iat_c + STD_TEMP_K;
    let temp_correction = STD_TEMP_K / iat_k;

    // Apply additional fuel correction factor from table
    let iat_fuel_mult = interpolate1d(&cfg.iat_fuel_temp_bins, &cfg.iat_fuel_corr, iat_c);

    // CLT enrichment (more fuel when cold)
    let clt_fuel_mult = interpolate1d(&cfg.clt_fuel_temp_bins, &cfg.clt_fuel_corr, clt_c);

    // Air density at standard conditions: ~1.293 g/L = 1.293e-3 g/cc
    const AIR_DENSITY_G_PER_CC: f32 = 1.293e-3;

    // Calculate air mass
    let base_mass = displacement_cc_per_cyl
        * (map_kpa / STD_ATMOSPHERE_KPA)
        * ve
        * AIR_DENSITY_G_PER_CC;

    // Apply all corrections
    let corrected_mass = base_mass * temp_correction * iat_fuel_mult * clt_fuel_mult;

    Some(corrected_mass)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn cc_per_min_conversion() {
        // 240 cc/min should be ~3.014 g/s
        let g_per_s = cc_per_min_to_g_per_s(240.0);
        assert_relative_eq!(g_per_s, 240.0 * 0.755 / 60.0, epsilon = 0.001);
    }

    #[test]
    fn airmass_estimate_at_std_atm() {
        // 250 cc/cyl, MAP = 101.325 kPa (WOT), VE = 0.85
        let mass = estimate_airmass_g(101.325, 250.0, 0.85);
        // Expected: 250 * 1.0 * 0.85 * 1.293e-3 ≈ 0.2748 g
        assert_relative_eq!(mass, 0.2748, epsilon = 0.001);
    }

    #[test]
    fn speed_density_basic_calculation() {
        let cfg = EngineConfig::default_4cyl();

        // Standard conditions: 20°C IAT, 80°C CLT, 100 kPa MAP, 3000 RPM
        let mass = calculate_airmass_speed_density(&cfg, 3000.0, 100.0, 20.0, 80.0, 375.0);

        assert!(mass.is_some());
        let m = mass.unwrap();

        // Expected calculation:
        // Base: 375 * (100/101.325) * 0.85 * 1.293e-3 ≈ 0.407 g
        // Temp correction: 273.15 / (20 + 273.15) ≈ 0.932
        // IAT fuel corr at 20°C: ~1.0
        // CLT fuel corr at 80°C: ~1.0
        // Expected: ~0.407 * 0.932 ≈ 0.379 g
        assert!(m > 0.3 && m < 0.5, "Air mass should be ~0.38g, got {}", m);
    }

    #[test]
    fn speed_density_iat_correction() {
        let cfg = EngineConfig::default_4cyl();

        // Cold IAT (-20°C) should give more air mass (denser air)
        let mass_cold = calculate_airmass_speed_density(&cfg, 3000.0, 100.0, -20.0, 80.0, 375.0);

        // Hot IAT (60°C) should give less air mass (less dense air)
        let mass_hot = calculate_airmass_speed_density(&cfg, 3000.0, 100.0, 60.0, 80.0, 375.0);

        assert!(mass_cold.is_some());
        assert!(mass_hot.is_some());

        // Cold air should have more mass than hot air
        assert!(
            mass_cold.unwrap() > mass_hot.unwrap(),
            "Cold air should be denser than hot air"
        );
    }

    #[test]
    fn speed_density_clt_enrichment() {
        let cfg = EngineConfig::default_4cyl();

        // Cold engine (-20°C CLT) should have enrichment
        let mass_cold_clt = calculate_airmass_speed_density(&cfg, 3000.0, 100.0, 20.0, -20.0, 375.0);

        // Warm engine (80°C CLT) should have no enrichment
        let mass_warm_clt = calculate_airmass_speed_density(&cfg, 3000.0, 100.0, 20.0, 80.0, 375.0);

        assert!(mass_cold_clt.is_some());
        assert!(mass_warm_clt.is_some());

        // Cold engine should have more fuel (airmass is used to calculate fuel)
        assert!(
            mass_cold_clt.unwrap() > mass_warm_clt.unwrap(),
            "Cold engine should have enrichment multiplier"
        );
    }

    // CrankingTaper tests
    #[test]
    fn cranking_taper_full_enrichment() {
        let cfg = CrankingConfig::default();
        let mut taper = CrankingTaper::new(cfg);

        taper.on_engine_start();

        // Below cranking RPM: full enrichment
        let mult = taper.update(300.0); // 300 RPM cranking
        assert!(mult > 1.0, "Should have enrichment during cranking");
        assert!(taper.current_multiplier() > 1.0);
    }

    #[test]
    fn cranking_taper_transition() {
        let cfg = CrankingConfig {
            cranking_rpm: 400.0,
            cranking_fuel_mg: 20.0,
            taper_cycles: 10,
            final_multiplier: 1.0,
        };
        let mut taper = CrankingTaper::new(cfg);

        taper.on_engine_start();

        // Above cranking RPM: enter taper phase
        let start_mult = taper.update(1000.0);
        assert!(start_mult > 1.0, "Should start with enrichment");

        // Simulate taper over several cycles
        let mut last_mult = start_mult;
        for _ in 0..10 {
            let mult = taper.update(1000.0);
            assert!(
                mult <= last_mult,
                "Multiplier should decrease during taper"
            );
            last_mult = mult;
        }

        // After taper complete
        let final_mult = taper.update(1000.0);
        assert_relative_eq!(final_mult, 1.0, epsilon = 0.01);
        assert!(!taper.active);
    }

    #[test]
    fn cranking_taper_reset() {
        let cfg = CrankingConfig::default();
        let mut taper = CrankingTaper::new(cfg);

        taper.on_engine_start();
        taper.update(1000.0); // Start taper
        assert!(taper.active);

        taper.reset();
        assert!(!taper.active);
        assert_relative_eq!(taper.current_multiplier(), 1.0, epsilon = 0.01);
    }

    // DFCO tests
    #[test]
    fn dfco_basic_activation() {
        let cfg = DfcoConfig::default();
        let mut dfco = DfcoController::new(cfg);

        // Not active initially
        assert!(!dfco.update(2000.0, 30.0, 0.0, 0)); // High RPM, low MAP, closed throttle

        // Wait for delay
        let delay_us = (cfg.delay_secs * 1_000_000.0) as u64 + 100_000;
        assert!(dfco.update(2000.0, 30.0, 0.0, delay_us));
        assert!(dfco.is_active());
    }

    #[test]
    fn dfco_conditions_not_met() {
        let cfg = DfcoConfig::default();
        let mut dfco = DfcoController::new(cfg);

        // High MAP (not decelerating)
        assert!(!dfco.update(2000.0, 60.0, 0.0, 1_000_000));
        assert!(!dfco.is_active());

        // High TPS (throttle open)
        assert!(!dfco.update(2000.0, 30.0, 10.0, 1_000_000));
        assert!(!dfco.is_active());

        // Low RPM
        assert!(!dfco.update(1000.0, 30.0, 0.0, 1_000_000));
        assert!(!dfco.is_active());
    }

    #[test]
    fn dfco_restore_conditions() {
        let cfg = DfcoConfig::default();
        let mut dfco = DfcoController::new(cfg);

        // Activate DFCO (arm, then engage once the delay has elapsed).
        let delay_us = (cfg.delay_secs * 1_000_000.0) as u64 + 100_000;
        dfco.update(2000.0, 30.0, 0.0, 0);
        dfco.update(2000.0, 30.0, 0.0, delay_us);
        assert!(dfco.is_active());

        // Restore: RPM below threshold
        assert!(!dfco.update(1100.0, 30.0, 0.0, delay_us + 100_000));
        assert!(!dfco.is_active());

        // Reactivate (re-arm, then engage after the delay again).
        dfco.update(2000.0, 30.0, 0.0, delay_us + 200_000);
        dfco.update(2000.0, 30.0, 0.0, delay_us + 200_000 + delay_us);
        assert!(dfco.is_active());

        // Restore: TPS applied
        assert!(!dfco.update(2000.0, 30.0, 5.0, delay_us + 200_000 + delay_us + 100_000));
        assert!(!dfco.is_active());
    }

    #[test]
    fn dfco_delay_not_met() {
        let cfg = DfcoConfig {
            delay_secs: 1.0,
            ..DfcoConfig::default()
        };
        let mut dfco = DfcoController::new(cfg);

        // Conditions met but delay not passed
        assert!(!dfco.update(2000.0, 30.0, 0.0, 0));
        assert!(!dfco.update(2000.0, 30.0, 0.0, 500_000)); // 0.5s later
        assert!(!dfco.is_active());

        // After delay
        assert!(dfco.update(2000.0, 30.0, 0.0, 1_100_000)); // 1.1s later
        assert!(dfco.is_active());
    }

    #[test]
    fn dfco_reset() {
        let cfg = DfcoConfig::default();
        let mut dfco = DfcoController::new(cfg);

        let delay_us = (cfg.delay_secs * 1_000_000.0) as u64 + 100_000;
        dfco.update(2000.0, 30.0, 0.0, 0);
        dfco.update(2000.0, 30.0, 0.0, delay_us);
        assert!(dfco.is_active());

        dfco.reset();
        assert!(!dfco.is_active());
    }

    // Alpha-N fuel calculation tests
    #[test]
    fn alpha_n_basic_calculation() {
        let cfg = AlphaNFuelConfig::default();
        let calc = AlphaNFuelCalculator::new(cfg);

        // At idle: low TPS, low fuel
        let idle_fuel = calc.calculate_fuel_mg(5.0, 800.0);
        assert!(idle_fuel > 0.0 && idle_fuel < 10.0);

        // At WOT: high TPS, high fuel
        let wot_fuel = calc.calculate_fuel_mg(100.0, 3000.0);
        assert!(wot_fuel > 20.0);
    }

    #[test]
    fn alpha_n_rpm_scaling() {
        let cfg = AlphaNFuelConfig::default();
        let calc = AlphaNFuelCalculator::new(cfg);

        let tps = 50.0;
        let fuel_2000 = calc.calculate_fuel_mg(tps, 2000.0);
        let fuel_6000 = calc.calculate_fuel_mg(tps, 6000.0);

        // Higher RPM = more fuel
        assert!(fuel_6000 > fuel_2000);
    }

    #[test]
    fn alpha_n_table_update() {
        let cfg = AlphaNFuelConfig::default();
        let mut calc = AlphaNFuelCalculator::new(cfg);

        calc.set_fuel_value(3, 3, 25.0);

        assert!((calc.config().fuel_table_mg[3][3] - 25.0).abs() < 0.1);
    }

    // MAF fuel calculation tests
    #[test]
    fn maf_air_flow_calculation() {
        let cfg = MafFuelConfig::default();
        let calc = MafFuelCalculator::new(cfg);

        // 2.5V should give ~100 g/s (2.5V - 0.5V) * 50 g/s/V
        let flow = calc.calculate_air_flow_g_per_s(2.5);
        assert!(flow > 90.0 && flow < 110.0);
    }

    #[test]
    fn maf_clamping() {
        let cfg = MafFuelConfig::default();
        let calc = MafFuelCalculator::new(cfg);

        // Below minimum
        let low = calc.calculate_air_flow_g_per_s(0.2);
        assert_relative_eq!(low, 0.0, epsilon = 0.01);

        // Above maximum
        let high = calc.calculate_air_flow_g_per_s(6.0);
        assert!(high > 0.0);
    }

    #[test]
    fn maf_fuel_calculation() {
        let cfg = MafFuelConfig::default();
        let calc = MafFuelCalculator::new(cfg);

        // Calculate fuel from MAF
        let fuel_g = calc.calculate_fuel_g_per_cyl(2.5, 3000.0, 4, 14.7, 1.0);
        assert!(fuel_g > 0.0);
    }

    // Acceleration enrichment tests
    #[test]
    fn accel_enrichment_trigger() {
        let cfg = AccelEnrichmentConfig::default();
        let mut accel = AccelEnrichmentController::new(cfg);

        // Slow TPS change - no enrichment
        let mult1 = accel.update(10.0, 1_000_000);
        assert_relative_eq!(mult1, 1.0, epsilon = 0.01);

        // Fast TPS change (50% in 0.5s = 100%/s > threshold) - should trigger
        let mult2 = accel.update(60.0, 1_500_000);
        assert!(mult2 > 1.2);
    }

    #[test]
    fn accel_enrichment_decay() {
        let cfg = AccelEnrichmentConfig {
            duration_s: 0.5,
            ..Default::default()
        };
        let mut accel = AccelEnrichmentController::new(cfg);

        // Trigger enrichment
        accel.update(0.0, 0);
        accel.update(60.0, 1000); // Fast change
        assert!(accel.is_active());

        // After duration - should be inactive
        let mult = accel.update(60.0, 600_000); // 0.6s later
        assert_relative_eq!(mult, 1.0, epsilon = 0.01);
        assert!(!accel.is_active());
    }

    #[test]
    fn accel_enrichment_reset() {
        let cfg = AccelEnrichmentConfig::default();
        let mut accel = AccelEnrichmentController::new(cfg);

        // Trigger enrichment
        accel.update(0.0, 0);
        accel.update(60.0, 1000);
        assert!(accel.is_active());

        // Reset
        accel.reset();
        assert!(!accel.is_active());
        assert_relative_eq!(accel.current_multiplier(), 1.0, epsilon = 0.01);
    }

    // Small pulse correction tests
    #[test]
    fn small_pulse_correction_below_min() {
        let cfg = SmallPulseConfig::default();
        let corr = SmallPulseCorrection::new(cfg);

        // Below minimum: apply maximum correction (2x)
        let corrected = corr.correct(0.3);
        assert_relative_eq!(corrected, 0.6, epsilon = 0.01);
    }

    #[test]
    fn small_pulse_correction_above_max() {
        let cfg = SmallPulseConfig::default();
        let corr = SmallPulseCorrection::new(cfg);

        // Above maximum: no correction
        let corrected = corr.correct(3.0);
        assert_relative_eq!(corrected, 3.0, epsilon = 0.01);
    }

    #[test]
    fn small_pulse_correction_interpolation() {
        let cfg = SmallPulseConfig::default();
        let corr = SmallPulseCorrection::new(cfg);

        // At midpoint: should be ~1.5x correction
        let corrected = corr.correct(1.25); // Midpoint between 0.5 and 2.0
        assert!(corrected > 1.25 && corrected < 2.5); // Should be between 1x and 2x
    }

    #[test]
    fn small_pulse_correction_at_min() {
        let cfg = SmallPulseConfig::default();
        let corr = SmallPulseCorrection::new(cfg);

        // At minimum boundary
        let corrected = corr.correct(0.5);
        assert_relative_eq!(corrected, 1.0, epsilon = 0.01); // 0.5 * 2.0 = 1.0
    }

    #[test]
    fn small_pulse_correction_at_max() {
        let cfg = SmallPulseConfig::default();
        let corr = SmallPulseCorrection::new(cfg);

        // At maximum boundary
        let corrected = corr.correct(2.0);
        assert_relative_eq!(corrected, 2.0, epsilon = 0.01); // No correction
    }

    // Batch injection tests
    #[test]
    fn batch_injection_disabled() {
        let cfg = BatchInjectionConfig {
            enabled: false,
            ..Default::default()
        };
        let batch = BatchInjectionController::new(cfg);

        let timing = batch.calculate_timing(0, 4, 360.0);
        assert_relative_eq!(timing, 360.0, epsilon = 0.01);
    }

    #[test]
    fn batch_injection_single_event() {
        let cfg = BatchInjectionConfig {
            enabled: true,
            events_per_cycle: 1,
            ..Default::default()
        };
        let batch = BatchInjectionController::new(cfg);

        let timing = batch.calculate_timing(0, 4, 360.0);
        assert_relative_eq!(timing, 360.0, epsilon = 0.01);
    }

    #[test]
    fn batch_injection_split_event() {
        let cfg = BatchInjectionConfig {
            enabled: true,
            events_per_cycle: 2,
            offset_deg: 180.0,
            ..Default::default()
        };
        let batch = BatchInjectionController::new(cfg);

        let timing = batch.calculate_timing(0, 4, 360.0);
        assert_relative_eq!(timing, 540.0, epsilon = 0.01); // 360 + 180
    }

    #[test]
    fn batch_injection_is_enabled() {
        let cfg = BatchInjectionConfig {
            enabled: true,
            ..Default::default()
        };
        let batch = BatchInjectionController::new(cfg);

        assert!(batch.is_enabled());
    }

    #[test]
    fn batch_injection_config_update() {
        let cfg = BatchInjectionConfig::default();
        let mut batch = BatchInjectionController::new(cfg);

        assert!(!batch.is_enabled());

        let new_cfg = BatchInjectionConfig {
            enabled: true,
            events_per_cycle: 2,
            offset_deg: 90.0,
        };
        batch.set_config(new_cfg);

        assert!(batch.is_enabled());
    }

    // Closed loop pause tests
    #[test]
    fn closed_loop_pause_dfco_active() {
        let cfg = ClosedLoopPauseConfig::default();
        let mut pause = ClosedLoopPauseController::new(cfg);

        let paused = pause.update(0, true); // DFCO active
        assert!(paused);
    }

    #[test]
    fn closed_loop_pause_after_dfco_exit() {
        let cfg = ClosedLoopPauseConfig {
            pause_duration_s: 1.0,
            ..Default::default()
        };
        let mut pause = ClosedLoopPauseController::new(cfg);

        // DFCO exit at t=0
        pause.on_dfco_exit(0);

        // Still paused at t=0.5s
        let paused = pause.update(500_000, false);
        assert!(paused);

        // Resume after 1s
        let paused = pause.update(1_500_000, false);
        assert!(!paused);
    }

    #[test]
    fn closed_loop_pause_expired() {
        let cfg = ClosedLoopPauseConfig {
            pause_duration_s: 1.0,
            ..Default::default()
        };
        let mut pause = ClosedLoopPauseController::new(cfg);

        // DFCO exit at t=0
        pause.on_dfco_exit(0);

        // Not paused after 1.5s
        let paused = pause.update(1_500_000, false);
        assert!(!paused);
    }

    #[test]
    fn closed_loop_pause_disabled() {
        let cfg = ClosedLoopPauseConfig {
            enabled: false,
            ..Default::default()
        };
        let mut pause = ClosedLoopPauseController::new(cfg);

        pause.on_dfco_exit(0);
        let paused = pause.update(500_000, false);
        assert!(!paused);
    }

    #[test]
    fn closed_loop_pause_reset() {
        let cfg = ClosedLoopPauseConfig::default();
        let mut pause = ClosedLoopPauseController::new(cfg);

        pause.on_dfco_exit(0);
        pause.reset();

        let paused = pause.update(500_000, false);
        assert!(!paused);
    }

    #[test]
    fn closed_loop_pause_is_paused() {
        let cfg = ClosedLoopPauseConfig::default();
        let mut pause = ClosedLoopPauseController::new(cfg);

        pause.on_dfco_exit(0);
        assert!(pause.is_paused());
    }
}

/// Flex fuel configuration for ethanol content sensor.
#[derive(Clone, Copy, Debug)]
pub struct FlexFuelConfig {
    /// Enable flex fuel compensation.
    pub enabled: bool,
    /// Stoichiometric AFR for pure gasoline (14.7:1 by default).
    pub afr_gasoline: f32,
    /// Stoichiometric AFR for pure ethanol (9.0:1 by default).
    pub afr_ethanol: f32,
    /// Ethanol content at 0% sensor reading (for calibration).
    pub ethanol_percent_0v: f32,
    /// Ethanol content at 5V sensor reading (for calibration).
    pub ethanol_percent_5v: f32,
}

impl Default for FlexFuelConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            afr_gasoline: 14.7,
            afr_ethanol: 9.0,
            ethanol_percent_0v: 0.0,
            ethanol_percent_5v: 100.0,
        }
    }
}

/// Flex fuel controller for ethanol content compensation.
pub struct FlexFuelController {
    cfg: FlexFuelConfig,
    /// Current ethanol content percentage (0-100%).
    ethanol_content: f32,
}

impl FlexFuelController {
    /// Create new flex fuel controller.
    pub fn new(cfg: FlexFuelConfig) -> Self {
        Self {
            cfg,
            ethanol_content: 0.0,
        }
    }

    /// Update ethanol content from sensor voltage.
    ///
    /// # Arguments
    /// * `sensor_voltage` - Flex fuel sensor voltage (0-5V)
    pub fn update(&mut self, sensor_voltage: f32) {
        if !self.cfg.enabled {
            self.ethanol_content = 0.0;
            return;
        }

        // Map voltage to ethanol percentage
        let voltage = sensor_voltage.clamp(0.0, 5.0);
        let normalized = voltage / 5.0;
        self.ethanol_content = self.cfg.ethanol_percent_0v
            + normalized * (self.cfg.ethanol_percent_5v - self.cfg.ethanol_percent_0v);
        self.ethanol_content = self.ethanol_content.clamp(0.0, 100.0);
    }

    /// Get fuel correction factor for current ethanol content.
    ///
    /// Returns multiplier > 1.0 for ethanol-rich fuel (needs more fuel).
    pub fn get_fuel_correction(&self) -> f32 {
        if !self.cfg.enabled {
            return 1.0;
        }

        // Linear interpolation between gasoline and ethanol stoichiometric AFR
        let afr_target = self.cfg.afr_gasoline
            + (self.ethanol_content / 100.0) * (self.cfg.afr_ethanol - self.cfg.afr_gasoline);
        
        // Correction factor = gasoline AFR / target AFR
        // Ethanol needs more fuel, so correction > 1.0
        self.cfg.afr_gasoline / afr_target
    }

    /// Get current ethanol content percentage.
    pub fn ethanol_content(&self) -> f32 {
        self.ethanol_content
    }

    /// Get configuration reference.
    pub fn config(&self) -> &FlexFuelConfig {
        &self.cfg
    }

    /// Update configuration.
    pub fn set_config(&mut self, cfg: FlexFuelConfig) {
        self.cfg = cfg;
    }
}

#[cfg(test)]
mod flex_fuel_tests {
    use super::*;

    #[test]
    fn flex_fuel_disabled_returns_1() {
        let cfg = FlexFuelConfig::default();
        let mut ctrl = FlexFuelController::new(cfg);
        
        ctrl.update(2.5);
        assert_eq!(ctrl.ethanol_content(), 0.0);
        assert_eq!(ctrl.get_fuel_correction(), 1.0);
    }

    #[test]
    fn flex_fuel_enabled_maps_voltage() {
        let mut cfg = FlexFuelConfig::default();
        cfg.enabled = true;
        let mut ctrl = FlexFuelController::new(cfg);
        
        ctrl.update(0.0);
        assert_eq!(ctrl.ethanol_content(), 0.0);
        
        ctrl.update(5.0);
        assert_eq!(ctrl.ethanol_content(), 100.0);
        
        ctrl.update(2.5);
        assert_eq!(ctrl.ethanol_content(), 50.0);
    }

    #[test]
    fn flex_fuel_correction_factor() {
        let mut cfg = FlexFuelConfig::default();
        cfg.enabled = true;
        let mut ctrl = FlexFuelController::new(cfg);
        
        ctrl.update(0.0); // Pure gasoline
        let corr = ctrl.get_fuel_correction();
        assert!((corr - 1.0).abs() < 0.01);
        
        ctrl.update(5.0); // Pure ethanol
        let corr = ctrl.get_fuel_correction();
        // 14.7 / 9.0 = 1.633
        assert!((corr - 1.633).abs() < 0.01);
        
        ctrl.update(2.5); // 50% ethanol
        let corr = ctrl.get_fuel_correction();
        assert!(corr > 1.0 && corr < 1.633);
    }

    #[test]
    fn flex_fuel_voltage_clamped() {
        let mut cfg = FlexFuelConfig::default();
        cfg.enabled = true;
        let mut ctrl = FlexFuelController::new(cfg);
        
        ctrl.update(-1.0);
        assert_eq!(ctrl.ethanol_content(), 0.0);
        
        ctrl.update(10.0);
        assert_eq!(ctrl.ethanol_content(), 100.0);
    }
}

/// Closed loop fuel correction configuration.
#[derive(Clone, Copy, Debug)]
pub struct ClosedLoopConfig {
    /// Enable closed loop fuel correction.
    pub enabled: bool,
    /// Target lambda (1.0 = stoichiometric).
    pub target_lambda: f32,
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain.
    pub ki: f32,
    /// Derivative gain.
    pub kd: f32,
    /// Minimum correction factor (0.8 = -20%).
    pub min_correction: f32,
    /// Maximum correction factor (1.2 = +20%).
    pub max_correction: f32,
    /// Minimum RPM for closed loop.
    pub min_rpm: f32,
    /// Maximum RPM for closed loop.
    pub max_rpm: f32,
    /// Minimum coolant temperature for closed loop (°C).
    pub min_clt: f32,
    /// Lambda error deadband.
    pub lambda_deadband: f32,
    /// Pause closed loop after DFCO (seconds).
    pub dfco_pause_duration_s: f32,
}

impl Default for ClosedLoopConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            target_lambda: 1.0,
            kp: 0.1,
            ki: 0.01,
            kd: 0.0,
            min_correction: 0.8,
            max_correction: 1.2,
            min_rpm: 1500.0,
            max_rpm: 5000.0,
            min_clt: 70.0,
            lambda_deadband: 0.05,
            dfco_pause_duration_s: 5.0,
        }
    }
}

/// Closed loop fuel correction controller using O2/Lambda sensor feedback.
pub struct ClosedLoopController {
    cfg: ClosedLoopConfig,
    /// Current correction factor (1.0 = no correction).
    correction: f32,
    /// Integral term.
    integral: f32,
    /// Previous lambda error.
    prev_error: f32,
    /// First update flag.
    first_update: bool,
    /// Pause timer (seconds remaining).
    pause_timer: f32,
    /// Currently paused.
    paused: bool,
}

impl ClosedLoopController {
    /// Create new closed loop controller with configuration.
    pub fn new(cfg: ClosedLoopConfig) -> Self {
        Self {
            cfg,
            correction: 1.0,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
            pause_timer: 0.0,
            paused: false,
        }
    }

    /// Update closed loop correction.
    ///
    /// # Arguments
    /// * `rpm` — Current engine RPM
    /// * `clt` — Coolant temperature (°C)
    /// * `lambda` — Measured lambda from O2 sensor
    /// * `dt_s` — Time since last update in seconds
    ///
    /// # Returns
    /// Fuel correction factor (1.0 = no correction, <1.0 = lean, >1.0 = rich)
    pub fn update(&mut self, rpm: f32, clt: f32, lambda: f32, dt_s: f32) -> f32 {
        if !self.cfg.enabled {
            self.correction = 1.0;
            return 1.0;
        }

        // Update pause timer
        if self.paused {
            self.pause_timer -= dt_s;
            if self.pause_timer > 0.0 {
                // Still paused: no correction.
                self.correction = 1.0;
                return 1.0;
            }
            // Pause just expired — resume closed-loop control this cycle.
            self.paused = false;
            self.pause_timer = 0.0;
        }

        // Check operating conditions
        if rpm < self.cfg.min_rpm || rpm > self.cfg.max_rpm || clt < self.cfg.min_clt {
            self.correction = 1.0;
            return 1.0;
        }

        // Calculate lambda error
        let error = lambda - self.cfg.target_lambda;

        // Check deadband
        if error.abs() < self.cfg.lambda_deadband {
            // No correction needed, slowly decay integral
            self.integral *= 0.95;
            self.correction = 1.0 + self.integral;
            return self.correction.clamp(self.cfg.min_correction, self.cfg.max_correction);
        }

        // PID calculation
        let p = self.cfg.kp * error;
        self.integral += self.cfg.ki * error * dt_s;
        self.integral = self.integral.clamp(-0.2, 0.2); // Limit integral

        let d = if self.first_update {
            self.first_update = false;
            0.0
        } else {
            self.cfg.kd * (error - self.prev_error) / dt_s
        };

        self.prev_error = error;

        // Calculate correction
        let pid_output = p + self.integral + d;
        self.correction = 1.0 + pid_output;

        self.correction.clamp(self.cfg.min_correction, self.cfg.max_correction)
    }

    /// Current correction factor.
    pub fn correction(&self) -> f32 {
        self.correction
    }

    /// Reset controller to initial state.
    pub fn reset(&mut self) {
        self.correction = 1.0;
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
        self.pause_timer = 0.0;
        self.paused = false;
    }
    
    /// Trigger pause (e.g., after DFCO).
    pub fn trigger_pause(&mut self) {
        self.pause_timer = self.cfg.dfco_pause_duration_s;
        self.paused = true;
    }
    
    /// Check if controller is currently paused.
    pub fn is_paused(&self) -> bool {
        self.paused
    }
}

#[cfg(test)]
mod closed_loop_tests {
    use super::*;

    #[test]
    fn closed_loop_disabled() {
        let cfg = ClosedLoopConfig::default();
        let mut ctrl = ClosedLoopController::new(cfg);
        
        let correction = ctrl.update(2000.0, 80.0, 1.1, 0.01);
        assert_eq!(correction, 1.0);
    }

    #[test]
    fn closed_loop_enabled() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        // Lambda > target (lean, excess air) → closed loop should add fuel.
        let correction = ctrl.update(2000.0, 80.0, 1.1, 0.01);
        assert!(correction > 1.0);
    }

    #[test]
    fn closed_loop_rpm_out_of_range() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        // Below minimum RPM
        let correction = ctrl.update(1000.0, 80.0, 1.1, 0.01);
        assert_eq!(correction, 1.0);
    }

    #[test]
    fn closed_loop_clt_out_of_range() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        // Below minimum coolant temperature
        let correction = ctrl.update(2000.0, 50.0, 1.1, 0.01);
        assert_eq!(correction, 1.0);
    }

    #[test]
    fn closed_loop_deadband() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        cfg.lambda_deadband = 0.1;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        // Within deadband
        let correction = ctrl.update(2000.0, 80.0, 1.05, 0.01);
        assert_eq!(correction, 1.0);
    }

    #[test]
    fn closed_loop_correction_limited() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        cfg.min_correction = 0.8;
        cfg.max_correction = 1.2;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        // Very rich condition
        let correction = ctrl.update(2000.0, 80.0, 2.0, 0.01);
        assert!(correction >= cfg.min_correction);
        assert!(correction <= cfg.max_correction);
    }

    #[test]
    fn closed_loop_reset() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        let _ = ctrl.update(2000.0, 80.0, 1.1, 0.01);
        assert_ne!(ctrl.correction(), 1.0);
        
        ctrl.reset();
        assert_eq!(ctrl.correction(), 1.0);
    }

    #[test]
    fn closed_loop_pause_trigger() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        ctrl.trigger_pause();
        assert!(ctrl.is_paused());
    }

    #[test]
    fn closed_loop_pause_active() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        cfg.dfco_pause_duration_s = 1.0;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        ctrl.trigger_pause();
        let correction = ctrl.update(2000.0, 80.0, 1.1, 0.01);
        assert_eq!(correction, 1.0);
        assert!(ctrl.is_paused());
    }

    #[test]
    fn closed_loop_pause_expire() {
        let mut cfg = ClosedLoopConfig::default();
        cfg.enabled = true;
        cfg.dfco_pause_duration_s = 0.5;
        let mut ctrl = ClosedLoopController::new(cfg);
        
        ctrl.trigger_pause();
        // Update for 0.6 seconds (pause should expire)
        let correction = ctrl.update(2000.0, 80.0, 1.1, 0.6);
        assert!(!ctrl.is_paused());
        // After pause expires, correction is calculated again. Lambda 1.1 is
        // lean, so the closed loop adds fuel (correction > 1.0).
        assert!(correction > 1.0);
    }
}
