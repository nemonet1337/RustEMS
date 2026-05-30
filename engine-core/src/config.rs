//! Engine configuration data structures.
//!
//! All tables use fixed-size arrays for `no_std` / heapless compatibility.
//! Table dimensions match the rusEFI 16×16 standard.

/// Number of RPM bins in the main lookup tables.
pub const RPM_BINS: usize = 16;
/// Number of load bins in the main lookup tables.
pub const LOAD_BINS: usize = 16;
/// Number of dwell table bins.
pub const DWELL_BINS: usize = 8;
/// Number of voltage correction bins.
pub const VOLT_BINS: usize = 8;
/// Number of temperature bins for correction tables.
pub const TEMP_BINS: usize = 8;
/// Maximum number of cylinders supported (Proteus/Huge are 12-cylinder boards).
pub const MAX_CYLINDERS: usize = 12;

/// Complete engine calibration configuration.
///
/// Intended to be stored in flash (read-only at runtime).
/// All fields use SI units unless otherwise stated.
#[derive(Clone, Debug)]
pub struct EngineConfig {
    // ── Engine geometry ─────────────────────────────────────────────────────
    /// Displacement per cylinder in cc.
    pub displacement_cc_per_cyl: f32,
    /// Firing order: `firing_order[step]` = 0-based cylinder index.
    /// Length must equal the cylinder count feature (`cyl-N`).
    pub firing_order: heapless::Vec<u8, MAX_CYLINDERS>,

    // ── Trigger ─────────────────────────────────────────────────────────────
    /// Total teeth on the crank wheel (including missing, e.g. 36 for 36-1).
    pub trigger_total_teeth: u32,
    /// Number of missing teeth (e.g. 1 for 36-1).
    pub trigger_missing_teeth: u32,

    // ── Ignition maps ───────────────────────────────────────────────────────
    /// RPM axis for the ignition advance table.
    pub ignition_rpm_bins: [f32; RPM_BINS],
    /// Load axis for the ignition advance table (0–100 %).
    pub ignition_load_bins: [f32; LOAD_BINS],
    /// Ignition advance table in degrees BTDC `[load_row][rpm_col]`.
    pub ignition_table: [[f32; RPM_BINS]; LOAD_BINS],

    // ── Cranking ────────────────────────────────────────────────────────────
    /// RPM threshold below which cranking timing / dwell apply.
    pub cranking_rpm: f32,
    /// Fixed advance angle during cranking (degrees BTDC).
    pub cranking_timing_deg: f32,
    /// Fixed dwell during cranking (ms).
    pub cranking_dwell_ms: f32,

    // ── Dwell ────────────────────────────────────────────────────────────────
    /// RPM axis for the dwell table.
    pub dwell_rpm_bins: [f32; DWELL_BINS],
    /// Dwell duration table (ms) indexed by RPM.
    pub dwell_ms_table: [f32; DWELL_BINS],
    /// Voltage axis for dwell correction.
    pub dwell_voltage_bins: [f32; VOLT_BINS],
    /// Dwell voltage correction factors (multiplied by base dwell).
    pub dwell_voltage_corr: [f32; VOLT_BINS],

    // ── Fuel (fi-only) ───────────────────────────────────────────────────────
    /// Stoichiometric ratio for the primary fuel (14.7 for gasoline).
    pub stoich_ratio_primary: f32,
    /// Injector flow rate in cc/min.
    pub injector_flow_cc_per_min: f32,
    /// RPM axis for the target lambda table.
    pub lambda_rpm_bins: [f32; RPM_BINS],
    /// Load axis for the target lambda table.
    pub lambda_load_bins: [f32; LOAD_BINS],
    /// Target lambda table `[load_row][rpm_col]`.
    pub lambda_table: [[f32; RPM_BINS]; LOAD_BINS],
    /// Pressure axis for injector deadtime table (kPa).
    pub injector_deadtime_pressure_bins: [f32; VOLT_BINS],
    /// Voltage axis for injector deadtime table (V).
    pub injector_deadtime_voltage_bins: [f32; VOLT_BINS],
    /// Injector deadtime table (ms) `[pressure_row][voltage_col]`.
    pub injector_deadtime_table: [[f32; VOLT_BINS]; VOLT_BINS],

    // ── Speed Density (VE table) ───────────────────────────────────────────
    /// RPM axis for the VE (volumetric efficiency) table.
    pub ve_rpm_bins: [f32; RPM_BINS],
    /// Load axis (MAP in kPa) for the VE table.
    pub ve_load_bins: [f32; LOAD_BINS],
    /// Volumetric efficiency table `[load_row][rpm_col]` (0.0–1.0+).
    /// Values > 1.0 indicate forced induction or tuned engine.
    pub ve_table: [[f32; RPM_BINS]; LOAD_BINS],

    // ── Fuel corrections ────────────────────────────────────────────────────
    /// Intake air temperature axis for fuel density correction (°C).
    pub iat_fuel_temp_bins: [f32; TEMP_BINS],
    /// IAT fuel correction factors (multiplied to fuel mass).
    /// Cold air is denser → more fuel needed (> 1.0).
    /// Hot air is less dense → less fuel needed (< 1.0).
    pub iat_fuel_corr: [f32; TEMP_BINS],
    /// Coolant temperature axis for fuel enrichment (°C).
    pub clt_fuel_temp_bins: [f32; TEMP_BINS],
    /// CLT fuel enrichment multipliers (>= 1.0 when cold).
    pub clt_fuel_corr: [f32; TEMP_BINS],

    // ── Timing corrections ────────────────────────────────────────────────
    /// Coolant temperature axis for CLT timing correction (°C).
    pub clt_corr_temp_bins: [f32; TEMP_BINS],
    /// CLT timing correction in degrees (added to base advance).
    /// Positive = more advance (cold engine), negative = retard (hot).
    pub clt_timing_corr: [f32; TEMP_BINS],
    /// Intake air temperature axis for IAT timing correction (°C).
    pub iat_corr_temp_bins: [f32; TEMP_BINS],
    /// IAT timing correction in degrees (added to base advance).
    /// Positive = more advance (cold air), negative = retard (hot air).
    pub iat_timing_corr: [f32; TEMP_BINS],
}

impl EngineConfig {
    /// Create a default configuration for a 4-cylinder 4-stroke engine with
    /// a 36-1 crank wheel, suitable for initial simulation.
    ///
    /// All maps are flat (constant values).
    pub fn default_4cyl() -> Self {
        Self {
            displacement_cc_per_cyl: 375.0, // 1500 cc / 4
            firing_order: {
                let mut v = heapless::Vec::new();
                // 1-3-4-2 order (0-based: 0, 2, 3, 1)
                let _ = v.push(0u8);
                let _ = v.push(2u8);
                let _ = v.push(3u8);
                let _ = v.push(1u8);
                v
            },

            trigger_total_teeth: 36,
            trigger_missing_teeth: 1,

            ignition_rpm_bins: [
                500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0,
                4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0,
            ],
            ignition_load_bins: [
                10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 50.0,
                60.0, 70.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0,
            ],
            ignition_table: [[10.0; RPM_BINS]; LOAD_BINS],

            cranking_rpm: 400.0,
            cranking_timing_deg: 5.0,
            cranking_dwell_ms: 6.0,

            dwell_rpm_bins: [500.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0],
            dwell_ms_table: [4.0, 4.0, 4.0, 3.5, 3.0, 2.5, 2.0, 2.0],
            dwell_voltage_bins: [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
            dwell_voltage_corr: [2.0, 1.8, 1.5, 1.2, 1.0, 0.9, 0.85, 0.8],

            stoich_ratio_primary: 14.7,
            injector_flow_cc_per_min: 240.0,
            lambda_rpm_bins: [
                500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0,
                4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0,
            ],
            lambda_load_bins: [
                10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 50.0,
                60.0, 70.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0,
            ],
            lambda_table: [[1.0; RPM_BINS]; LOAD_BINS],
            injector_deadtime_pressure_bins: [50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0],
            injector_deadtime_voltage_bins: [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
            injector_deadtime_table: [[0.5; VOLT_BINS]; VOLT_BINS],

            // Speed Density VE table: flat 85% VE for default
            ve_rpm_bins: [
                500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0,
                4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0,
            ],
            ve_load_bins: [
                20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0,
                100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 180.0,
            ],
            ve_table: [[0.85; RPM_BINS]; LOAD_BINS],

            // IAT fuel correction: cold air needs more fuel, hot air less
            iat_fuel_temp_bins: [-20.0, 0.0, 10.0, 20.0, 30.0, 40.0, 60.0, 80.0],
            iat_fuel_corr: [1.15, 1.08, 1.04, 1.0, 0.96, 0.92, 0.85, 0.78],

            // CLT fuel enrichment: more fuel when cold
            clt_fuel_temp_bins: [-40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 120.0],
            clt_fuel_corr: [1.5, 1.3, 1.15, 1.05, 1.0, 1.0, 1.0, 1.0],

            // CLT: more advance when cold (+5° at -40°C), retard when hot (-2° at 120°C)
            clt_corr_temp_bins: [-40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 120.0],
            clt_timing_corr: [5.0, 4.0, 3.0, 1.0, 0.0, -1.0, -1.5, -2.0],
            // IAT: more advance when cold (+2° at -20°C), retard when hot (-3° at 80°C)
            iat_corr_temp_bins: [-20.0, 0.0, 10.0, 20.0, 30.0, 40.0, 60.0, 80.0],
            iat_timing_corr: [2.0, 1.5, 1.0, 0.5, 0.0, -1.0, -2.0, -3.0],
        }
    }

    /// Create a default 1-cylinder configuration.
    pub fn default_1cyl() -> Self {
        let mut cfg = Self::default_4cyl();
        cfg.displacement_cc_per_cyl = 150.0;
        cfg.firing_order = {
            let mut v = heapless::Vec::new();
            let _ = v.push(0u8);
            v
        };
        cfg
    }

    /// Create a default 2-cylinder configuration.
    pub fn default_2cyl() -> Self {
        Self::with_cylinders(250.0, &[0, 1])
    }

    /// Create a default 3-cylinder configuration (firing order 1-3-2).
    pub fn default_3cyl() -> Self {
        Self::with_cylinders(333.0, &[0, 2, 1])
    }

    /// Create a default 5-cylinder configuration (firing order 1-2-4-5-3).
    pub fn default_5cyl() -> Self {
        Self::with_cylinders(500.0, &[0, 1, 3, 4, 2])
    }

    /// Create a default 6-cylinder configuration (inline-6, 1-5-3-6-2-4).
    pub fn default_6cyl() -> Self {
        Self::with_cylinders(500.0, &[0, 4, 2, 5, 1, 3])
    }

    /// Create a default 8-cylinder configuration (V8, 1-8-4-3-6-5-7-2).
    pub fn default_8cyl() -> Self {
        Self::with_cylinders(500.0, &[0, 7, 3, 2, 5, 4, 6, 1])
    }

    /// Create a default 10-cylinder configuration (V10, 1-10-9-4-3-6-5-8-7-2).
    pub fn default_10cyl() -> Self {
        Self::with_cylinders(500.0, &[0, 9, 8, 3, 2, 5, 4, 7, 6, 1])
    }

    /// Create a default 12-cylinder configuration (V12, 1-7-5-11-3-9-6-12-2-8-4-10).
    pub fn default_12cyl() -> Self {
        Self::with_cylinders(500.0, &[0, 6, 4, 10, 2, 8, 5, 11, 1, 7, 3, 9])
    }

    /// Build a configuration from the 4-cylinder base, overriding the
    /// per-cylinder displacement and firing order. The firing order must be a
    /// permutation of `0..N` with `N <= MAX_CYLINDERS`.
    fn with_cylinders(displacement_cc_per_cyl: f32, firing_order: &[u8]) -> Self {
        let mut cfg = Self::default_4cyl();
        cfg.displacement_cc_per_cyl = displacement_cc_per_cyl;
        let mut v = heapless::Vec::new();
        for &cyl in firing_order.iter().take(MAX_CYLINDERS) {
            let _ = v.push(cyl);
        }
        cfg.firing_order = v;
        cfg
    }
}
