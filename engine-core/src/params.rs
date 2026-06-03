//! RDP parameter catalog — type-safe access to [`crate::config::EngineConfig`] fields.
//!
//! Provides:
//! - [`ParamId`]          — numeric identifier for each tunable parameter.
//! - [`ParamType`]        — value type (scalar f32, table cell, axis value).
//! - [`ParamDescriptor`]  — human-readable metadata for a parameter.
//! - [`get_param`]        — read a parameter value from a config snapshot.
//! - [`set_param`]        — write a parameter value into a mutable config.
//!
//! No raw pointer arithmetic or byte offsets are used; each `ParamId` arm
//! maps directly to the corresponding struct field.

use crate::config::{EngineConfig, DWELL_BINS, LOAD_BINS, RPM_BINS, TEMP_BINS, VOLT_BINS};

// ─── Parameter value types ────────────────────────────────────────────────────

/// The wire type of a parameter value in RDP messages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamType {
    /// 32-bit IEEE float (all scalar engine parameters).
    F32,
}

// ─── Parameter identifier ─────────────────────────────────────────────────────

/// Numeric identifier for every tunable parameter in [`EngineConfig`].
///
/// IDs are grouped into ranges by subsystem.  The numeric values are stable
/// across firmware versions; never renumber or reuse a retired ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum ParamId {
    // ── Engine geometry (0x0100) ───────────────────────────────────────────
    /// Displacement per cylinder (cc).
    DisplacementCcPerCyl = 0x0100,

    // ── Trigger (0x0110) ──────────────────────────────────────────────────
    /// Total teeth on crank wheel.
    TriggerTotalTeeth = 0x0110,
    /// Missing teeth on crank wheel.
    TriggerMissingTeeth = 0x0111,

    // ── Cranking (0x0120) ─────────────────────────────────────────────────
    /// Cranking RPM threshold.
    CrankingRpm = 0x0120,
    /// Cranking ignition advance (° BTDC).
    CrankingTimingDeg = 0x0121,
    /// Cranking dwell (ms).
    CrankingDwellMs = 0x0122,

    // ── Fuel globals (0x0130) ─────────────────────────────────────────────
    /// Stoichiometric AFR for primary fuel.
    StoichRatioPrimary = 0x0130,
    /// Injector flow rate (cc/min).
    InjectorFlowCcPerMin = 0x0131,

    /// Base ID for the 16×16 ignition advance table cells (`[load][rpm]`).
    IgnitionTableBase = 0x0200,

    /// Base ID for the 16×16 target lambda table cells (`[load][rpm]`).
    LambdaTableBase = 0x0300,

    /// Base ID for the 16×16 volumetric-efficiency (VE) table cells (`[load][rpm]`).
    VeTableBase = 0x0400,

    /// Base ID for the dwell RPM axis array (8 elements).
    DwellRpmBinBase       = 0x0500,
    /// Base ID for the dwell duration table (8 elements, ms).
    DwellMsTableBase      = 0x0508,
    /// Base ID for the dwell voltage-correction axis (8 elements, V).
    DwellVoltageBinBase   = 0x0510,
    /// Base ID for the dwell voltage-correction factor array (8 elements).
    DwellVoltageCorr      = 0x0518,

    /// Base ID for the IAT fuel-correction temperature axis (8 elements, °C).
    IatFuelTempBinBase = 0x0600,
    /// Base ID for the IAT fuel-correction factor array (8 elements).
    IatFuelCorrBase    = 0x0608,

    /// Base ID for the CLT fuel-correction temperature axis (8 elements, °C).
    CltFuelTempBinBase = 0x0610,
    /// Base ID for the CLT fuel-correction factor array (8 elements).
    CltFuelCorrBase    = 0x0618,

    /// Base ID for the CLT timing-correction temperature axis (8 elements, °C).
    CltCorrTempBinBase = 0x0620,
    /// Base ID for the CLT timing-correction array (8 elements, °).
    CltTimingCorrBase  = 0x0628,

    /// Base ID for the IAT timing-correction temperature axis (8 elements, °C).
    IatCorrTempBinBase = 0x0630,
    /// Base ID for the IAT timing-correction array (8 elements, °).
    IatTimingCorrBase  = 0x0638,
}

impl ParamId {
    /// Convert a raw u16 to a ParamId, returning None for unknown values.
    pub fn from_u16(v: u16) -> Option<Self> {
        match v {
            0x0100 => Some(Self::DisplacementCcPerCyl),
            0x0110 => Some(Self::TriggerTotalTeeth),
            0x0111 => Some(Self::TriggerMissingTeeth),
            0x0120 => Some(Self::CrankingRpm),
            0x0121 => Some(Self::CrankingTimingDeg),
            0x0122 => Some(Self::CrankingDwellMs),
            0x0130 => Some(Self::StoichRatioPrimary),
            0x0131 => Some(Self::InjectorFlowCcPerMin),
            0x0200 => Some(Self::IgnitionTableBase),
            0x0300 => Some(Self::LambdaTableBase),
            0x0400 => Some(Self::VeTableBase),
            0x0500 => Some(Self::DwellRpmBinBase),
            0x0508 => Some(Self::DwellMsTableBase),
            0x0510 => Some(Self::DwellVoltageBinBase),
            0x0518 => Some(Self::DwellVoltageCorr),
            0x0600 => Some(Self::IatFuelTempBinBase),
            0x0608 => Some(Self::IatFuelCorrBase),
            0x0610 => Some(Self::CltFuelTempBinBase),
            0x0618 => Some(Self::CltFuelCorrBase),
            0x0620 => Some(Self::CltCorrTempBinBase),
            0x0628 => Some(Self::CltTimingCorrBase),
            0x0630 => Some(Self::IatCorrTempBinBase),
            0x0638 => Some(Self::IatTimingCorrBase),
            _ => None,
        }
    }

    /// Wire ID as u16.
    pub const fn as_u16(self) -> u16 {
        self as u16
    }
}

// ─── Parameter descriptor ─────────────────────────────────────────────────────

/// Human-readable metadata for one parameter.
#[derive(Clone, Copy, Debug)]
pub struct ParamDescriptor {
    /// Wire ID.
    pub id: u16,
    /// Short ASCII name (null-padded).
    pub name: &'static str,
    /// Physical unit string.
    pub unit: &'static str,
    /// Minimum allowed value.
    pub min: f32,
    /// Maximum allowed value.
    pub max: f32,
    /// Value type on the wire.
    pub param_type: ParamType,
}

/// Return the descriptor for a scalar param, or `None` if unknown / table-based.
pub fn param_descriptor(id: ParamId) -> Option<ParamDescriptor> {
    use ParamId::*;
    use ParamType::F32;
    match id {
        DisplacementCcPerCyl => Some(ParamDescriptor {
            id: id.as_u16(), name: "disp_cc_per_cyl", unit: "cc",
            min: 1.0, max: 5000.0, param_type: F32,
        }),
        TriggerTotalTeeth => Some(ParamDescriptor {
            id: id.as_u16(), name: "trigger_teeth_total", unit: "count",
            min: 4.0, max: 256.0, param_type: F32,
        }),
        TriggerMissingTeeth => Some(ParamDescriptor {
            id: id.as_u16(), name: "trigger_teeth_missing", unit: "count",
            min: 1.0, max: 4.0, param_type: F32,
        }),
        CrankingRpm => Some(ParamDescriptor {
            id: id.as_u16(), name: "cranking_rpm", unit: "RPM",
            min: 50.0, max: 1000.0, param_type: F32,
        }),
        CrankingTimingDeg => Some(ParamDescriptor {
            id: id.as_u16(), name: "cranking_timing_deg", unit: "°BTDC",
            min: -10.0, max: 30.0, param_type: F32,
        }),
        CrankingDwellMs => Some(ParamDescriptor {
            id: id.as_u16(), name: "cranking_dwell_ms", unit: "ms",
            min: 1.0, max: 15.0, param_type: F32,
        }),
        StoichRatioPrimary => Some(ParamDescriptor {
            id: id.as_u16(), name: "stoich_ratio", unit: "AFR",
            min: 10.0, max: 20.0, param_type: F32,
        }),
        InjectorFlowCcPerMin => Some(ParamDescriptor {
            id: id.as_u16(), name: "inj_flow_cc_min", unit: "cc/min",
            min: 50.0, max: 2000.0, param_type: F32,
        }),
        // Table-base params have no simple scalar descriptor
        IgnitionTableBase | LambdaTableBase | VeTableBase => None,
        DwellRpmBinBase | DwellMsTableBase | DwellVoltageBinBase | DwellVoltageCorr => None,
        IatFuelTempBinBase | IatFuelCorrBase => None,
        CltFuelTempBinBase | CltFuelCorrBase => None,
        CltCorrTempBinBase | CltTimingCorrBase => None,
        IatCorrTempBinBase | IatTimingCorrBase => None,
    }
}

// ─── Scalar get / set ─────────────────────────────────────────────────────────

/// Read a scalar parameter from the engine config.
///
/// For table-based params, use the cell-level accessors below.
/// Returns `None` for unknown or table-base IDs.
pub fn get_param(cfg: &EngineConfig, id: ParamId) -> Option<f32> {
    use ParamId::*;
    match id {
        DisplacementCcPerCyl  => Some(cfg.displacement_cc_per_cyl),
        TriggerTotalTeeth     => Some(cfg.trigger_total_teeth as f32),
        TriggerMissingTeeth   => Some(cfg.trigger_missing_teeth as f32),
        CrankingRpm           => Some(cfg.cranking_rpm),
        CrankingTimingDeg     => Some(cfg.cranking_timing_deg),
        CrankingDwellMs       => Some(cfg.cranking_dwell_ms),
        StoichRatioPrimary    => Some(cfg.stoich_ratio_primary),
        InjectorFlowCcPerMin  => Some(cfg.injector_flow_cc_per_min),
        // Table bases and array params are not readable as single scalars
        IgnitionTableBase | LambdaTableBase | VeTableBase => None,
        DwellRpmBinBase | DwellMsTableBase | DwellVoltageBinBase | DwellVoltageCorr => None,
        IatFuelTempBinBase | IatFuelCorrBase => None,
        CltFuelTempBinBase | CltFuelCorrBase => None,
        CltCorrTempBinBase | CltTimingCorrBase => None,
        IatCorrTempBinBase | IatTimingCorrBase => None,
    }
}

/// Write a scalar parameter into the engine config.
///
/// Returns `true` on success, `false` if the param ID is unknown or read-only.
/// Values are clamped to the descriptor's min/max when applicable.
pub fn set_param(cfg: &mut EngineConfig, id: ParamId, value: f32) -> bool {
    use ParamId::*;
    match id {
        DisplacementCcPerCyl  => { cfg.displacement_cc_per_cyl  = value.max(1.0); true }
        TriggerTotalTeeth     => { cfg.trigger_total_teeth       = (value as u32).max(1); true }
        TriggerMissingTeeth   => { cfg.trigger_missing_teeth     = (value as u32).max(1); true }
        CrankingRpm           => { cfg.cranking_rpm              = value.clamp(50.0, 1000.0); true }
        CrankingTimingDeg     => { cfg.cranking_timing_deg       = value.clamp(-10.0, 30.0); true }
        CrankingDwellMs       => { cfg.cranking_dwell_ms         = value.clamp(1.0, 15.0); true }
        StoichRatioPrimary    => { cfg.stoich_ratio_primary      = value.clamp(10.0, 20.0); true }
        InjectorFlowCcPerMin  => { cfg.injector_flow_cc_per_min  = value.clamp(50.0, 2000.0); true }
        _ => false,
    }
}

// ─── Table / array cell accessors ────────────────────────────────────────────

/// Read one cell from a 2D table by base param ID, load-row, and RPM-column.
///
/// Returns `None` if the indices are out of bounds or the ID is not a table.
pub fn get_table_cell(cfg: &EngineConfig, base: ParamId, row: usize, col: usize) -> Option<f32> {
    use ParamId::*;
    match base {
        IgnitionTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                Some(cfg.ignition_table[row][col])
            } else {
                None
            }
        }
        LambdaTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                Some(cfg.lambda_table[row][col])
            } else {
                None
            }
        }
        VeTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                Some(cfg.ve_table[row][col])
            } else {
                None
            }
        }
        _ => None,
    }
}

/// Write one cell into a 2D table.
pub fn set_table_cell(cfg: &mut EngineConfig, base: ParamId, row: usize, col: usize, value: f32) -> bool {
    use ParamId::*;
    match base {
        IgnitionTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                cfg.ignition_table[row][col] = value;
                true
            } else {
                false
            }
        }
        LambdaTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                cfg.lambda_table[row][col] = value.clamp(0.5, 2.0);
                true
            } else {
                false
            }
        }
        VeTableBase => {
            if row < LOAD_BINS && col < RPM_BINS {
                cfg.ve_table[row][col] = value.clamp(0.1, 1.5);
                true
            } else {
                false
            }
        }
        _ => false,
    }
}

/// Read one element from a 1D array parameter.
pub fn get_array_element(cfg: &EngineConfig, base: ParamId, idx: usize) -> Option<f32> {
    use ParamId::*;
    match base {
        DwellRpmBinBase     => cfg.dwell_rpm_bins.get(idx).copied(),
        DwellMsTableBase    => cfg.dwell_ms_table.get(idx).copied(),
        DwellVoltageBinBase => cfg.dwell_voltage_bins.get(idx).copied(),
        DwellVoltageCorr    => cfg.dwell_voltage_corr.get(idx).copied(),
        IatFuelTempBinBase  => cfg.iat_fuel_temp_bins.get(idx).copied(),
        IatFuelCorrBase     => cfg.iat_fuel_corr.get(idx).copied(),
        CltFuelTempBinBase  => cfg.clt_fuel_temp_bins.get(idx).copied(),
        CltFuelCorrBase     => cfg.clt_fuel_corr.get(idx).copied(),
        CltCorrTempBinBase  => cfg.clt_corr_temp_bins.get(idx).copied(),
        CltTimingCorrBase   => cfg.clt_timing_corr.get(idx).copied(),
        IatCorrTempBinBase  => cfg.iat_corr_temp_bins.get(idx).copied(),
        IatTimingCorrBase   => cfg.iat_timing_corr.get(idx).copied(),
        _ => None,
    }
}

/// Write one element into a 1D array parameter.
pub fn set_array_element(cfg: &mut EngineConfig, base: ParamId, idx: usize, value: f32) -> bool {
    use ParamId::*;
    match base {
        DwellRpmBinBase     => { if idx < DWELL_BINS { cfg.dwell_rpm_bins[idx]      = value; true } else { false } }
        DwellMsTableBase    => { if idx < DWELL_BINS { cfg.dwell_ms_table[idx]      = value.clamp(0.5, 15.0); true } else { false } }
        DwellVoltageBinBase => { if idx < VOLT_BINS  { cfg.dwell_voltage_bins[idx]   = value; true } else { false } }
        DwellVoltageCorr    => { if idx < VOLT_BINS  { cfg.dwell_voltage_corr[idx]   = value.clamp(0.1, 5.0); true } else { false } }
        IatFuelTempBinBase  => { if idx < TEMP_BINS  { cfg.iat_fuel_temp_bins[idx]   = value; true } else { false } }
        IatFuelCorrBase     => { if idx < TEMP_BINS  { cfg.iat_fuel_corr[idx]          = value.clamp(0.5, 2.0); true } else { false } }
        CltFuelTempBinBase  => { if idx < TEMP_BINS  { cfg.clt_fuel_temp_bins[idx]   = value; true } else { false } }
        CltFuelCorrBase     => { if idx < TEMP_BINS  { cfg.clt_fuel_corr[idx]         = value.clamp(0.5, 2.0); true } else { false } }
        CltCorrTempBinBase  => { if idx < TEMP_BINS  { cfg.clt_corr_temp_bins[idx]   = value; true } else { false } }
        CltTimingCorrBase   => { if idx < TEMP_BINS  { cfg.clt_timing_corr[idx]       = value.clamp(-20.0, 20.0); true } else { false } }
        IatCorrTempBinBase  => { if idx < TEMP_BINS  { cfg.iat_corr_temp_bins[idx]   = value; true } else { false } }
        IatTimingCorrBase   => { if idx < TEMP_BINS  { cfg.iat_timing_corr[idx]       = value.clamp(-20.0, 20.0); true } else { false } }
        _ => false,
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_scalar() {
        let mut cfg = EngineConfig::default_4cyl();
        assert!(set_param(&mut cfg, ParamId::CrankingRpm, 350.0));
        assert_eq!(get_param(&cfg, ParamId::CrankingRpm), Some(350.0));
    }

    #[test]
    fn round_trip_table_cell() {
        let mut cfg = EngineConfig::default_4cyl();
        assert!(set_table_cell(&mut cfg, ParamId::IgnitionTableBase, 2, 4, 25.0));
        assert_eq!(get_table_cell(&cfg, ParamId::IgnitionTableBase, 2, 4), Some(25.0));
    }

    #[test]
    fn round_trip_array_element() {
        let mut cfg = EngineConfig::default_4cyl();
        assert!(set_array_element(&mut cfg, ParamId::DwellMsTableBase, 0, 5.0));
        assert_eq!(get_array_element(&cfg, ParamId::DwellMsTableBase, 0), Some(5.0));
    }

    #[test]
    fn out_of_bounds_returns_false() {
        let mut cfg = EngineConfig::default_4cyl();
        assert!(!set_table_cell(&mut cfg, ParamId::IgnitionTableBase, 99, 0, 10.0));
        assert!(get_table_cell(&cfg, ParamId::IgnitionTableBase, 99, 0).is_none());
    }

    #[test]
    fn from_u16_roundtrip() {
        let id = ParamId::CrankingRpm;
        assert_eq!(ParamId::from_u16(id.as_u16()), Some(id));
    }

    #[test]
    fn unknown_id_returns_none() {
        assert!(get_param(&EngineConfig::default_4cyl(), ParamId::IgnitionTableBase).is_none());
    }

    #[test]
    fn descriptor_has_sane_bounds() {
        let d = param_descriptor(ParamId::CrankingRpm).unwrap();
        assert!(d.min < d.max);
    }
}
