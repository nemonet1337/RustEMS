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

// ─── RDP catalog (self-describing schema) ────────────────────────────────────
//
// Static descriptor data served by the Descriptor.* opcodes
// (see docs/api/04-parameter-model.md). Descriptors are flash-resident
// (`&'static`) and consume no RAM.

/// UI category names, indexed by the `category` field of the descriptors.
pub const CATEGORIES: &[&str] = &[
    "Engine", "Trigger", "Ignition", "Fuel", "Enrichment",
    "Idle", "Boost", "Vvt", "Sensors", "Protection",
];

/// Descriptor flag: parameter is read-only.
pub const PFLAG_READ_ONLY: u8 = 0x01;
/// Descriptor flag: change requires a reboot.
pub const PFLAG_NEEDS_REBOOT: u8 = 0x02;
/// Descriptor flag: writable only while the engine is stopped.
pub const PFLAG_ENGINE_STOPPED_ONLY: u8 = 0x04;

/// Full static metadata for one scalar parameter (RDP `ParamDesc`).
pub struct ParamMeta {
    /// Stable wire ID.
    pub id: ParamId,
    /// Machine-readable key, e.g. `"fuel.injector_flow_cc_min"`.
    pub key: &'static str,
    /// Human-readable label.
    pub label: &'static str,
    /// Category index into [`CATEGORIES`].
    pub category: u8,
    /// Physical unit string.
    pub unit: &'static str,
    /// Minimum allowed value (physical units).
    pub min: f32,
    /// Maximum allowed value (physical units).
    pub max: f32,
    /// Default value (physical units).
    pub default: f32,
    /// Display decimal digits.
    pub digits: u8,
    /// `PFLAG_*` bits.
    pub flags: u8,
}

/// Static scalar parameter catalog served by `Descriptor.GetParamCatalog`.
pub const PARAM_CATALOG: &[ParamMeta] = &[
    ParamMeta { id: ParamId::DisplacementCcPerCyl, key: "engine.disp_cc_per_cyl",
        label: "Displacement / cylinder", category: 0, unit: "cc",
        min: 1.0, max: 5000.0, default: 375.0, digits: 0, flags: PFLAG_ENGINE_STOPPED_ONLY },
    ParamMeta { id: ParamId::TriggerTotalTeeth, key: "trigger.total_teeth",
        label: "Trigger total teeth", category: 1, unit: "count",
        min: 4.0, max: 256.0, default: 36.0, digits: 0, flags: PFLAG_ENGINE_STOPPED_ONLY },
    ParamMeta { id: ParamId::TriggerMissingTeeth, key: "trigger.missing_teeth",
        label: "Trigger missing teeth", category: 1, unit: "count",
        min: 1.0, max: 4.0, default: 1.0, digits: 0, flags: PFLAG_ENGINE_STOPPED_ONLY },
    ParamMeta { id: ParamId::CrankingRpm, key: "ignition.cranking_rpm",
        label: "Cranking RPM threshold", category: 2, unit: "RPM",
        min: 50.0, max: 1000.0, default: 400.0, digits: 0, flags: 0 },
    ParamMeta { id: ParamId::CrankingTimingDeg, key: "ignition.cranking_timing",
        label: "Cranking advance", category: 2, unit: "°BTDC",
        min: -10.0, max: 30.0, default: 5.0, digits: 1, flags: 0 },
    ParamMeta { id: ParamId::CrankingDwellMs, key: "ignition.cranking_dwell",
        label: "Cranking dwell", category: 2, unit: "ms",
        min: 1.0, max: 15.0, default: 6.0, digits: 1, flags: 0 },
    ParamMeta { id: ParamId::StoichRatioPrimary, key: "fuel.stoich_ratio",
        label: "Stoichiometric AFR", category: 3, unit: "AFR",
        min: 10.0, max: 20.0, default: 14.7, digits: 1, flags: 0 },
    ParamMeta { id: ParamId::InjectorFlowCcPerMin, key: "fuel.injector_flow",
        label: "Injector flow rate", category: 3, unit: "cc/min",
        min: 50.0, max: 2000.0, default: 240.0, digits: 0, flags: 0 },
];

// ─── Table catalog ───────────────────────────────────────────────────────────

/// Stable wire IDs for tables / 1D curves (separate ID space from `ParamId`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum TableId {
    /// 16×16 ignition advance table (x=RPM, y=load).
    Ignition = 1,
    /// 16×16 target lambda table.
    Lambda = 2,
    /// 16×16 volumetric efficiency table.
    Ve = 3,
    /// 8×8 injector deadtime table (x=voltage, y=pressure).
    InjectorDeadtime = 4,
    /// 1D dwell duration curve vs RPM (8 points).
    DwellMs = 10,
    /// 1D dwell voltage-correction curve (8 points).
    DwellVoltageCorr = 11,
    /// 1D IAT fuel-correction curve (8 points).
    IatFuelCorr = 12,
    /// 1D CLT fuel-enrichment curve (8 points).
    CltFuelCorr = 13,
    /// 1D CLT timing-correction curve (8 points).
    CltTimingCorr = 14,
    /// 1D IAT timing-correction curve (8 points).
    IatTimingCorr = 15,
}

impl TableId {
    /// Parse a raw wire ID.
    pub fn from_u16(v: u16) -> Option<Self> {
        match v {
            1 => Some(Self::Ignition),
            2 => Some(Self::Lambda),
            3 => Some(Self::Ve),
            4 => Some(Self::InjectorDeadtime),
            10 => Some(Self::DwellMs),
            11 => Some(Self::DwellVoltageCorr),
            12 => Some(Self::IatFuelCorr),
            13 => Some(Self::CltFuelCorr),
            14 => Some(Self::CltTimingCorr),
            15 => Some(Self::IatTimingCorr),
            _ => None,
        }
    }

    /// Wire ID as u16.
    pub const fn as_u16(self) -> u16 {
        self as u16
    }
}

/// Full static metadata for one table (RDP `TableDesc`).
pub struct TableMeta {
    /// Stable wire ID.
    pub id: TableId,
    /// Machine-readable key, e.g. `"fuel.ve_table"`.
    pub key: &'static str,
    /// Human-readable label.
    pub label: &'static str,
    /// Category index into [`CATEGORIES`].
    pub category: u8,
    /// 1 or 2 dimensions.
    pub dims: u8,
    /// Number of x (column) entries.
    pub x_size: u16,
    /// Number of y (row) entries (0 for 1D curves).
    pub y_size: u16,
    /// X axis key.
    pub x_axis_key: &'static str,
    /// Y axis key (empty for 1D curves).
    pub y_axis_key: &'static str,
    /// X axis unit.
    pub x_unit: &'static str,
    /// Y axis unit.
    pub y_unit: &'static str,
    /// Cell unit.
    pub cell_unit: &'static str,
    /// Minimum allowed cell value.
    pub cell_min: f32,
    /// Maximum allowed cell value.
    pub cell_max: f32,
    /// Display decimal digits for cells.
    pub cell_digits: u8,
}

/// Static table catalog served by `Descriptor.GetTableCatalog`.
pub const TABLE_CATALOG: &[TableMeta] = &[
    TableMeta { id: TableId::Ignition, key: "ignition.advance_table", label: "Ignition advance",
        category: 2, dims: 2, x_size: RPM_BINS as u16, y_size: LOAD_BINS as u16,
        x_axis_key: "rpm", y_axis_key: "load", x_unit: "RPM", y_unit: "%", cell_unit: "°BTDC",
        cell_min: -10.0, cell_max: 60.0, cell_digits: 1 },
    TableMeta { id: TableId::Lambda, key: "fuel.lambda_table", label: "Target lambda",
        category: 3, dims: 2, x_size: RPM_BINS as u16, y_size: LOAD_BINS as u16,
        x_axis_key: "rpm", y_axis_key: "load", x_unit: "RPM", y_unit: "%", cell_unit: "λ",
        cell_min: 0.5, cell_max: 2.0, cell_digits: 3 },
    TableMeta { id: TableId::Ve, key: "fuel.ve_table", label: "Volumetric efficiency",
        category: 3, dims: 2, x_size: RPM_BINS as u16, y_size: LOAD_BINS as u16,
        x_axis_key: "rpm", y_axis_key: "map", x_unit: "RPM", y_unit: "kPa", cell_unit: "ratio",
        cell_min: 0.1, cell_max: 1.5, cell_digits: 2 },
    TableMeta { id: TableId::InjectorDeadtime, key: "fuel.deadtime_table", label: "Injector deadtime",
        category: 3, dims: 2, x_size: VOLT_BINS as u16, y_size: VOLT_BINS as u16,
        x_axis_key: "vbatt", y_axis_key: "fuel_press", x_unit: "V", y_unit: "kPa", cell_unit: "ms",
        cell_min: 0.0, cell_max: 5.0, cell_digits: 2 },
    TableMeta { id: TableId::DwellMs, key: "ignition.dwell_curve", label: "Dwell duration",
        category: 2, dims: 1, x_size: DWELL_BINS as u16, y_size: 0,
        x_axis_key: "rpm", y_axis_key: "", x_unit: "RPM", y_unit: "", cell_unit: "ms",
        cell_min: 0.5, cell_max: 15.0, cell_digits: 1 },
    TableMeta { id: TableId::DwellVoltageCorr, key: "ignition.dwell_volt_corr", label: "Dwell voltage correction",
        category: 2, dims: 1, x_size: VOLT_BINS as u16, y_size: 0,
        x_axis_key: "vbatt", y_axis_key: "", x_unit: "V", y_unit: "", cell_unit: "ratio",
        cell_min: 0.1, cell_max: 5.0, cell_digits: 2 },
    TableMeta { id: TableId::IatFuelCorr, key: "enrich.iat_fuel_corr", label: "IAT fuel correction",
        category: 4, dims: 1, x_size: TEMP_BINS as u16, y_size: 0,
        x_axis_key: "iat", y_axis_key: "", x_unit: "°C", y_unit: "", cell_unit: "ratio",
        cell_min: 0.5, cell_max: 2.0, cell_digits: 2 },
    TableMeta { id: TableId::CltFuelCorr, key: "enrich.clt_fuel_corr", label: "CLT fuel enrichment",
        category: 4, dims: 1, x_size: TEMP_BINS as u16, y_size: 0,
        x_axis_key: "clt", y_axis_key: "", x_unit: "°C", y_unit: "", cell_unit: "ratio",
        cell_min: 0.5, cell_max: 2.0, cell_digits: 2 },
    TableMeta { id: TableId::CltTimingCorr, key: "enrich.clt_timing_corr", label: "CLT timing correction",
        category: 4, dims: 1, x_size: TEMP_BINS as u16, y_size: 0,
        x_axis_key: "clt", y_axis_key: "", x_unit: "°C", y_unit: "", cell_unit: "deg",
        cell_min: -20.0, cell_max: 20.0, cell_digits: 1 },
    TableMeta { id: TableId::IatTimingCorr, key: "enrich.iat_timing_corr", label: "IAT timing correction",
        category: 4, dims: 1, x_size: TEMP_BINS as u16, y_size: 0,
        x_axis_key: "iat", y_axis_key: "", x_unit: "°C", y_unit: "", cell_unit: "deg",
        cell_min: -20.0, cell_max: 20.0, cell_digits: 1 },
];

/// Find the catalog entry for a table.
pub fn table_meta(id: TableId) -> Option<&'static TableMeta> {
    TABLE_CATALOG.iter().find(|m| m.id == id)
}

/// Find the catalog entry for a scalar parameter.
pub fn param_meta(id: ParamId) -> Option<&'static ParamMeta> {
    PARAM_CATALOG.iter().find(|m| m.id == id)
}

// ─── Whole-table access (RDP Config.TableGet / TableSet*) ───────────────────

/// Snapshot of one table's axes and cells (row-major `[y][x]`).
pub struct TableData {
    /// X axis values.
    pub x_axis: heapless::Vec<f32, 16>,
    /// Y axis values (empty for 1D curves).
    pub y_axis: heapless::Vec<f32, 16>,
    /// Cell values, row-major.
    pub cells: heapless::Vec<f32, 256>,
}

fn push_axis(dst: &mut heapless::Vec<f32, 16>, src: &[f32]) {
    for &v in src {
        let _ = dst.push(v);
    }
}

fn push_cells_2d<const C: usize, const R: usize>(
    dst: &mut heapless::Vec<f32, 256>,
    table: &[[f32; C]; R],
) {
    for row in table.iter() {
        for &v in row.iter() {
            let _ = dst.push(v);
        }
    }
}

/// Read a full table (axes + cells) by wire ID.
pub fn table_get(cfg: &EngineConfig, id: TableId) -> TableData {
    let mut data = TableData {
        x_axis: heapless::Vec::new(),
        y_axis: heapless::Vec::new(),
        cells: heapless::Vec::new(),
    };
    match id {
        TableId::Ignition => {
            push_axis(&mut data.x_axis, &cfg.ignition_rpm_bins);
            push_axis(&mut data.y_axis, &cfg.ignition_load_bins);
            push_cells_2d(&mut data.cells, &cfg.ignition_table);
        }
        TableId::Lambda => {
            push_axis(&mut data.x_axis, &cfg.lambda_rpm_bins);
            push_axis(&mut data.y_axis, &cfg.lambda_load_bins);
            push_cells_2d(&mut data.cells, &cfg.lambda_table);
        }
        TableId::Ve => {
            push_axis(&mut data.x_axis, &cfg.ve_rpm_bins);
            push_axis(&mut data.y_axis, &cfg.ve_load_bins);
            push_cells_2d(&mut data.cells, &cfg.ve_table);
        }
        TableId::InjectorDeadtime => {
            push_axis(&mut data.x_axis, &cfg.injector_deadtime_voltage_bins);
            push_axis(&mut data.y_axis, &cfg.injector_deadtime_pressure_bins);
            push_cells_2d(&mut data.cells, &cfg.injector_deadtime_table);
        }
        TableId::DwellMs => {
            push_axis(&mut data.x_axis, &cfg.dwell_rpm_bins);
            for &v in cfg.dwell_ms_table.iter() { let _ = data.cells.push(v); }
        }
        TableId::DwellVoltageCorr => {
            push_axis(&mut data.x_axis, &cfg.dwell_voltage_bins);
            for &v in cfg.dwell_voltage_corr.iter() { let _ = data.cells.push(v); }
        }
        TableId::IatFuelCorr => {
            push_axis(&mut data.x_axis, &cfg.iat_fuel_temp_bins);
            for &v in cfg.iat_fuel_corr.iter() { let _ = data.cells.push(v); }
        }
        TableId::CltFuelCorr => {
            push_axis(&mut data.x_axis, &cfg.clt_fuel_temp_bins);
            for &v in cfg.clt_fuel_corr.iter() { let _ = data.cells.push(v); }
        }
        TableId::CltTimingCorr => {
            push_axis(&mut data.x_axis, &cfg.clt_corr_temp_bins);
            for &v in cfg.clt_timing_corr.iter() { let _ = data.cells.push(v); }
        }
        TableId::IatTimingCorr => {
            push_axis(&mut data.x_axis, &cfg.iat_corr_temp_bins);
            for &v in cfg.iat_timing_corr.iter() { let _ = data.cells.push(v); }
        }
    }
    data
}

/// Outcome of an RDP table write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableWriteError {
    /// `ix` / `iy` outside the table dimensions.
    BadIndex,
    /// Value outside the descriptor's `cell_min..=cell_max` range.
    OutOfRange,
}

/// Write one table cell by wire ID and `(ix, iy)` position (`ix` = column / x,
/// `iy` = row / y; `iy` is ignored for 1D curves).
pub fn table_write_cell(
    cfg: &mut EngineConfig,
    id: TableId,
    ix: usize,
    iy: usize,
    value: f32,
) -> Result<(), TableWriteError> {
    let Some(meta) = table_meta(id) else { return Err(TableWriteError::BadIndex) };
    if !(meta.cell_min..=meta.cell_max).contains(&value) {
        return Err(TableWriteError::OutOfRange);
    }
    let ok = match id {
        TableId::Ignition => bounded_set_2d(&mut cfg.ignition_table, iy, ix, value),
        TableId::Lambda => bounded_set_2d(&mut cfg.lambda_table, iy, ix, value),
        TableId::Ve => bounded_set_2d(&mut cfg.ve_table, iy, ix, value),
        TableId::InjectorDeadtime => bounded_set_2d(&mut cfg.injector_deadtime_table, iy, ix, value),
        TableId::DwellMs => bounded_set_1d(&mut cfg.dwell_ms_table, ix, value),
        TableId::DwellVoltageCorr => bounded_set_1d(&mut cfg.dwell_voltage_corr, ix, value),
        TableId::IatFuelCorr => bounded_set_1d(&mut cfg.iat_fuel_corr, ix, value),
        TableId::CltFuelCorr => bounded_set_1d(&mut cfg.clt_fuel_corr, ix, value),
        TableId::CltTimingCorr => bounded_set_1d(&mut cfg.clt_timing_corr, ix, value),
        TableId::IatTimingCorr => bounded_set_1d(&mut cfg.iat_timing_corr, ix, value),
    };
    if ok { Ok(()) } else { Err(TableWriteError::BadIndex) }
}

fn bounded_set_2d<const C: usize, const R: usize>(
    table: &mut [[f32; C]; R],
    row: usize,
    col: usize,
    value: f32,
) -> bool {
    if row < R && col < C {
        table[row][col] = value;
        true
    } else {
        false
    }
}

fn bounded_set_1d<const N: usize>(arr: &mut [f32; N], idx: usize, value: f32) -> bool {
    if idx < N {
        arr[idx] = value;
        true
    } else {
        false
    }
}

/// Axis selector for [`table_write_axis`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableAxis {
    /// X (column) axis.
    X,
    /// Y (row) axis.
    Y,
}

/// Replace a table axis. `values` must not exceed the axis length; shorter
/// writes update the leading entries (axes must stay monotonically increasing
/// for interpolation, which is the caller's responsibility to validate).
pub fn table_write_axis(
    cfg: &mut EngineConfig,
    id: TableId,
    axis: TableAxis,
    values: &[f32],
) -> Result<(), TableWriteError> {
    fn copy_into(dst: &mut [f32], src: &[f32]) -> Result<(), TableWriteError> {
        if src.len() > dst.len() {
            return Err(TableWriteError::BadIndex);
        }
        dst[..src.len()].copy_from_slice(src);
        Ok(())
    }
    match (id, axis) {
        (TableId::Ignition, TableAxis::X) => copy_into(&mut cfg.ignition_rpm_bins, values),
        (TableId::Ignition, TableAxis::Y) => copy_into(&mut cfg.ignition_load_bins, values),
        (TableId::Lambda, TableAxis::X) => copy_into(&mut cfg.lambda_rpm_bins, values),
        (TableId::Lambda, TableAxis::Y) => copy_into(&mut cfg.lambda_load_bins, values),
        (TableId::Ve, TableAxis::X) => copy_into(&mut cfg.ve_rpm_bins, values),
        (TableId::Ve, TableAxis::Y) => copy_into(&mut cfg.ve_load_bins, values),
        (TableId::InjectorDeadtime, TableAxis::X) => copy_into(&mut cfg.injector_deadtime_voltage_bins, values),
        (TableId::InjectorDeadtime, TableAxis::Y) => copy_into(&mut cfg.injector_deadtime_pressure_bins, values),
        (TableId::DwellMs, TableAxis::X) => copy_into(&mut cfg.dwell_rpm_bins, values),
        (TableId::DwellVoltageCorr, TableAxis::X) => copy_into(&mut cfg.dwell_voltage_bins, values),
        (TableId::IatFuelCorr, TableAxis::X) => copy_into(&mut cfg.iat_fuel_temp_bins, values),
        (TableId::CltFuelCorr, TableAxis::X) => copy_into(&mut cfg.clt_fuel_temp_bins, values),
        (TableId::CltTimingCorr, TableAxis::X) => copy_into(&mut cfg.clt_corr_temp_bins, values),
        (TableId::IatTimingCorr, TableAxis::X) => copy_into(&mut cfg.iat_corr_temp_bins, values),
        _ => Err(TableWriteError::BadIndex),
    }
}

// ─── Schema hash & config CRC ───────────────────────────────────────────────

/// FNV-1a 32-bit step.
#[inline]
const fn fnv1a_step(hash: u32, byte: u8) -> u32 {
    (hash ^ byte as u32).wrapping_mul(0x0100_0193)
}

/// FNV-1a over a byte slice, continuing from `hash`.
pub fn fnv1a(mut hash: u32, bytes: &[u8]) -> u32 {
    let mut i = 0;
    while i < bytes.len() {
        hash = fnv1a_step(hash, bytes[i]);
        i += 1;
    }
    hash
}

/// FNV-1a offset basis.
pub const FNV_OFFSET: u32 = 0x811C_9DC5;

/// Deterministic hash of the parameter + table catalogs.
///
/// Combined with the telemetry catalog hash to form the `schema_hash`
/// advertised in `HelloInfo` (see `docs/api/04-parameter-model.md` §7).
pub fn catalog_hash() -> u32 {
    let mut h = FNV_OFFSET;
    for m in PARAM_CATALOG {
        h = fnv1a(h, &m.id.as_u16().to_le_bytes());
        h = fnv1a(h, m.key.as_bytes());
        h = fnv1a(h, &m.min.to_bits().to_le_bytes());
        h = fnv1a(h, &m.max.to_bits().to_le_bytes());
        h = fnv1a(h, &[m.category, m.digits, m.flags]);
    }
    for m in TABLE_CATALOG {
        h = fnv1a(h, &m.id.as_u16().to_le_bytes());
        h = fnv1a(h, m.key.as_bytes());
        h = fnv1a(h, &m.x_size.to_le_bytes());
        h = fnv1a(h, &m.y_size.to_le_bytes());
        h = fnv1a(h, &[m.category, m.dims, m.cell_digits]);
    }
    h
}

/// Deterministic checksum of every tunable value in a config snapshot.
///
/// Used by `Config.ConfigStatus` to report `ram_crc` / `flash_crc` and detect
/// unsaved (dirty) edits.
pub fn config_crc(cfg: &EngineConfig) -> u32 {
    let mut h = FNV_OFFSET;
    for m in PARAM_CATALOG {
        if let Some(v) = get_param(cfg, m.id) {
            h = fnv1a(h, &v.to_bits().to_le_bytes());
        }
    }
    for m in TABLE_CATALOG {
        let data = table_get(cfg, m.id);
        for v in data.x_axis.iter().chain(data.y_axis.iter()).chain(data.cells.iter()) {
            h = fnv1a(h, &v.to_bits().to_le_bytes());
        }
    }
    for &cyl in cfg.firing_order.iter() {
        h = fnv1a(h, &[cyl]);
    }
    h
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

    #[test]
    fn catalog_ids_unique() {
        for (i, a) in PARAM_CATALOG.iter().enumerate() {
            for b in PARAM_CATALOG.iter().skip(i + 1) {
                assert_ne!(a.id.as_u16(), b.id.as_u16());
            }
        }
        for (i, a) in TABLE_CATALOG.iter().enumerate() {
            for b in TABLE_CATALOG.iter().skip(i + 1) {
                assert_ne!(a.id.as_u16(), b.id.as_u16());
            }
        }
    }

    #[test]
    fn table_get_dimensions_match_meta() {
        let cfg = EngineConfig::default_4cyl();
        for m in TABLE_CATALOG {
            let data = table_get(&cfg, m.id);
            assert_eq!(data.x_axis.len(), m.x_size as usize, "x axis of {}", m.key);
            if m.dims == 2 {
                assert_eq!(data.y_axis.len(), m.y_size as usize, "y axis of {}", m.key);
                assert_eq!(data.cells.len(), (m.x_size * m.y_size) as usize, "cells of {}", m.key);
            } else {
                assert!(data.y_axis.is_empty());
                assert_eq!(data.cells.len(), m.x_size as usize, "cells of {}", m.key);
            }
        }
    }

    #[test]
    fn table_write_cell_round_trip() {
        let mut cfg = EngineConfig::default_4cyl();
        assert!(table_write_cell(&mut cfg, TableId::Ve, 3, 2, 0.9).is_ok());
        let data = table_get(&cfg, TableId::Ve);
        assert_eq!(data.cells[2 * RPM_BINS + 3], 0.9);
    }

    #[test]
    fn table_write_cell_rejects_out_of_range() {
        let mut cfg = EngineConfig::default_4cyl();
        assert_eq!(
            table_write_cell(&mut cfg, TableId::Lambda, 0, 0, 9.0),
            Err(TableWriteError::OutOfRange)
        );
        assert_eq!(
            table_write_cell(&mut cfg, TableId::Lambda, 99, 0, 1.0),
            Err(TableWriteError::BadIndex)
        );
    }

    #[test]
    fn table_write_axis_updates_bins() {
        let mut cfg = EngineConfig::default_4cyl();
        let new_axis = [400.0, 900.0, 1400.0, 1900.0, 2400.0, 2900.0, 3400.0, 3900.0,
                        4400.0, 4900.0, 5400.0, 5900.0, 6400.0, 6900.0, 7400.0, 7900.0];
        assert!(table_write_axis(&mut cfg, TableId::Ignition, TableAxis::X, &new_axis).is_ok());
        assert_eq!(cfg.ignition_rpm_bins[0], 400.0);
        // Y axis on a 1D curve is rejected
        assert!(table_write_axis(&mut cfg, TableId::DwellMs, TableAxis::Y, &[1.0]).is_err());
    }

    #[test]
    fn config_crc_detects_changes() {
        let cfg = EngineConfig::default_4cyl();
        let mut edited = cfg.clone();
        let base = config_crc(&cfg);
        assert_eq!(base, config_crc(&edited));
        assert!(set_param(&mut edited, ParamId::CrankingRpm, 350.0));
        assert_ne!(base, config_crc(&edited));
    }

    #[test]
    fn catalog_hash_is_stable() {
        assert_eq!(catalog_hash(), catalog_hash());
        assert_ne!(catalog_hash(), 0);
    }
}
