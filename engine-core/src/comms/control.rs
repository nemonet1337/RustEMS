//! Control-plane actions: temporary overrides, bench tests, calibration (RDP).
//!
//! See `docs/api/05-telemetry-control-diagnostics.md` §2. All overrides are
//! fail-safe: they expire after `timeout_ms`, on explicit clear, or when the
//! transport disconnects ([`Overrides::clear_all`]).

// ─── Overrides ───────────────────────────────────────────────────────────────

/// Override targets (`SetOverride.target`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OverrideTarget {
    /// Cut spark on all cylinders while set (`value != 0`).
    SparkCut = 0,
    /// Cut fuel on all cylinders while set (`value != 0`).
    FuelCut = 1,
    /// Fix ignition timing at `value` degrees BTDC.
    TimingFix = 2,
    /// Fix idle valve position at `value` percent.
    IdlePosition = 3,
    /// Fix wastegate duty at `value` percent.
    BoostDuty = 4,
    /// Add `value` percent to injector duty.
    InjectorDuty = 5,
}

/// Number of override targets.
pub const OVERRIDE_TARGETS: usize = 6;

impl OverrideTarget {
    /// Parse from the wire byte.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::SparkCut),
            1 => Some(Self::FuelCut),
            2 => Some(Self::TimingFix),
            3 => Some(Self::IdlePosition),
            4 => Some(Self::BoostDuty),
            5 => Some(Self::InjectorDuty),
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
struct OverrideSlot {
    value: f32,
    expires_ms: u32,
}

/// Active control overrides with automatic expiry.
#[derive(Default)]
pub struct Overrides {
    slots: [Option<OverrideSlot>; OVERRIDE_TARGETS],
}

impl Overrides {
    /// Create with no overrides active.
    pub const fn new() -> Self {
        Self {
            slots: [None; OVERRIDE_TARGETS],
        }
    }

    /// Activate an override for `timeout_ms` (clamped to 1..=30000 ms).
    pub fn set(&mut self, target: OverrideTarget, value: f32, timeout_ms: u16, now_ms: u32) {
        let timeout = timeout_ms.clamp(1, 30_000) as u32;
        self.slots[target as usize] = Some(OverrideSlot {
            value,
            expires_ms: now_ms.wrapping_add(timeout),
        });
    }

    /// Deactivate an override. Returns `true` when one was active.
    pub fn clear(&mut self, target: OverrideTarget) -> bool {
        self.slots[target as usize].take().is_some()
    }

    /// Deactivate everything (transport disconnect fail-safe).
    pub fn clear_all(&mut self) {
        self.slots = [None; OVERRIDE_TARGETS];
    }

    /// Current override value, expiring lazily.
    pub fn get(&mut self, target: OverrideTarget, now_ms: u32) -> Option<f32> {
        let slot = self.slots[target as usize]?;
        // Wrapping-aware "expired" check: valid windows are ≤ 30 s.
        if now_ms.wrapping_sub(slot.expires_ms) < u32::MAX / 2 {
            self.slots[target as usize] = None;
            return None;
        }
        Some(slot.value)
    }

    /// True when any override is currently set (without expiry processing).
    pub fn any_active(&self) -> bool {
        self.slots.iter().any(|s| s.is_some())
    }
}

// ─── Bench test ──────────────────────────────────────────────────────────────

/// Bench-test actuator targets (`BenchTest.target`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BenchTarget {
    /// Fuel injector by cylinder index.
    Injector = 0,
    /// Ignition coil by cylinder index.
    IgnitionCoil = 1,
    /// Fuel pump relay.
    FuelPump = 2,
    /// Cooling fan relay.
    Fan = 3,
    /// Idle valve.
    Idle = 4,
    /// Tachometer output.
    Tachometer = 5,
}

impl BenchTarget {
    /// Parse from the wire byte.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Injector),
            1 => Some(Self::IgnitionCoil),
            2 => Some(Self::FuelPump),
            3 => Some(Self::Fan),
            4 => Some(Self::Idle),
            5 => Some(Self::Tachometer),
            _ => None,
        }
    }
}

/// A validated bench-test request, queued for the control loop to execute.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BenchTestSpec {
    /// Actuator to exercise.
    pub target: BenchTarget,
    /// Channel index (cylinder, fan number, …).
    pub index: u8,
    /// On time per pulse (ms).
    pub on_ms: u16,
    /// Off time between pulses (ms).
    pub off_ms: u16,
    /// Number of pulses.
    pub count: u16,
}

// ─── Calibration ─────────────────────────────────────────────────────────────

/// Calibration routines (`Calibrate.routine`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CalibrateRoutine {
    /// Learn current TPS ADC as closed throttle.
    TpsClosed = 0,
    /// Learn current TPS ADC as wide-open throttle.
    TpsOpen = 1,
    /// Learn barometric pressure from MAP at key-on.
    MapBaro = 2,
    /// Clear adaptive learning (LTFT trims etc.).
    ClearAdaptive = 3,
}

impl CalibrateRoutine {
    /// Parse from the wire byte.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::TpsClosed),
            1 => Some(Self::TpsOpen),
            2 => Some(Self::MapBaro),
            3 => Some(Self::ClearAdaptive),
            _ => None,
        }
    }
}

/// Learned calibration values, updated by `Calibrate` requests.
#[derive(Clone, Copy, Debug)]
pub struct Calibrations {
    /// TPS percent reading captured at closed throttle.
    pub tps_closed_pct: f32,
    /// TPS percent reading captured at wide-open throttle.
    pub tps_open_pct: f32,
    /// Barometric pressure learned from MAP (kPa).
    pub baro_kpa: f32,
    /// Set when adaptive trims should be wiped by the control loop.
    pub clear_adaptive_pending: bool,
}

impl Default for Calibrations {
    fn default() -> Self {
        Self {
            tps_closed_pct: 0.0,
            tps_open_pct: 100.0,
            baro_kpa: 101.325,
            clear_adaptive_pending: false,
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn override_set_get_clear() {
        let mut ov = Overrides::new();
        ov.set(OverrideTarget::TimingFix, 10.0, 1000, 0);
        assert_eq!(ov.get(OverrideTarget::TimingFix, 500), Some(10.0));
        assert!(ov.clear(OverrideTarget::TimingFix));
        assert_eq!(ov.get(OverrideTarget::TimingFix, 500), None);
    }

    #[test]
    fn override_expires() {
        let mut ov = Overrides::new();
        ov.set(OverrideTarget::SparkCut, 1.0, 100, 0);
        assert_eq!(ov.get(OverrideTarget::SparkCut, 99), Some(1.0));
        assert_eq!(ov.get(OverrideTarget::SparkCut, 101), None);
        // Expiry is sticky
        assert_eq!(ov.get(OverrideTarget::SparkCut, 50), None);
    }

    #[test]
    fn clear_all_failsafe() {
        let mut ov = Overrides::new();
        ov.set(OverrideTarget::FuelCut, 1.0, 30_000, 0);
        ov.set(OverrideTarget::BoostDuty, 50.0, 30_000, 0);
        assert!(ov.any_active());
        ov.clear_all();
        assert!(!ov.any_active());
    }

    #[test]
    fn enums_round_trip() {
        for v in 0..OVERRIDE_TARGETS as u8 {
            assert_eq!(OverrideTarget::from_u8(v).map(|t| t as u8), Some(v));
        }
        assert!(OverrideTarget::from_u8(99).is_none());
        for v in 0..=5u8 {
            assert_eq!(BenchTarget::from_u8(v).map(|t| t as u8), Some(v));
        }
        for v in 0..=3u8 {
            assert_eq!(CalibrateRoutine::from_u8(v).map(|t| t as u8), Some(v));
        }
    }
}
