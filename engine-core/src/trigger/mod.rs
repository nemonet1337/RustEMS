//! Trigger wheel decoding — converts crank/cam pulse timestamps into
//! crank angle and engine-cycle synchronisation state.
//!
//! Supports missing-tooth crank wheels (e.g. 36-1, 60-2, 12-1) with an
//! optional cam sensor for 4-stroke phase identification.
//!
//! Algorithm derived from `firmware/controllers/trigger/trigger_decoder.cpp`.

pub mod missing_tooth;

pub use missing_tooth::MissingToothDecoder;

// ─────────────────────────────────────────────────────────────
// Core types
// ─────────────────────────────────────────────────────────────

/// Which edge of the trigger signal to use for synchronisation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncEdge {
    /// React to both rising and falling edges.
    Both,
    /// React to rising edges only.
    Rise,
    /// React to falling edges only.
    Fall,
}

/// A single trigger event from a crank or cam sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerSignal {
    /// Rising edge on the primary (crank) sensor.
    CrankRise,
    /// Falling edge on the primary (crank) sensor.
    CrankFall,
    /// Rising edge on the secondary (cam) sensor.
    CamRise,
    /// Falling edge on the secondary (cam) sensor.
    CamFall,
}

/// Synchronisation status of the decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncState {
    /// No valid sync point observed yet.
    Unsynced,
    /// Crank sync achieved; 4-stroke phase unknown (cam not yet used).
    CrankSynced,
    /// Full sync: crank position **and** 4-stroke phase are known.
    FullSync,
}

/// Engine cycle position for 4-stroke engines.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CyclePosition {
    /// Intake stroke (0-180° of 720° cycle).
    Intake,
    /// Compression stroke (180-360°).
    Compression,
    /// Power stroke (360-540°).
    Power,
    /// Exhaust stroke (540-720°).
    Exhaust,
}

impl CyclePosition {
    /// Get the cycle position from cam phase and tooth position.
    ///
    /// # Arguments
    /// * `cam_phase` - false = first 360°, true = second 360°
    /// * `tooth_deg` - Current crank angle (0-360°)
    pub fn from_phase_and_angle(cam_phase: bool, tooth_deg: f32) -> Self {
        let absolute_deg = if cam_phase {
            tooth_deg + 360.0
        } else {
            tooth_deg
        };
        match absolute_deg {
            0.0..=180.0 => CyclePosition::Intake,
            180.0..=360.0 => CyclePosition::Compression,
            360.0..=540.0 => CyclePosition::Power,
            _ => CyclePosition::Exhaust,
        }
    }
}

/// Output produced each time `decodeTriggerEvent` successfully advances state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriggerState {
    /// 0-based index of the current trigger tooth within one engine cycle.
    pub tooth_index: u32,
    /// Current synchronisation status.
    pub sync: SyncState,
    /// Estimated engine RPM based on the last tooth interval.
    ///
    /// `None` until at least two teeth have been observed.
    pub rpm: Option<f32>,
    /// Absolute crank angle within the engine cycle in degrees.
    ///
    /// For 4-stroke engines this spans `0.0..720.0`, and for 2-stroke engines
    /// it spans `0.0..360.0`.
    pub angle_deg: f32,
    /// For 4-stroke engines: true if we're in the second 360° of the 720° cycle
    /// (compression/power stroke vs intake/exhaust). Only valid when sync == FullSync.
    pub cam_phase: bool,
    /// Current engine cycle position (intake/compression/power/exhaust).
    /// Only valid when sync == FullSync.
    pub cycle_position: Option<CyclePosition>,
    /// Index of the currently firing cylinder (0-based).
    /// Only valid when sync == FullSync and cam is synced.
    pub current_cylinder: Option<u8>,
}

impl TriggerState {
    /// Returns true if full sequential injection is possible.
    pub fn is_sequential_ready(&self) -> bool {
        self.sync == SyncState::FullSync && self.cycle_position.is_some()
    }
}

/// Error produced when decoding fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerError {
    /// Too many teeth observed between two sync points.
    TooManyTeeth,
    /// Too few teeth observed between two sync points.
    TooFewTeeth,
    /// More than 1 second elapsed since the last trigger event (engine stalled).
    EngineStalled,
    /// Gap ratio outside the expected sync window — noise or misconfiguration.
    InvalidGapRatio,
}
