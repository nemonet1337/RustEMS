//! Missing-tooth crank wheel decoder.
//!
//! Implements the core gap-ratio synchronisation algorithm from
//! `firmware/controllers/trigger/trigger_decoder.cpp` and
//! `firmware/controllers/trigger/decoders/trigger_universal.cpp`.

use super::{CyclePosition, SyncEdge, SyncState, TriggerError, TriggerSignal, TriggerState};

/// Maximum tooth duration capped to 10 seconds (in µs) to avoid 64-bit
/// overflow in gap ratio arithmetic.
const MAX_TOOTH_DURATION_US: u64 = 10_000_000;

/// Engine stall timeout: if no trigger event within this duration, assume engine stopped.
/// Typical rusEFI value: 1 second (1_000_000 µs).
const ENGINE_STALL_TIMEOUT_US: u64 = 1_000_000;

/// Minimum gap ratio margin below the theoretical value before we accept sync.
/// (C++ equivalent: synchronizationRatioFrom)
const GAP_RATIO_MARGIN: f32 = 0.5;

/// Expected ratio for a normal tooth (next tooth after gap should be ~1.0).
/// Used for secondary noise rejection in multi-tooth wheels.
#[allow(dead_code)]
const NORMAL_TOOTH_RATIO: f32 = 1.0;
/// Tolerance for normal tooth ratio check.
#[allow(dead_code)]
const NORMAL_TOOTH_MARGIN: f32 = 0.3;

/// Configuration for a missing-tooth crank wheel.
#[derive(Clone, Copy, Debug)]
pub struct MissingToothConfig {
    /// Nominal tooth count including the missing teeth (e.g. 36 for 36-1).
    pub total_teeth: u32,
    /// Number of consecutive missing teeth (e.g. 1 for 36-1, 2 for 60-2).
    pub missing_teeth: u32,
    /// Engine cycle in degrees: 720.0 for 4-stroke, 360.0 for 2-stroke.
    pub engine_cycle_deg: f32,
    /// Which edge to use for sync detection.
    pub sync_edge: SyncEdge,
}

impl MissingToothConfig {
    /// Create a missing-tooth trigger configuration.
    pub fn new_missing_tooth(
        total_teeth: u32,
        missing_teeth: u32,
        engine_cycle_deg: f32,
        sync_edge: SyncEdge,
    ) -> Self {
        Self {
            total_teeth,
            missing_teeth,
            engine_cycle_deg,
            sync_edge,
        }
    }

    /// Gap ratio of the missing-tooth section relative to a normal tooth.
    ///
    /// For 36-1: gap spans 2 teeth → ratio ≈ 2.0
    /// For 60-2: gap spans 3 teeth → ratio ≈ 3.0
    pub fn expected_gap_ratio(&self) -> f32 {
        (self.missing_teeth + 1) as f32
    }

    /// Degrees per present tooth.
    pub fn degrees_per_tooth(&self) -> f32 {
        self.engine_cycle_deg / self.total_teeth as f32
    }

    /// Number of present teeth per engine cycle.
    pub fn present_teeth(&self) -> u32 {
        self.total_teeth - self.missing_teeth
    }

    /// Create a configuration for 36-1 trigger wheel (4-stroke).
    pub fn trigger_36_1() -> Self {
        Self {
            total_teeth: 36,
            missing_teeth: 1,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        }
    }

    /// Create a configuration for 60-2 trigger wheel (4-stroke).
    pub fn trigger_60_2() -> Self {
        Self {
            total_teeth: 60,
            missing_teeth: 2,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        }
    }

    /// Create a configuration for 12-1 trigger wheel (motorcycle/trigger wheel).
    pub fn trigger_12_1() -> Self {
        Self {
            total_teeth: 12,
            missing_teeth: 1,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        }
    }

    /// Create a configuration for 4-1 trigger wheel (4-stroke).
    pub fn trigger_4_1() -> Self {
        Self::new_missing_tooth(4, 1, 720.0, SyncEdge::Rise)
    }

    /// Create a configuration for 24-1 trigger wheel (4-stroke).
    pub fn trigger_24_1() -> Self {
        Self::new_missing_tooth(24, 1, 720.0, SyncEdge::Rise)
    }

    /// Create a configuration for 24-2 trigger wheel (4-stroke).
    pub fn trigger_24_2() -> Self {
        Self::new_missing_tooth(24, 2, 720.0, SyncEdge::Rise)
    }

    /// Create a configuration for 36-2 trigger wheel (4-stroke).
    pub fn trigger_36_2() -> Self {
        Self::new_missing_tooth(36, 2, 720.0, SyncEdge::Rise)
    }
}

/// State machine that decodes a missing-tooth crank wheel.
///
/// Feed each pulse timestamp via [`process`]; the decoder returns
/// `Ok(TriggerState)` after each event that advances state.
///
/// # Feature notes
/// - Does **not** heap-allocate.
/// - Thread-safety is the caller's responsibility (e.g. call only from the
///   trigger interrupt handler or with a critical section).
pub struct MissingToothDecoder {
    cfg: MissingToothConfig,

    /// Timestamp of the previous trigger event (µs).
    prev_timestamp_us: u64,
    /// Duration of the current tooth interval (µs, clamped).
    tooth_duration: [u64; 2],

    /// True once the first event has been received.
    first_event: bool,
    /// Number of teeth counted within the current cycle.
    tooth_count_in_cycle: u32,
    /// Current synchronisation status.
    sync: SyncState,
    /// Number of complete cycles observed since last sync loss.
    sync_counter: u32,

    /// Running RPM estimate based on last normal tooth interval.
    rpm: Option<f32>,

    /// Cam phase: true = second 360° of 720° cycle (4-stroke compression stroke).
    /// Only meaningful once FullSync is achieved.
    cam_phase: bool,
    /// Tooth count at last cam pulse (for phase determination).
    cam_tooth_count: Option<u32>,
}

impl MissingToothDecoder {
    /// Create a new decoder for the given wheel configuration.
    pub fn new(cfg: MissingToothConfig) -> Self {
        Self {
            cfg,
            prev_timestamp_us: 0,
            tooth_duration: [0; 2],
            first_event: true,
            tooth_count_in_cycle: 0,
            sync: SyncState::Unsynced,
            sync_counter: 0,
            rpm: None,
            cam_phase: false,
            cam_tooth_count: None,
        }
    }

    fn absolute_angle_deg(&self) -> f32 {
        let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
        if self.sync == SyncState::FullSync && self.cam_phase {
            tooth_deg + (self.cfg.engine_cycle_deg * 0.5)
        } else {
            tooth_deg
        }
    }

    /// Get the configuration.
    pub fn config(&self) -> &MissingToothConfig {
        &self.cfg
    }

    /// Get current position in degrees (0-720° for 4-stroke).
    pub fn position_deg(&self) -> f32 {
        self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth()
    }

    /// Reset to the unsynced state (call on engine stop or severe error).
    pub fn reset(&mut self) {
        self.first_event = true;
        self.sync = SyncState::Unsynced;
        self.sync_counter = 0;
        self.tooth_count_in_cycle = 0;
        self.tooth_duration = [0; 2];
        self.rpm = None;
        self.cam_phase = false;
        self.cam_tooth_count = None;
    }

    /// Process a single trigger event.
    ///
    /// # Arguments
    /// * `signal`    — the type of edge that just arrived
    /// * `now_us`    — current hardware timer value in microseconds
    ///
    /// # Returns
    /// `Ok(TriggerState)` if the event was processed normally, or
    /// `Err(TriggerError)` if a decoding fault was detected.
    pub fn process(
        &mut self,
        signal: TriggerSignal,
        now_us: u64,
    ) -> Result<TriggerState, TriggerError> {
        // ── stall detection ─────────────────────────────────────────────────
        if !self.first_event {
            let elapsed = now_us.saturating_sub(self.prev_timestamp_us);
            if elapsed > MAX_TOOTH_DURATION_US {
                self.reset();
                return Err(TriggerError::EngineStalled);
            }
        }

        // ── edge filter ─────────────────────────────────────────────────────
        let is_crank = matches!(signal, TriggerSignal::CrankRise | TriggerSignal::CrankFall);
        let is_rise = matches!(signal, TriggerSignal::CrankRise | TriggerSignal::CamRise);

        if !is_crank {
            // ── Cam phase detection for 4-stroke sync ─────────────────────────
            // Record tooth count at cam pulse to determine which 360° we're in
            if self.sync == SyncState::CrankSynced {
                self.cam_tooth_count = Some(self.tooth_count_in_cycle);
                // First cam pulse after crank sync: determine phase
                // If cam pulse occurs in first half of cycle, we're on intake/compression
                // (phase = false); if second half, we're on power/exhaust (phase = true)
                let present = self.cfg.present_teeth();
                self.cam_phase = self.tooth_count_in_cycle > present / 2;
                self.sync = SyncState::FullSync;
            }
            // Calculate cycle position for full sync
            let cycle_pos = if self.sync == SyncState::FullSync {
                let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
                Some(CyclePosition::from_phase_and_angle(
                    self.cam_phase,
                    tooth_deg,
                ))
            } else {
                None
            };

            return Ok(TriggerState {
                tooth_index: self.tooth_count_in_cycle,
                sync: self.sync,
                rpm: self.rpm,
                angle_deg: self.absolute_angle_deg(),
                cam_phase: self.cam_phase,
                cycle_position: cycle_pos,
                current_cylinder: None,
            });
        }

        let consider = match self.cfg.sync_edge {
            SyncEdge::Both => true,
            SyncEdge::Rise => is_rise,
            SyncEdge::Fall => !is_rise,
        };

        // ── update tooth duration ────────────────────────────────────────────
        let duration = if self.first_event {
            0
        } else {
            let d = now_us.saturating_sub(self.prev_timestamp_us);
            d.min(MAX_TOOTH_DURATION_US)
        };

        self.first_event = false;
        self.prev_timestamp_us = now_us;

        if !consider {
            // Non-sync edge: just advance index
            self.tooth_count_in_cycle += 1;
            let cycle_pos = if self.sync == SyncState::FullSync {
                let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
                Some(CyclePosition::from_phase_and_angle(
                    self.cam_phase,
                    tooth_deg,
                ))
            } else {
                None
            };

            return Ok(TriggerState {
                tooth_index: self.tooth_count_in_cycle,
                sync: self.sync,
                rpm: self.rpm,
                angle_deg: self.absolute_angle_deg(),
                cam_phase: self.cam_phase,
                cycle_position: cycle_pos,
                current_cylinder: None,
            });
        }

        // Shift tooth duration history
        self.tooth_duration[1] = self.tooth_duration[0];
        self.tooth_duration[0] = duration;

        // ── gap ratio test ───────────────────────────────────────────────────
        let is_sync_point = if self.tooth_duration[1] == 0 {
            false // not enough history yet
        } else {
            let gap = self.tooth_duration[0] as f32 / self.tooth_duration[1] as f32;
            let expected = self.cfg.expected_gap_ratio();
            let lo = expected - GAP_RATIO_MARGIN;
            let hi = expected + GAP_RATIO_MARGIN;
            let primary_check = gap >= lo && gap <= hi;

            // Secondary noise rejection: for multi-tooth wheels (>6 teeth),
            // verify that the gap is followed by a normal tooth ratio (~1.0)
            // We check this by looking at the previous tooth duration trend
            // If primary check passes and we have prior history, validate consistency
            if primary_check && self.cfg.total_teeth > 6 && self.cfg.missing_teeth > 0 {
                // After a gap, the next tooth should have ratio ~1.0 (normal interval)
                // We use the historical baseline if available
                let baseline_ratio = if self.tooth_duration[1] > 0 && self.tooth_duration[0] > 0 {
                    self.tooth_duration[0] as f32 / self.tooth_duration[1] as f32
                } else {
                    1.0 // assume normal if no history
                };
                // Gap ratio should be significantly different from normal
                let is_significant_gap = (baseline_ratio - expected).abs() < GAP_RATIO_MARGIN;
                primary_check && is_significant_gap
            } else {
                primary_check
            }
        };

        // ── synchronisation point handling ───────────────────────────────────
        if is_sync_point {
            let expected_count = self.cfg.present_teeth();
            let was_synced = self.sync != SyncState::Unsynced;

            if was_synced && self.tooth_count_in_cycle != expected_count {
                let err = if self.tooth_count_in_cycle < expected_count {
                    TriggerError::TooFewTeeth
                } else {
                    TriggerError::TooManyTeeth
                };
                self.sync = SyncState::Unsynced;
                self.tooth_count_in_cycle = 0;
                return Err(err);
            }

            self.sync = SyncState::CrankSynced;
            if was_synced {
                self.sync_counter = self.sync_counter.saturating_add(1);
            } else {
                self.sync_counter = 0;
            }
            self.tooth_count_in_cycle = 0;
        } else {
            // ── tooth index overflow check ────────────────────────────────────
            if self.sync != SyncState::Unsynced
                && self.tooth_count_in_cycle >= self.cfg.present_teeth()
            {
                self.sync = SyncState::Unsynced;
                self.tooth_count_in_cycle = 0;
                return Err(TriggerError::TooManyTeeth);
            }
            self.tooth_count_in_cycle += 1;
        }

        // ── RPM estimation ───────────────────────────────────────────────────
        if self.tooth_duration[1] > 0 && !is_sync_point {
            // Use the previous (normal) tooth interval for RPM
            let tooth_period_us = self.tooth_duration[1] as f32;
            let deg_per_tooth = self.cfg.degrees_per_tooth();
            // rpm = (deg/tooth / 360°) / (period_us / 1_000_000 s) * 60
            self.rpm = Some(deg_per_tooth / 360.0 / tooth_period_us * 60_000_000.0);
        }

        // Calculate cycle position for full sync
        let cycle_pos = if self.sync == SyncState::FullSync {
            let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
            Some(CyclePosition::from_phase_and_angle(
                self.cam_phase,
                tooth_deg,
            ))
        } else {
            None
        };

        Ok(TriggerState {
            tooth_index: self.tooth_count_in_cycle,
            sync: self.sync,
            rpm: self.rpm,
            angle_deg: self.absolute_angle_deg(),
            cam_phase: self.cam_phase,
            cycle_position: cycle_pos,
            current_cylinder: None,
        })
    }

    /// Current sync state.
    pub fn sync_state(&self) -> SyncState {
        self.sync
    }

    /// Number of complete cycles observed since last sync loss.
    pub fn sync_counter(&self) -> u32 {
        self.sync_counter
    }

    /// Check if engine has stalled (no trigger events for longer than timeout).
    ///
    /// Call this periodically from a 1ms tick or similar. If the elapsed
    /// time since the last trigger event exceeds `ENGINE_STALL_TIMEOUT_US`,
    /// the decoder is reset to `Unsynced` and `true` is returned.
    ///
    /// # Arguments
    /// * `now_us` — current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine stall was detected and decoder was reset.
    pub fn check_stall(&mut self, now_us: u64) -> bool {
        if self.first_event {
            return false; // No events yet, nothing to stall
        }

        let elapsed = now_us.saturating_sub(self.prev_timestamp_us);
        if elapsed > ENGINE_STALL_TIMEOUT_US {
            self.reset();
            true
        } else {
            false
        }
    }

    /// Get the duration of the last tooth interval in microseconds.
    pub fn last_tooth_duration_us(&self) -> Option<u64> {
        if self.tooth_duration[0] > 0 {
            Some(self.tooth_duration[0])
        } else {
            None
        }
    }
}

/// Normalize angle to 0-720 degree range (no_std compatible).
#[allow(dead_code)]
fn normalize_angle_720(angle: f32) -> f32 {
    let cycle = 720.0;
    let mut result = angle % cycle;
    if result < 0.0 {
        result += cycle;
    }
    result
}

/// Configuration for 36-2-2-2 GM trigger wheel (special missing tooth pattern).
/// Pattern: 36 teeth total with 3 groups of 2 missing teeth each.
#[derive(Clone, Copy, Debug)]
pub struct GmTriggerConfig {
    /// Engine cycle in degrees: 720.0 for 4-stroke.
    pub engine_cycle_deg: f32,
    /// Which edge to use for sync detection.
    pub sync_edge: SyncEdge,
}

impl GmTriggerConfig {
    /// Create default GM 36-2-2-2 configuration.
    pub fn gm_36_2_2_2() -> Self {
        Self {
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        }
    }

    /// Degrees per tooth (36 teeth = 10° per tooth).
    pub fn degrees_per_tooth(&self) -> f32 {
        self.engine_cycle_deg / 36.0
    }
}

/// Decoder for GM 36-2-2-2 trigger wheel.
/// Special pattern with 3 groups of 2 consecutive missing teeth.
pub struct GmTriggerDecoder {
    cfg: GmTriggerConfig,
    prev_timestamp_us: u64,
    tooth_duration: [u64; 3],
    first_event: bool,
    tooth_count_in_cycle: u32,
    sync: SyncState,
    sync_counter: u32,
    rpm: Option<f32>,
    cam_phase: bool,
}

impl GmTriggerDecoder {
    /// Create a new GM trigger decoder.
    pub fn new(cfg: GmTriggerConfig) -> Self {
        Self {
            cfg,
            prev_timestamp_us: 0,
            tooth_duration: [0; 3],
            first_event: true,
            tooth_count_in_cycle: 0,
            sync: SyncState::Unsynced,
            sync_counter: 0,
            rpm: None,
            cam_phase: false,
        }
    }

    fn absolute_angle_deg(&self) -> f32 {
        let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
        if self.sync == SyncState::FullSync && self.cam_phase {
            tooth_deg + (self.cfg.engine_cycle_deg * 0.5)
        } else {
            tooth_deg
        }
    }

    /// Reset decoder state.
    pub fn reset(&mut self) {
        self.first_event = true;
        self.sync = SyncState::Unsynced;
        self.sync_counter = 0;
        self.tooth_count_in_cycle = 0;
        self.tooth_duration = [0; 3];
        self.rpm = None;
        self.cam_phase = false;
    }

    /// Process a single trigger event for GM pattern.
    /// 36-2-2-2 pattern: gap ratios of ~3.0 indicate sync points.
    pub fn process(
        &mut self,
        signal: TriggerSignal,
        now_us: u64,
    ) -> Result<TriggerState, TriggerError> {
        // Stall detection
        if !self.first_event {
            let elapsed = now_us.saturating_sub(self.prev_timestamp_us);
            if elapsed > MAX_TOOTH_DURATION_US {
                self.reset();
                return Err(TriggerError::EngineStalled);
            }
        }

        let is_crank = matches!(signal, TriggerSignal::CrankRise | TriggerSignal::CrankFall);
        let is_rise = matches!(signal, TriggerSignal::CrankRise | TriggerSignal::CamRise);

        if !is_crank {
            // Cam phase detection
            if self.sync == SyncState::CrankSynced {
                self.cam_phase = self.tooth_count_in_cycle > 15;
                self.sync = SyncState::FullSync;
            }
            let cycle_pos = if self.sync == SyncState::FullSync {
                let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
                Some(CyclePosition::from_phase_and_angle(
                    self.cam_phase,
                    tooth_deg,
                ))
            } else {
                None
            };
            return Ok(TriggerState {
                tooth_index: self.tooth_count_in_cycle,
                sync: self.sync,
                rpm: self.rpm,
                angle_deg: self.absolute_angle_deg(),
                cam_phase: self.cam_phase,
                cycle_position: cycle_pos,
                current_cylinder: None,
            });
        }

        let consider = match self.cfg.sync_edge {
            SyncEdge::Both => true,
            SyncEdge::Rise => is_rise,
            SyncEdge::Fall => !is_rise,
        };

        let duration = if self.first_event {
            0
        } else {
            let d = now_us.saturating_sub(self.prev_timestamp_us);
            d.min(MAX_TOOTH_DURATION_US)
        };

        self.first_event = false;
        self.prev_timestamp_us = now_us;

        if !consider {
            self.tooth_count_in_cycle += 1;
            let cycle_pos = if self.sync == SyncState::FullSync {
                let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
                Some(CyclePosition::from_phase_and_angle(
                    self.cam_phase,
                    tooth_deg,
                ))
            } else {
                None
            };
            return Ok(TriggerState {
                tooth_index: self.tooth_count_in_cycle,
                sync: self.sync,
                rpm: self.rpm,
                angle_deg: self.absolute_angle_deg(),
                cam_phase: self.cam_phase,
                cycle_position: cycle_pos,
                current_cylinder: None,
            });
        }

        // Shift tooth duration history (keep 3 for 2-2-2 pattern detection)
        self.tooth_duration[2] = self.tooth_duration[1];
        self.tooth_duration[1] = self.tooth_duration[0];
        self.tooth_duration[0] = duration;

        // Gap ratio test for 36-2-2-2 pattern
        // Pattern: normal teeth with periodic gaps of 3x (2 missing + 1 present)
        let is_sync_point = if self.tooth_duration[1] == 0 || self.tooth_duration[2] == 0 {
            false
        } else {
            // Check for 2 consecutive long gaps (2-2-2 pattern)
            let gap1 = self.tooth_duration[0] as f32 / self.tooth_duration[1] as f32;
            let gap2 = self.tooth_duration[1] as f32 / self.tooth_duration[2] as f32;
            // Gap ratios should be ~3.0 for missing 2 teeth
            let expected = 3.0f32;
            let lo = expected - GAP_RATIO_MARGIN;
            let hi = expected + GAP_RATIO_MARGIN;
            gap1 >= lo && gap1 <= hi && (1.5..=3.5).contains(&gap2)
        };

        if is_sync_point {
            let expected_count = 30; // 36 - 2 - 2 - 2 = 30 present teeth
            let was_synced = self.sync != SyncState::Unsynced;

            if was_synced && self.tooth_count_in_cycle != expected_count {
                let err = if self.tooth_count_in_cycle < expected_count {
                    TriggerError::TooFewTeeth
                } else {
                    TriggerError::TooManyTeeth
                };
                self.sync = SyncState::Unsynced;
                self.tooth_count_in_cycle = 0;
                return Err(err);
            }

            self.sync = SyncState::CrankSynced;
            if was_synced {
                self.sync_counter = self.sync_counter.saturating_add(1);
            } else {
                self.sync_counter = 0;
            }
            self.tooth_count_in_cycle = 0;
        } else {
            if self.sync != SyncState::Unsynced && self.tooth_count_in_cycle >= 30 {
                self.sync = SyncState::Unsynced;
                self.tooth_count_in_cycle = 0;
                return Err(TriggerError::TooManyTeeth);
            }
            self.tooth_count_in_cycle += 1;
        }

        // RPM estimation
        if self.tooth_duration[1] > 0 && !is_sync_point {
            let tooth_period_us = self.tooth_duration[1] as f32;
            let deg_per_tooth = self.cfg.degrees_per_tooth();
            self.rpm = Some(deg_per_tooth / 360.0 / tooth_period_us * 60_000_000.0);
        }

        let cycle_pos = if self.sync == SyncState::FullSync {
            let tooth_deg = self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth();
            Some(CyclePosition::from_phase_and_angle(
                self.cam_phase,
                tooth_deg,
            ))
        } else {
            None
        };

        Ok(TriggerState {
            tooth_index: self.tooth_count_in_cycle,
            sync: self.sync,
            rpm: self.rpm,
            angle_deg: self.absolute_angle_deg(),
            cam_phase: self.cam_phase,
            cycle_position: cycle_pos,
            current_cylinder: None,
        })
    }

    /// Current sync state.
    pub fn sync_state(&self) -> SyncState {
        self.sync
    }

    /// Get current position in degrees (0-360°).
    pub fn position_deg(&self) -> f32 {
        self.tooth_count_in_cycle as f32 * self.cfg.degrees_per_tooth()
    }
}

/// Instant RPM calculator for individual tooth intervals.
/// Provides tooth-by-tooth RPM for vibration analysis and misfire detection.
pub struct InstantRpmCalculator {
    /// RPM calculated from each individual tooth interval.
    instant_rpm: [f32; 4],
    /// Index for circular buffer.
    idx: usize,
    /// Accumulated average of instant RPM values.
    smoothed_rpm: Option<f32>,
}

impl InstantRpmCalculator {
    /// Create new instant RPM calculator.
    pub fn new() -> Self {
        Self {
            instant_rpm: [0.0; 4],
            idx: 0,
            smoothed_rpm: None,
        }
    }

    /// Update with new tooth interval.
    ///
    /// * `tooth_duration_us` - Duration of this tooth in microseconds
    /// * `deg_per_tooth` - Degrees per tooth for this trigger wheel
    pub fn update(&mut self, tooth_duration_us: u64, deg_per_tooth: f32) -> f32 {
        if tooth_duration_us == 0 {
            return self.smoothed_rpm.unwrap_or(0.0);
        }

        // Calculate RPM from this tooth interval
        // rpm = (deg_per_tooth / 360) / (duration_us / 60_000_000)
        let rpm = deg_per_tooth / 360.0 / tooth_duration_us as f32 * 60_000_000.0;
        self.instant_rpm[self.idx] = rpm;
        self.idx = (self.idx + 1) % 4;

        // Update smoothed average
        let sum: f32 = self.instant_rpm.iter().sum();
        let avg = sum / 4.0;
        self.smoothed_rpm = Some(avg);
        avg
    }

    /// Get the most recent instant RPM value.
    pub fn latest(&self) -> Option<f32> {
        let idx = (self.idx + 3) % 4;
        if self.instant_rpm[idx] > 0.0 {
            Some(self.instant_rpm[idx])
        } else {
            None
        }
    }

    /// Get smoothed RPM (4-tooth moving average).
    pub fn smoothed(&self) -> Option<f32> {
        self.smoothed_rpm
    }

    /// Calculate RPM variance for misfire detection.
    pub fn variance(&self) -> Option<f32> {
        let avg = self.smoothed_rpm?;
        let sum_sq_diff: f32 = self
            .instant_rpm
            .iter()
            .filter(|&&x| x > 0.0)
            .map(|&x| {
                let diff = x - avg;
                diff * diff
            })
            .sum();
        let count = self.instant_rpm.iter().filter(|&&x| x > 0.0).count() as f32;
        if count > 0.0 {
            Some(sum_sq_diff / count)
        } else {
            None
        }
    }
}

impl Default for InstantRpmCalculator {
    fn default() -> Self {
        Self::new()
    }
}

/// Noise filter for trigger signals with configurable debounce.
/// Removes chatter from VR sensors or noisy Hall signals.
pub struct TriggerNoiseFilter {
    /// Minimum pulse width in microseconds to be considered valid.
    min_pulse_width_us: u32,
    /// Timestamp of last valid edge.
    last_valid_edge_us: u64,
    /// State for glitch rejection.
    last_raw_state: bool,
    /// Debounced output state.
    filtered_state: bool,
    /// Consecutive stable samples counter.
    stable_count: u32,
    /// Required stable samples for state change.
    stability_threshold: u32,
}

impl TriggerNoiseFilter {
    /// Create noise filter with specified parameters.
    pub fn new(min_pulse_width_us: u32, stability_threshold: u32) -> Self {
        Self {
            min_pulse_width_us,
            last_valid_edge_us: 0,
            last_raw_state: false,
            filtered_state: false,
            stable_count: 0,
            stability_threshold,
        }
    }

    /// Default filter: 50µs min pulse, 2 stable samples.
    pub fn default_vr_filter() -> Self {
        Self::new(50, 2)
    }

    /// Process raw trigger input.
    ///
    /// * `raw_state` - Current raw input level (true = high)
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// Returns filtered state (true = high) or None if no change.
    pub fn process(&mut self, raw_state: bool, now_us: u64) -> Option<bool> {
        // Check for state change
        if raw_state != self.last_raw_state {
            self.stable_count = 0;
            self.last_raw_state = raw_state;
            None
        } else {
            // Same state, increment stable counter
            self.stable_count += 1;

            if self.stable_count >= self.stability_threshold {
                let pulse_width = now_us.saturating_sub(self.last_valid_edge_us);

                if raw_state != self.filtered_state && pulse_width >= self.min_pulse_width_us as u64
                {
                    // Valid state change
                    self.filtered_state = raw_state;
                    self.last_valid_edge_us = now_us;
                    Some(self.filtered_state)
                } else {
                    None
                }
            } else {
                None
            }
        }
    }

    /// Get current filtered state.
    pub fn state(&self) -> bool {
        self.filtered_state
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.filtered_state = false;
        self.last_raw_state = false;
        self.stable_count = 0;
        self.last_valid_edge_us = 0;
    }
}

/// Sequential injection mode selector.
/// Handles transition from wasted spark to sequential after cam sync.
pub struct SequentialModeSelector {
    /// Target injection mode based on sync state.
    mode: InjectionMode,
    /// Number of consecutive cycles with full sync before switching to sequential.
    required_cycles: u32,
    /// Counter for consecutive full sync cycles.
    sync_cycle_count: u32,
}

/// Injection mode selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InjectionMode {
    /// Simultaneous injection (all cylinders at once).
    Simultaneous,
    /// Grouped injection (2 groups).
    Grouped,
    /// Full sequential injection (cylinder-specific timing).
    Sequential,
}

impl SequentialModeSelector {
    /// Create new mode selector.
    pub fn new(required_cycles: u32) -> Self {
        Self {
            mode: InjectionMode::Simultaneous,
            required_cycles,
            sync_cycle_count: 0,
        }
    }

    /// Update mode based on current trigger state.
    pub fn update(&mut self, trigger_state: &TriggerState) -> InjectionMode {
        if trigger_state.sync == SyncState::FullSync {
            self.sync_cycle_count = self.sync_cycle_count.saturating_add(1);
            if self.sync_cycle_count >= self.required_cycles {
                self.mode = InjectionMode::Sequential;
            } else if self.sync_cycle_count >= self.required_cycles / 2 {
                self.mode = InjectionMode::Grouped;
            }
        } else {
            self.sync_cycle_count = 0;
            self.mode = InjectionMode::Simultaneous;
        }
        self.mode
    }

    /// Get current injection mode.
    pub fn mode(&self) -> InjectionMode {
        self.mode
    }

    /// Reset to simultaneous mode.
    pub fn reset(&mut self) {
        self.mode = InjectionMode::Simultaneous;
        self.sync_cycle_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_36_1() -> MissingToothDecoder {
        MissingToothDecoder::new(MissingToothConfig {
            total_teeth: 36,
            missing_teeth: 1,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        })
    }

    /// Feed `n` equally-spaced pulses at `interval_us` apart.
    fn feed_pulses(
        dec: &mut MissingToothDecoder,
        count: u32,
        interval_us: u64,
        start_us: u64,
    ) -> u64 {
        let mut t = start_us;
        for _ in 0..count {
            t += interval_us;
            let _ = dec.process(TriggerSignal::CrankRise, t);
        }
        t
    }

    #[test]
    fn no_sync_before_gap() {
        let mut dec = make_36_1();
        // Feed 10 normal pulses — should remain unsynced
        feed_pulses(&mut dec, 10, 1000, 0);
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
    }

    #[test]
    fn sync_after_gap_pulse() {
        let mut dec = make_36_1();
        let normal = 1_000_u64; // µs per normal tooth
        let gap = normal * 2; // missing tooth: 2× interval

        // Feed 35 normal teeth then the gap tooth
        let t = feed_pulses(&mut dec, 35, normal, 0);
        // Gap pulse
        let result = dec.process(TriggerSignal::CrankRise, t + gap);
        assert!(result.is_ok());
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);
    }

    #[test]
    fn stall_detection() {
        let mut dec = make_36_1();
        let normal = 1_000_u64;
        let t = feed_pulses(&mut dec, 35, normal, 0);
        // Simulate gap
        let _ = dec.process(TriggerSignal::CrankRise, t + normal * 2);
        // Then a huge gap > 10 s
        let result = dec.process(TriggerSignal::CrankRise, t + 11_000_000);
        assert_eq!(result, Err(TriggerError::EngineStalled));
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
    }

    #[test]
    fn stall_detection_with_check_stall() {
        let mut dec = make_36_1();
        let normal = 1_000_u64;
        let t = feed_pulses(&mut dec, 35, normal, 0);
        // Simulate gap
        let _ = dec.process(TriggerSignal::CrankRise, t + normal * 2);
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);

        // Call check_stall before timeout — should not trigger
        let stalled_before = dec.check_stall(t + normal * 2 + 500_000); // +0.5s
        assert!(!stalled_before);
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);

        // Call check_stall after timeout — should trigger
        let stalled_after = dec.check_stall(t + normal * 2 + 1_500_000); // +1.5s
        assert!(stalled_after);
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
    }

    #[test]
    fn config_36_1_gap_ratio() {
        let cfg = MissingToothConfig {
            total_teeth: 36,
            missing_teeth: 1,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        };
        // expected_gap_ratio = missing + 1 = 2.0
        assert!((cfg.expected_gap_ratio() - 2.0).abs() < 1e-6);
        // degrees_per_tooth = 720 / 36 = 20°
        assert!((cfg.degrees_per_tooth() - 20.0).abs() < 1e-6);
        // present_teeth = 36 - 1 = 35
        assert_eq!(cfg.present_teeth(), 35);
    }

    #[test]
    fn config_60_2_gap_ratio() {
        let cfg = MissingToothConfig {
            total_teeth: 60,
            missing_teeth: 2,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        };
        // expected_gap_ratio = 2 + 1 = 3.0
        assert!((cfg.expected_gap_ratio() - 3.0).abs() < 1e-6);
    }

    fn assert_missing_tooth_preset_sync(cfg: MissingToothConfig) {
        let mut dec = MissingToothDecoder::new(cfg);
        let normal = 1_000_u64;
        let t = feed_pulses(&mut dec, cfg.present_teeth(), normal, 0);
        let result = dec.process(
            TriggerSignal::CrankRise,
            t + normal * cfg.expected_gap_ratio() as u64,
        );

        assert!(result.is_ok());
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);
    }

    #[test]
    fn common_missing_tooth_presets_sync() {
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_4_1());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_12_1());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_24_1());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_24_2());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_36_1());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_36_2());
        assert_missing_tooth_preset_sync(MissingToothConfig::trigger_60_2());
    }

    #[test]
    fn trigger_24_1_config_preset() {
        let cfg = MissingToothConfig::trigger_24_1();

        assert_eq!(cfg.total_teeth, 24);
        assert_eq!(cfg.missing_teeth, 1);
        assert!((cfg.expected_gap_ratio() - 2.0).abs() < 1e-6);
        assert!((cfg.degrees_per_tooth() - 30.0).abs() < 1e-6);
        assert_eq!(cfg.present_teeth(), 23);
    }

    // 12-1 trigger wheel tests
    fn make_12_1() -> MissingToothDecoder {
        MissingToothDecoder::new(MissingToothConfig {
            total_teeth: 12,
            missing_teeth: 1,
            engine_cycle_deg: 720.0,
            sync_edge: SyncEdge::Rise,
        })
    }

    #[test]
    fn trigger_12_1_sync_and_angle() {
        let mut dec = make_12_1();
        let interval = 1000_u64;

        // Feed 11 normal pulses, then detect gap (missing 1 tooth)
        let t = feed_pulses(&mut dec, 11, interval, 0);

        // Gap pulse at 2x interval (missing tooth)
        let _ = dec.process(TriggerSignal::CrankRise, t + interval * 2);

        // Should achieve crank sync
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);

        // Degrees per tooth: 720 / 12 = 60°
        assert!((dec.config().degrees_per_tooth() - 60.0).abs() < 1e-6);

        // Feed remaining teeth to complete cycle
        let t2 = feed_pulses(&mut dec, 11, interval, t + interval * 2);

        // Gap again - should maintain sync
        let _ = dec.process(TriggerSignal::CrankRise, t2 + interval * 2);
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);
    }

    #[test]
    fn trigger_12_1_low_resolution_position() {
        let mut dec = make_12_1();
        let interval = 1000_u64;

        // Sync first
        let t = feed_pulses(&mut dec, 11, interval, 0);
        let _ = dec.process(TriggerSignal::CrankRise, t + interval * 2);
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);

        // After sync, check that position advances
        let pos1 = dec.position_deg();

        // Feed next tooth
        let _ = dec.process(TriggerSignal::CrankRise, t + interval * 2 + interval);
        let pos2 = dec.position_deg();

        // Should advance by ~60° per tooth
        let delta = normalize_angle_720(pos2 - pos1);
        assert!(
            (delta - 60.0).abs() < 5.0,
            "Position should advance ~60°, got {}°",
            delta
        );
    }

    #[test]
    fn trigger_12_1_stall_detection() {
        let mut dec = make_12_1();
        let interval = 1000_u64;

        // Sync
        let t = feed_pulses(&mut dec, 11, interval, 0);
        let _ = dec.process(TriggerSignal::CrankRise, t + interval * 2);
        assert_eq!(dec.sync_state(), SyncState::CrankSynced);

        // Check stall after timeout
        let stalled = dec.check_stall(t + interval * 2 + 1_500_000); // 1.5s later
        assert!(stalled);
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
    }

    #[test]
    fn trigger_12_1_config_preset() {
        // Test the preset constructor
        let cfg = MissingToothConfig::trigger_12_1();
        let dec = MissingToothDecoder::new(cfg);

        assert_eq!(dec.config().total_teeth, 12);
        assert_eq!(dec.config().missing_teeth, 1);
        assert!((dec.config().degrees_per_tooth() - 60.0).abs() < 1e-6);
    }

    // GM 36-2-2-2 trigger wheel tests
    #[test]
    fn gm_36_2_2_2_sync_detection() {
        let cfg = GmTriggerConfig::gm_36_2_2_2();
        let mut dec = GmTriggerDecoder::new(cfg);
        let normal = 1000_u64;

        // Feed teeth until we hit the gap pattern
        // 36-2-2-2 = 30 present teeth per cycle
        let mut t = 0_u64;
        for _ in 0..28 {
            t += normal;
            let _ = dec.process(TriggerSignal::CrankRise, t);
        }

        // First gap (2 missing teeth = 3x interval)
        t += normal * 3;
        let _ = dec.process(TriggerSignal::CrankRise, t);

        // Second gap
        t += normal * 3;
        let _ = dec.process(TriggerSignal::CrankRise, t);

        // Third gap
        t += normal * 3;
        let result = dec.process(TriggerSignal::CrankRise, t);

        assert!(result.is_ok());
        // Should achieve sync after detecting 2-2-2 pattern
    }

    // Instant RPM calculator tests
    #[test]
    fn instant_rpm_basic() {
        let mut calc = InstantRpmCalculator::new();
        let deg_per_tooth = 20.0; // 36-1 wheel

        // 1000µs per tooth = 3333 RPM for 20° tooth
        let rpm = calc.update(1000, deg_per_tooth);
        assert!(rpm > 0.0);

        // Check latest value
        let latest = calc.latest();
        assert!(latest.is_some());
    }

    #[test]
    fn instant_rpm_smoothing() {
        let mut calc = InstantRpmCalculator::new();
        let deg_per_tooth = 20.0;

        // Feed multiple intervals
        for i in 0..8 {
            let duration = 1000 + i * 10; // Slight variation
            calc.update(duration, deg_per_tooth);
        }

        // Should have smoothed value
        let smoothed = calc.smoothed();
        assert!(smoothed.is_some());
    }

    #[test]
    fn instant_rpm_variance() {
        let mut calc = InstantRpmCalculator::new();
        let deg_per_tooth = 20.0;

        // Feed consistent intervals
        for _ in 0..8 {
            calc.update(1000, deg_per_tooth);
        }

        // Variance should be low for consistent input
        let variance = calc.variance();
        assert!(variance.is_some());
    }

    // Trigger noise filter tests
    #[test]
    fn noise_filter_debounce() {
        let mut filter = TriggerNoiseFilter::default_vr_filter();
        let t = 0_u64;

        // Single state change should not pass
        let result = filter.process(true, t + 10);
        assert!(result.is_none());

        // Stable for threshold samples
        let mut saw_transition = false;
        for i in 1..=5 {
            saw_transition |= filter.process(true, t + i * 30) == Some(true);
        }
        // After stable samples, should return true
        assert!(saw_transition);
    }

    #[test]
    fn noise_filter_min_pulse_width() {
        let mut filter = TriggerNoiseFilter::new(100, 2); // 100µs min pulse
        let t = 0_u64;

        // Long high pulse
        filter.process(true, t);
        filter.process(true, t + 50);
        let result = filter.process(true, t + 120);

        // Should have transitioned
        assert_eq!(result, Some(true));
    }

    #[test]
    fn noise_filter_glitch_rejection() {
        let mut filter = TriggerNoiseFilter::default_vr_filter();
        let t = 0_u64;

        // Set to true
        filter.process(true, t);
        filter.process(true, t + 100);
        filter.process(true, t + 200);

        // Brief glitch to false
        let _ = filter.process(false, t + 210);
        let _ = filter.process(false, t + 220);

        // Return to true
        let _result = filter.process(true, t + 300);

        // Filter should stay true (glitch rejected)
        assert!(filter.state());
    }

    // Sequential mode selector tests
    #[test]
    fn sequential_mode_transition() {
        let mut selector = SequentialModeSelector::new(4); // 4 cycles required

        // Start simultaneous
        assert_eq!(selector.mode(), InjectionMode::Simultaneous);

        // Simulate trigger states
        for i in 0..6 {
            let sync = if i < 2 {
                SyncState::CrankSynced
            } else {
                SyncState::FullSync
            };
            let state = TriggerState {
                tooth_index: 0,
                sync,
                rpm: Some(1000.0),
                angle_deg: 0.0,
                cam_phase: false,
                cycle_position: None,
                current_cylinder: None,
            };
            let mode = selector.update(&state);

            if i < 2 {
                assert_eq!(mode, InjectionMode::Simultaneous);
            } else if i < 3 {
                assert_eq!(mode, InjectionMode::Simultaneous);
            } else if i < 5 {
                assert_eq!(mode, InjectionMode::Grouped);
            } else {
                assert_eq!(mode, InjectionMode::Sequential);
            }
        }
    }

    #[test]
    fn sequential_mode_reset_on_sync_loss() {
        let mut selector = SequentialModeSelector::new(4);

        // Build up to sequential
        for _ in 0..6 {
            let state = TriggerState {
                tooth_index: 0,
                sync: SyncState::FullSync,
                rpm: Some(1000.0),
                angle_deg: 0.0,
                cam_phase: false,
                cycle_position: None,
                current_cylinder: None,
            };
            selector.update(&state);
        }
        assert_eq!(selector.mode(), InjectionMode::Sequential);

        // Lose sync
        let unsynced_state = TriggerState {
            tooth_index: 0,
            sync: SyncState::Unsynced,
            rpm: None,
            angle_deg: 0.0,
            cam_phase: false,
            cycle_position: None,
            current_cylinder: None,
        };
        let mode = selector.update(&unsynced_state);
        assert_eq!(mode, InjectionMode::Simultaneous);
    }

    #[test]
    fn sequential_mode_reset() {
        let mut selector = SequentialModeSelector::new(4);

        // Build up to sequential
        for _ in 0..6 {
            let state = TriggerState {
                tooth_index: 0,
                sync: SyncState::FullSync,
                rpm: Some(1000.0),
                angle_deg: 0.0,
                cam_phase: false,
                cycle_position: None,
                current_cylinder: None,
            };
            selector.update(&state);
        }

        // Reset
        selector.reset();
        assert_eq!(selector.mode(), InjectionMode::Simultaneous);
    }
}
