//! Simulator trigger input: replays a CSV trigger log or generates synthetic
//! missing-tooth pulses at a given RPM.

use rusefi_core::hal::TriggerInput;
use std::collections::VecDeque;

/// A trigger event sourced from a CSV replay or synthetic generator.
#[derive(Clone, Copy, Debug)]
pub struct TriggerEvent {
    /// Timestamp in microseconds.
    pub timestamp_us: u64,
    /// True = crank pulse, False = cam pulse.
    pub is_crank: bool,
}

/// Simulator implementation of [`TriggerInput`].
///
/// Events are queued externally (e.g. from CSV replay or synthetic generator)
/// and consumed one at a time via the HAL trait.
pub struct SimTriggerInput {
    crank_queue: VecDeque<u64>,
    cam_queue: VecDeque<u64>,
}

impl SimTriggerInput {
    /// Create an empty trigger input.
    pub fn new() -> Self {
        Self {
            crank_queue: VecDeque::new(),
            cam_queue: VecDeque::new(),
        }
    }

    /// Enqueue a crank pulse timestamp (µs).
    pub fn push_crank(&mut self, timestamp_us: u64) {
        self.crank_queue.push_back(timestamp_us);
    }

    /// Enqueue a cam pulse timestamp (µs).
    pub fn push_cam(&mut self, timestamp_us: u64) {
        self.cam_queue.push_back(timestamp_us);
    }

    /// Generate synthetic missing-tooth crank pulses for `cycles` engine cycles
    /// at `rpm`.
    ///
    /// # Arguments
    /// * `total_teeth`   — nominal wheel tooth count (e.g. 36)
    /// * `missing_teeth` — missing teeth count (e.g. 1)
    /// * `rpm`           — engine speed for the generated pulses
    /// * `cycles`        — number of 720° engine cycles to generate
    /// * `start_us`      — starting timestamp in µs
    pub fn generate_missing_tooth(
        &mut self,
        total_teeth: u32,
        missing_teeth: u32,
        rpm: f32,
        cycles: u32,
        start_us: u64,
    ) {
        let present_teeth = total_teeth - missing_teeth;
        // Duration of one engine cycle (720°) in µs
        let cycle_us = 120_000_000.0 / rpm; // 60 s / rpm * 2 revs * 1e6 µs/s
        let normal_tooth_us = (cycle_us / total_teeth as f32) as u64;
        let gap_tooth_us = normal_tooth_us * (missing_teeth + 1) as u64;

        let mut t = start_us;
        for _ in 0..cycles {
            // Normal teeth
            for _tooth in 0..present_teeth {
                t += normal_tooth_us;
                self.push_crank(t);
            }
            // Gap (missing teeth represented as one long interval)
            t += gap_tooth_us;
            self.push_crank(t);
        }
    }
}

impl Default for SimTriggerInput {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerInput for SimTriggerInput {
    fn read_crank_timestamp(&mut self) -> Option<u64> {
        self.crank_queue.pop_front()
    }

    fn read_cam_timestamp(&mut self) -> Option<u64> {
        self.cam_queue.pop_front()
    }
}
