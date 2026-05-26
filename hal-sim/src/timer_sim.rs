//! Simulator system timer: monotonic microsecond clock backed by
//! `std::time::Instant`.

use rusefi_core::hal::{SystemTimer, TimerCallback};
use std::time::Instant;

/// Simulator implementation of [`SystemTimer`].
///
/// Provides a wall-clock-backed monotonic timer.
/// Scheduled callbacks are stored and fired synchronously on the next
/// [`SimSystemTimer::fire_pending`] call (suitable for single-threaded sim).
pub struct SimSystemTimer {
    start: Instant,
    pending: Option<(u64, TimerCallback)>,
}

impl SimSystemTimer {
    /// Create a new timer starting from now.
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
            pending: None,
        }
    }

    /// Fire any pending callback whose scheduled time has passed.
    ///
    /// Call this from the simulator main loop.
    pub fn fire_pending(&mut self) {
        if let Some((fire_at_us, cb)) = self.pending {
            if self.now_us() >= fire_at_us {
                self.pending = None;
                cb();
            }
        }
    }
}

impl Default for SimSystemTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl SystemTimer for SimSystemTimer {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }

    fn schedule_us(&mut self, delay_us: u64, callback: TimerCallback) {
        let fire_at = self.now_us() + delay_us;
        self.pending = Some((fire_at, callback));
    }
}
