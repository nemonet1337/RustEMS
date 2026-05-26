//! microRusEFI System Timer Implementation
//!
//! Uses embassy-time for monotonic microseconds and async delays.

use rusefi_core::hal::{SystemTimer, TimerCallback};
use embassy_time::{Duration, Instant};

/// microRusEFI System Timer using embassy-time.
pub struct Stm32SystemTimer;

impl Stm32SystemTimer {
    /// Create a new system timer.
    pub fn new() -> Self {
        Self
    }

    /// Sleep for the specified number of microseconds (async).
    pub async fn sleep_us(us: u64) {
        embassy_time::Timer::after(Duration::from_micros(us)).await;
    }

    /// Sleep for the specified number of milliseconds (async).
    pub async fn sleep_ms(ms: u64) {
        embassy_time::Timer::after(Duration::from_millis(ms)).await;
    }
}

impl Default for Stm32SystemTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl SystemTimer for Stm32SystemTimer {
    fn now_us(&self) -> u64 {
        // embassy-time provides monotonic time since boot
        Instant::now().as_micros() as u64
    }

    fn schedule_us(&mut self, delay_us: u64, callback: TimerCallback) {
        // Blocks the current executor task until the delay elapses, then fires
        // the callback. Use sleep_us() instead for async contexts.
        let deadline = embassy_time::Instant::now()
            + embassy_time::Duration::from_micros(delay_us);
        while embassy_time::Instant::now() < deadline {
            core::hint::spin_loop();
        }
        callback();
    }
}
