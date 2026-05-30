//! Engine start/stop controller.
//!
//! Manages engine start and stop sequences including:
//! - Cranking detection
//! - Start success/failure detection
//! - Auto stop/start functionality
//! - Starter motor control

use crate::sensors::SensorData;

/// Start state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartState {
    /// Engine not running, ready to start.
    Stopped,
    /// Cranking in progress (starter motor engaged).
    Cranking,
    /// Engine running, starter disengaged.
    Running,
    /// Start failed (cranking timeout without engine start).
    StartFailed,
    /// Stopping in progress.
    Stopping,
}

/// Start failure reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartFailureReason {
    /// Cranking timeout exceeded.
    Timeout,
    /// No RPM detected during cranking.
    NoRpm,
    /// Sensor failure preventing start.
    SensorFailure,
}

/// Start controller configuration.
#[derive(Clone, Copy, Debug)]
pub struct StartConfig {
    /// Maximum cranking duration (ms).
    pub crank_timeout_ms: u32,
    /// RPM threshold to consider engine started.
    pub start_rpm_threshold: f32,
    /// Minimum RPM to consider engine stopped.
    pub stop_rpm_threshold: f32,
    /// Time to wait before retrying start (ms).
    pub retry_delay_ms: u32,
}

impl Default for StartConfig {
    fn default() -> Self {
        Self {
            crank_timeout_ms: 5000, // 5 seconds
            start_rpm_threshold: 400.0,
            stop_rpm_threshold: 300.0,
            retry_delay_ms: 2000, // 2 seconds
        }
    }
}

/// Start/stop controller.
pub struct StartStopController {
    cfg: StartConfig,
    state: StartState,
    crank_timer_ms: u32,
    failure_reason: Option<StartFailureReason>,
    retry_timer_ms: u32,
}

impl StartStopController {
    /// Create a new start/stop controller with default configuration.
    pub fn new() -> Self {
        Self::with_config(StartConfig::default())
    }

    /// Create a new start/stop controller with custom configuration.
    pub fn with_config(cfg: StartConfig) -> Self {
        Self {
            cfg,
            state: StartState::Stopped,
            crank_timer_ms: 0,
            failure_reason: None,
            retry_timer_ms: 0,
        }
    }

    /// Initiate engine start sequence.
    pub fn start(&mut self) {
        if self.state == StartState::Stopped || self.state == StartState::StartFailed {
            self.state = StartState::Cranking;
            self.crank_timer_ms = self.cfg.crank_timeout_ms;
            self.failure_reason = None;
            self.retry_timer_ms = 0;
        }
    }

    /// Initiate engine stop sequence.
    pub fn stop(&mut self) {
        if self.state == StartState::Running {
            self.state = StartState::Stopping;
        }
    }

    /// Update start/stop state based on sensor data.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor data
    /// * `dt_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// `true` if starter motor should be engaged.
    pub fn update(&mut self, sensors: &SensorData, dt_ms: u32) -> bool {
        let rpm = sensors.rpm.unwrap_or(0.0);

        match self.state {
            StartState::Stopped => {
                // Handle retry delay after failed start
                if let Some(_reason) = self.failure_reason {
                    self.retry_timer_ms = self.retry_timer_ms.saturating_sub(dt_ms);
                    if self.retry_timer_ms == 0 {
                        self.failure_reason = None;
                    }
                }
                false
            }
            StartState::Cranking => {
                self.crank_timer_ms = self.crank_timer_ms.saturating_sub(dt_ms);

                // Check if engine started
                if rpm >= self.cfg.start_rpm_threshold {
                    self.state = StartState::Running;
                    return false;
                }

                // Check for timeout
                if self.crank_timer_ms == 0 {
                    self.state = StartState::StartFailed;
                    self.failure_reason = Some(StartFailureReason::Timeout);
                    self.retry_timer_ms = self.cfg.retry_delay_ms;
                    return false;
                }

                // Check for no RPM during cranking
                if rpm < 50.0 && self.crank_timer_ms < (self.cfg.crank_timeout_ms - 1000) {
                    self.state = StartState::StartFailed;
                    self.failure_reason = Some(StartFailureReason::NoRpm);
                    self.retry_timer_ms = self.cfg.retry_delay_ms;
                    return false;
                }

                true // Keep cranking
            }
            StartState::Running => {
                // Check if engine stopped
                if rpm < self.cfg.stop_rpm_threshold {
                    self.state = StartState::Stopped;
                }
                false
            }
            StartState::StartFailed => {
                // Remain in the failed state and count down the retry delay so
                // the engine cannot be immediately re-cranked. The state is left
                // only via an explicit start() or reset().
                self.retry_timer_ms = self.retry_timer_ms.saturating_sub(dt_ms);
                false
            }
            StartState::Stopping => {
                // Wait for engine to stop
                if rpm < self.cfg.stop_rpm_threshold {
                    self.state = StartState::Stopped;
                }
                false
            }
        }
    }

    /// Get current start state.
    pub fn state(&self) -> StartState {
        self.state
    }

    /// Get start failure reason if start failed.
    pub fn failure_reason(&self) -> Option<StartFailureReason> {
        self.failure_reason
    }

    /// Check if engine is running.
    pub fn is_running(&self) -> bool {
        self.state == StartState::Running
    }

    /// Check if cranking is in progress.
    pub fn is_cranking(&self) -> bool {
        self.state == StartState::Cranking
    }

    /// Check if start is failed.
    pub fn is_failed(&self) -> bool {
        self.state == StartState::StartFailed
    }

    /// Check if ready to retry start.
    pub fn is_ready_to_retry(&self) -> bool {
        self.state == StartState::StartFailed && self.retry_timer_ms == 0
    }

    /// Reset to stopped state.
    pub fn reset(&mut self) {
        self.state = StartState::Stopped;
        self.crank_timer_ms = 0;
        self.failure_reason = None;
        self.retry_timer_ms = 0;
    }

    /// Get the configuration.
    pub fn config(&self) -> StartConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: StartConfig) {
        self.cfg = cfg;
    }
}

impl Default for StartStopController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_initiates_cranking() {
        let mut controller = StartStopController::new();
        let sensors = SensorData::default();

        controller.start();
        let cranking = controller.update(&sensors, 10);

        assert_eq!(controller.state(), StartState::Cranking);
        assert!(cranking);
    }

    #[test]
    fn engine_starts_successfully() {
        let mut controller = StartStopController::new();
        let sensors = SensorData {
            rpm: Some(500.0), // Above 400 RPM threshold
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        controller.start();
        controller.update(&sensors, 10);

        assert_eq!(controller.state(), StartState::Running);
        assert!(!controller.is_cranking());
    }

    #[test]
    fn crank_timeout_triggers_failure() {
        let mut controller = StartStopController::new();
        let sensors = SensorData {
            rpm: Some(300.0), // Below 400 RPM threshold
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        controller.start();

        // Simulate 5.1 seconds of cranking (exceeds 5 second timeout)
        for _ in 0..510 {
            controller.update(&sensors, 10);
        }

        assert_eq!(controller.state(), StartState::StartFailed);
        assert_eq!(controller.failure_reason(), Some(StartFailureReason::Timeout));
    }

    #[test]
    fn no_rpm_during_crank_fails() {
        let mut controller = StartStopController::new();
        let sensors = SensorData {
            rpm: Some(0.0), // No RPM
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        controller.start();

        // Simulate 4.1 seconds of cranking (after 4 seconds, no RPM check)
        for _ in 0..410 {
            controller.update(&sensors, 10);
        }

        assert_eq!(controller.state(), StartState::StartFailed);
        assert_eq!(controller.failure_reason(), Some(StartFailureReason::NoRpm));
    }

    #[test]
    fn stop_transitions_to_stopped() {
        let mut controller = StartStopController::new();
        let sensors_running = SensorData {
            rpm: Some(500.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };
        let sensors_stopped = SensorData {
            rpm: Some(200.0), // Below 300 RPM threshold
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        controller.start();
        controller.update(&sensors_running, 10); // Engine running

        controller.stop();
        controller.update(&sensors_stopped, 10);

        assert_eq!(controller.state(), StartState::Stopped);
    }

    #[test]
    fn retry_delay_prevents_immediate_restart() {
        let mut controller = StartStopController::new();
        let sensors = SensorData {
            rpm: Some(300.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        // Fail start
        controller.start();
        for _ in 0..510 {
            controller.update(&sensors, 10);
        }

        assert!(!controller.is_ready_to_retry());

        // Wait for retry delay
        for _ in 0..210 {
            controller.update(&sensors, 10);
        }

        assert!(controller.is_ready_to_retry());
    }

    #[test]
    fn reset_clears_failure() {
        let mut controller = StartStopController::new();
        let sensors = SensorData {
            rpm: Some(300.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        controller.start();
        for _ in 0..510 {
            controller.update(&sensors, 10);
        }

        controller.reset();

        assert_eq!(controller.state(), StartState::Stopped);
        assert!(controller.failure_reason().is_none());
    }

    #[test]
    fn normal_operation_no_start() {
        let mut controller = StartStopController::new();
        let sensors = SensorData::default();

        let cranking = controller.update(&sensors, 10);

        assert_eq!(controller.state(), StartState::Stopped);
        assert!(!cranking);
    }
}
