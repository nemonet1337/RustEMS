//! Engine shutdown controller.
//!
//! Manages safe engine shutdown sequences including:
//! - Key-off detection
//! - Emergency shutdown
//! - Fuel pump control during shutdown
//! - Cool-down procedures

use crate::sensors::SensorData;

/// Shutdown trigger reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShutdownReason {
    /// Key turned off (normal shutdown).
    KeyOff,
    /// Emergency stop triggered (manual or safety system).
    EmergencyStop,
    /// Critical protection condition (overheat, low oil pressure).
    CriticalProtection,
    /// Battery voltage too low to continue operation.
    LowBattery,
    /// Timeout (engine failed to start).
    Timeout,
}

/// Shutdown state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShutdownState {
    /// Normal operation - not shutting down.
    Running,
    /// Shutdown initiated, fuel pump running for prime/recovery.
    FuelPumpOn,
    /// Fuel pump off, engine coasting down.
    Coasting,
    /// Engine stopped, shutdown complete.
    Stopped,
}

/// Shutdown controller configuration.
#[derive(Clone, Copy, Debug)]
pub struct ShutdownConfig {
    /// Fuel pump run time after key-off (ms).
    pub fuel_pump_run_ms: u32,
    /// Minimum RPM to consider engine running.
    pub min_running_rpm: f32,
    /// Timeout for engine start (ms).
    pub start_timeout_ms: u32,
    /// Battery voltage threshold for shutdown (V).
    pub battery_shutdown_v: f32,
}

impl Default for ShutdownConfig {
    fn default() -> Self {
        Self {
            fuel_pump_run_ms: 2000, // 2 seconds
            min_running_rpm: 300.0,
            start_timeout_ms: 5000, // 5 seconds
            battery_shutdown_v: 8.0,
        }
    }
}

/// Shutdown controller.
pub struct ShutdownController {
    cfg: ShutdownConfig,
    state: ShutdownState,
    reason: Option<ShutdownReason>,
    fuel_pump_timer_ms: u32,
    uptime_ms: u32,
}

impl ShutdownController {
    /// Create a new shutdown controller with default configuration.
    pub fn new() -> Self {
        Self::with_config(ShutdownConfig::default())
    }

    /// Create a new shutdown controller with custom configuration.
    pub fn with_config(cfg: ShutdownConfig) -> Self {
        Self {
            cfg,
            state: ShutdownState::Running,
            reason: None,
            fuel_pump_timer_ms: 0,
            uptime_ms: 0,
        }
    }

    /// Initiate shutdown sequence.
    ///
    /// # Arguments
    /// * `reason` - Reason for shutdown
    pub fn shutdown(&mut self, reason: ShutdownReason) {
        if self.state == ShutdownState::Running {
            self.state = ShutdownState::FuelPumpOn;
            self.reason = Some(reason);
            self.fuel_pump_timer_ms = self.cfg.fuel_pump_run_ms;
        }
    }

    /// Update shutdown state based on sensor data and time.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor data
    /// * `dt_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// `true` if fuel pump should be energized.
    pub fn update(&mut self, sensors: &SensorData, dt_ms: u32) -> bool {
        self.uptime_ms += dt_ms;

        match self.state {
            ShutdownState::Running => {
                // Check for automatic shutdown conditions
                if let Some(batt) = sensors.battery_volts {
                    if batt < self.cfg.battery_shutdown_v {
                        self.shutdown(ShutdownReason::LowBattery);
                    }
                }

                // Check start timeout
                if self.uptime_ms > self.cfg.start_timeout_ms {
                    if let Some(rpm) = sensors.rpm {
                        if rpm < self.cfg.min_running_rpm {
                            self.shutdown(ShutdownReason::Timeout);
                        }
                    }
                }

                // In running state, fuel pump is controlled by FuelPumpController
                false
            }
            ShutdownState::FuelPumpOn => {
                self.fuel_pump_timer_ms = self.fuel_pump_timer_ms.saturating_sub(dt_ms);

                if self.fuel_pump_timer_ms == 0 {
                    self.state = ShutdownState::Coasting;
                    false // Timer expired: fuel pump off
                } else {
                    true // Keep fuel pump on
                }
            }
            ShutdownState::Coasting => {
                // Wait for engine to stop
                if let Some(rpm) = sensors.rpm {
                    if rpm < self.cfg.min_running_rpm {
                        self.state = ShutdownState::Stopped;
                    }
                } else {
                    self.state = ShutdownState::Stopped;
                }

                false // Fuel pump off
            }
            ShutdownState::Stopped => {
                false // Everything off
            }
        }
    }

    /// Get current shutdown state.
    pub fn state(&self) -> ShutdownState {
        self.state
    }

    /// Get shutdown reason if shutdown was triggered.
    pub fn reason(&self) -> Option<ShutdownReason> {
        self.reason
    }

    /// Check if shutdown is in progress.
    pub fn is_shutting_down(&self) -> bool {
        self.state != ShutdownState::Running
    }

    /// Check if engine is stopped.
    pub fn is_stopped(&self) -> bool {
        self.state == ShutdownState::Stopped
    }

    /// Reset to running state.
    pub fn reset(&mut self) {
        self.state = ShutdownState::Running;
        self.reason = None;
        self.fuel_pump_timer_ms = 0;
        self.uptime_ms = 0;
    }

    /// Get the configuration.
    pub fn config(&self) -> ShutdownConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: ShutdownConfig) {
        self.cfg = cfg;
    }
}

impl Default for ShutdownController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shutdown_initiates_sequence() {
        let mut controller = ShutdownController::new();
        let sensors = SensorData::default();

        controller.shutdown(ShutdownReason::KeyOff);
        let pump_on = controller.update(&sensors, 10);

        assert_eq!(controller.state(), ShutdownState::FuelPumpOn);
        assert!(pump_on);
        assert_eq!(controller.reason(), Some(ShutdownReason::KeyOff));
    }

    #[test]
    fn fuel_pump_runs_then_off() {
        let mut controller = ShutdownController::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(1000.0);

        controller.shutdown(ShutdownReason::KeyOff);

        // Pump should be on initially
        let pump_on = controller.update(&sensors, 10);
        assert!(pump_on);

        // After timer expires, pump should turn off
        let mut controller = ShutdownController::new();
        controller.shutdown(ShutdownReason::KeyOff);
        let pump_on = controller.update(&sensors, 2100); // 2.1 seconds > 2 second timeout
        assert!(!pump_on);
    }

    #[test]
    fn low_battery_triggers_shutdown() {
        let mut controller = ShutdownController::new();
        let mut sensors = SensorData::default();
        sensors.battery_volts = Some(7.0); // Below 8V threshold

        controller.update(&sensors, 10);

        assert!(controller.is_shutting_down());
        assert_eq!(controller.reason(), Some(ShutdownReason::LowBattery));
    }

    #[test]
    fn start_timeout_triggers_shutdown() {
        let mut controller = ShutdownController::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(200.0); // Below 300 RPM threshold

        // Simulate 6 seconds of operation with low RPM
        for _ in 0..600 {
            controller.update(&sensors, 10);
        }

        assert!(controller.is_shutting_down());
        assert_eq!(controller.reason(), Some(ShutdownReason::Timeout));
    }

    #[test]
    fn reset_clears_shutdown() {
        let mut controller = ShutdownController::new();
        let sensors = SensorData::default();

        controller.shutdown(ShutdownReason::KeyOff);
        controller.update(&sensors, 10);

        controller.reset();

        assert_eq!(controller.state(), ShutdownState::Running);
        assert!(!controller.is_shutting_down());
        assert!(controller.reason().is_none());
    }

    #[test]
    fn normal_operation_no_shutdown() {
        let mut controller = ShutdownController::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(1000.0);
        sensors.battery_volts = Some(12.0);

        let pump_on = controller.update(&sensors, 10);

        assert_eq!(controller.state(), ShutdownState::Running);
        assert!(!pump_on);
    }
}
