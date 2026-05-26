//! Engine protection and limp mode.
//!
//! Monitors critical sensors and activates limp mode when
//! dangerous conditions are detected (overheating, low oil pressure, etc.).

use crate::sensors::SensorData;

/// Limp mode state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LimpModeState {
    /// Normal operation - no protection active.
    Normal,
    /// Warning condition - limit RPM but still driveable.
    Warning,
    /// Critical condition - severe RPM limit, reduced power.
    Critical,
    /// Emergency - engine should be shut down.
    Emergency,
}

/// Protection condition type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProtectionCondition {
    /// Coolant temperature too high.
    Overheat,
    /// Oil pressure too low.
    LowOilPressure,
    /// Intake air temperature too high.
    OverheatIat,
    /// Battery voltage too low.
    LowBattery,
    /// Sensor failure.
    SensorFailure,
}

/// Protection event with details.
#[derive(Clone, Copy, Debug)]
pub struct ProtectionEvent {
    /// Condition type.
    pub condition: ProtectionCondition,
    /// Current value that triggered the condition.
    pub current_value: f32,
    /// Threshold value.
    pub threshold: f32,
    /// Time when condition was detected (ms since boot).
    pub timestamp_ms: u32,
}

/// Engine protection configuration.
#[derive(Clone, Copy, Debug)]
pub struct ProtectionConfig {
    /// Coolant temperature threshold for warning (°C).
    pub clt_warning_c: f32,
    /// Coolant temperature threshold for critical (°C).
    pub clt_critical_c: f32,
    /// Coolant temperature threshold for emergency (°C).
    pub clt_emergency_c: f32,

    /// Oil pressure threshold for warning (kPa).
    pub oil_warning_kpa: f32,
    /// Oil pressure threshold for critical (kPa).
    pub oil_critical_kpa: f32,
    /// Oil pressure threshold for emergency (kPa).
    pub oil_emergency_kpa: f32,

    /// IAT temperature threshold for warning (°C).
    pub iat_warning_c: f32,
    /// IAT temperature threshold for critical (°C).
    pub iat_critical_c: f32,

    /// Battery voltage threshold for warning (V).
    pub battery_warning_v: f32,
    /// Battery voltage threshold for critical (V).
    pub battery_critical_v: f32,

    /// RPM limit during warning mode.
    pub rpm_limit_warning: f32,
    /// RPM limit during critical mode.
    pub rpm_limit_critical: f32,
}

impl Default for ProtectionConfig {
    fn default() -> Self {
        Self {
            clt_warning_c: 105.0,
            clt_critical_c: 115.0,
            clt_emergency_c: 125.0,
            oil_warning_kpa: 100.0,
            oil_critical_kpa: 50.0,
            oil_emergency_kpa: 20.0,
            iat_warning_c: 80.0,
            iat_critical_c: 100.0,
            battery_warning_v: 11.0,
            battery_critical_v: 9.0,
            rpm_limit_warning: 4000.0,
            rpm_limit_critical: 2500.0,
        }
    }
}

/// Engine protection monitor.
pub struct ProtectionMonitor {
    cfg: ProtectionConfig,
    state: LimpModeState,
    current_rpm_limit: f32,
    active_conditions: [Option<ProtectionCondition>; 8],
    uptime_ms: u32,
}

impl ProtectionMonitor {
    /// Create a new protection monitor with default configuration.
    pub fn new() -> Self {
        Self::with_config(ProtectionConfig::default())
    }

    /// Create a new protection monitor with custom configuration.
    pub fn with_config(cfg: ProtectionConfig) -> Self {
        Self {
            cfg,
            state: LimpModeState::Normal,
            current_rpm_limit: f32::MAX,
            active_conditions: [None; 8],
            uptime_ms: 0,
        }
    }

    /// Update protection state based on sensor readings.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor data
    /// * `dt_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// Current RPM limit (f32::MAX = no limit).
    pub fn update(&mut self, sensors: &SensorData, dt_ms: u32) -> f32 {
        self.uptime_ms += dt_ms;
        let mut max_severity = LimpModeState::Normal;
        let mut condition_idx = 0;

        // Check coolant temperature
        if let Some(clt) = sensors.clt_celsius {
            if clt >= self.cfg.clt_emergency_c {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Emergency);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::Overheat);
                condition_idx += 1;
            } else if clt >= self.cfg.clt_critical_c {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Critical);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::Overheat);
                condition_idx += 1;
            } else if clt >= self.cfg.clt_warning_c {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Warning);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::Overheat);
                condition_idx += 1;
            }
        }

        // Check oil pressure
        if let Some(oil) = sensors.oil_pressure_kpa {
            if oil <= self.cfg.oil_emergency_kpa {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Emergency);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::LowOilPressure);
                condition_idx += 1;
            } else if oil <= self.cfg.oil_critical_kpa {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Critical);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::LowOilPressure);
                condition_idx += 1;
            } else if oil <= self.cfg.oil_warning_kpa {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Warning);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::LowOilPressure);
                condition_idx += 1;
            }
        }

        // Check IAT
        if let Some(iat) = sensors.iat_celsius {
            if iat >= self.cfg.iat_critical_c {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Critical);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::OverheatIat);
                condition_idx += 1;
            } else if iat >= self.cfg.iat_warning_c {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Warning);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::OverheatIat);
                condition_idx += 1;
            }
        }

        // Check battery voltage
        if let Some(batt) = sensors.battery_volts {
            if batt <= self.cfg.battery_critical_v {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Critical);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::LowBattery);
                condition_idx += 1;
            } else if batt <= self.cfg.battery_warning_v {
                max_severity = LimpModeState::max(max_severity, LimpModeState::Warning);
                self.active_conditions[condition_idx] = Some(ProtectionCondition::LowBattery);
                condition_idx += 1;
            }
        }

        // Clear remaining condition slots
        for i in condition_idx..8 {
            self.active_conditions[i] = None;
        }

        self.state = max_severity;

        // Set RPM limit based on state
        self.current_rpm_limit = match self.state {
            LimpModeState::Normal => f32::MAX,
            LimpModeState::Warning => self.cfg.rpm_limit_warning,
            LimpModeState::Critical => self.cfg.rpm_limit_critical,
            LimpModeState::Emergency => 0.0, // Engine should stop
        };

        self.current_rpm_limit
    }

    /// Get current limp mode state.
    pub fn state(&self) -> LimpModeState {
        self.state
    }

    /// Get current RPM limit.
    pub fn rpm_limit(&self) -> f32 {
        self.current_rpm_limit
    }

    /// Get active protection conditions.
    pub fn active_conditions(&self) -> &[Option<ProtectionCondition>; 8] {
        &self.active_conditions
    }

    /// Check if protection is active (not in normal mode).
    pub fn is_protection_active(&self) -> bool {
        self.state != LimpModeState::Normal
    }

    /// Get the configuration.
    pub fn config(&self) -> ProtectionConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: ProtectionConfig) {
        self.cfg = cfg;
    }

    /// Reset to normal operation.
    pub fn reset(&mut self) {
        self.state = LimpModeState::Normal;
        self.current_rpm_limit = f32::MAX;
        self.active_conditions = [None; 8];
    }
}

impl Default for ProtectionMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl LimpModeState {
    /// Compare two states and return the more severe one.
    fn max(a: Self, b: Self) -> Self {
        match (a, b) {
            (Self::Emergency, _) | (_, Self::Emergency) => Self::Emergency,
            (Self::Critical, _) | (_, Self::Critical) => Self::Critical,
            (Self::Warning, _) | (_, Self::Warning) => Self::Warning,
            _ => Self::Normal,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limp_mode_severity_order() {
        assert_eq!(
            LimpModeState::max(LimpModeState::Normal, LimpModeState::Warning),
            LimpModeState::Warning
        );
        assert_eq!(
            LimpModeState::max(LimpModeState::Warning, LimpModeState::Critical),
            LimpModeState::Critical
        );
        assert_eq!(
            LimpModeState::max(LimpModeState::Critical, LimpModeState::Emergency),
            LimpModeState::Emergency
        );
    }

    #[test]
    fn coolant_overheat_triggers_limp() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.clt_celsius = Some(110.0); // Above warning (105°C)

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Warning);
        assert_eq!(monitor.rpm_limit(), 4000.0);
    }

    #[test]
    fn critical_overheat_reduces_rpm() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.clt_celsius = Some(120.0); // Above critical (115°C)

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Critical);
        assert_eq!(monitor.rpm_limit(), 2500.0);
    }

    #[test]
    fn emergency_overheat_stops_engine() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.clt_celsius = Some(130.0); // Above emergency (125°C)

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Emergency);
        assert_eq!(monitor.rpm_limit(), 0.0);
    }

    #[test]
    fn low_oil_pressure_triggers_protection() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.oil_pressure_kpa = Some(80.0); // Below warning (100 kPa)

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Warning);
    }

    #[test]
    fn normal_operation_no_limit() {
        let mut monitor = ProtectionMonitor::new();
        let sensors = SensorData::default(); // All None = normal

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Normal);
        assert_eq!(monitor.rpm_limit(), f32::MAX);
    }

    #[test]
    fn reset_clears_protection() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.clt_celsius = Some(110.0);

        monitor.update(&sensors, 10);
        assert!(monitor.is_protection_active());

        monitor.reset();
        assert_eq!(monitor.state(), LimpModeState::Normal);
        assert!(!monitor.is_protection_active());
    }

    #[test]
    fn low_battery_triggers_warning() {
        let mut monitor = ProtectionMonitor::new();
        let mut sensors = SensorData::default();
        sensors.battery_volts = Some(10.5); // Below warning (11V)

        monitor.update(&sensors, 10);
        assert_eq!(monitor.state(), LimpModeState::Warning);
    }
}
