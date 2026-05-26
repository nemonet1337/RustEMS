//! Knock sensing — software-based knock detection.
//!
//! Derived from `firmware/controllers/knock/knock.cpp`.
//!
//! Software knock detection uses the knock sensor signal (typically from a piezoelectric
//! sensor) to detect engine knock by analyzing signal intensity during the knock window.

use crate::maps::interpolation::interpolate1d;

/// Knock detection configuration.
#[derive(Clone, Copy, Debug)]
pub struct KnockConfig {
    /// Enable knock detection.
    pub enabled: bool,
    /// Knock detection threshold (arbitrary units).
    pub threshold: f32,
    /// Knock window start angle (degrees after TDC).
    pub window_start_deg: f32,
    /// Knock window end angle (degrees after TDC).
    pub window_end_deg: f32,
    /// Maximum knock retard (degrees).
    pub max_retard: f32,
    /// Knock retard recovery rate (degrees per second).
    pub recovery_rate: f32,
    /// Minimum RPM for knock detection.
    pub min_rpm: f32,
    /// Maximum RPM for knock detection.
    pub max_rpm: f32,
    /// Knock threshold table (RPM axis).
    pub threshold_rpm_bins: [f32; 8],
    /// Knock threshold table (threshold at each RPM).
    pub threshold_table: [f32; 8],
}

impl Default for KnockConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            threshold: 50.0,
            window_start_deg: 10.0,
            window_end_deg: 50.0,
            max_retard: 15.0,
            recovery_rate: 1.0,
            min_rpm: 1000.0,
            max_rpm: 7000.0,
            threshold_rpm_bins: [1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0],
            threshold_table: [30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0],
        }
    }
}

/// Knock detection controller.
pub struct KnockController {
    cfg: KnockConfig,
    /// Current knock retard (degrees).
    current_retard: f32,
    /// Knock count since last reset.
    knock_count: u32,
    /// Maximum knock intensity observed.
    max_intensity: f32,
}

impl KnockController {
    /// Create new knock controller with configuration.
    pub fn new(cfg: KnockConfig) -> Self {
        Self {
            cfg,
            current_retard: 0.0,
            knock_count: 0,
            max_intensity: 0.0,
        }
    }

    /// Update knock detection.
    ///
    /// # Arguments
    /// * `rpm` — Current engine RPM
    /// * `crank_angle_deg` — Current crank angle (degrees after TDC)
    /// * `sensor_intensity` — Knock sensor signal intensity (arbitrary units)
    /// * `dt_s` — Time since last update in seconds
    ///
    /// # Returns
    /// Recommended ignition timing retard in degrees (positive = retard)
    pub fn update(&mut self, rpm: f32, crank_angle_deg: f32, sensor_intensity: f32, dt_s: f32) -> f32 {
        if !self.cfg.enabled {
            self.current_retard = 0.0;
            return 0.0;
        }

        // Check RPM range
        if rpm < self.cfg.min_rpm || rpm > self.cfg.max_rpm {
            self.current_retard = 0.0;
            return 0.0;
        }

        // Check if within knock window
        let in_window = crank_angle_deg >= self.cfg.window_start_deg && crank_angle_deg <= self.cfg.window_end_deg;

        if in_window {
            // Get threshold from table based on RPM
            let threshold = interpolate1d(&self.cfg.threshold_rpm_bins, &self.cfg.threshold_table, rpm);
            
            // Detect knock
            if sensor_intensity > threshold {
                self.knock_count += 1;
                self.max_intensity = self.max_intensity.max(sensor_intensity);

                // Apply retard based on intensity
                let intensity_ratio = (sensor_intensity - threshold) / threshold;
                let retard_step = intensity_ratio * 2.0; // 2 degrees per threshold unit
                self.current_retard = (self.current_retard + retard_step).min(self.cfg.max_retard);
            }
        }

        // Recover retard gradually
        if self.current_retard > 0.0 {
            let recovery = self.cfg.recovery_rate * dt_s;
            self.current_retard = (self.current_retard - recovery).max(0.0);
        }

        self.current_retard
    }

    /// Current knock retard (degrees).
    pub fn current_retard(&self) -> f32 {
        self.current_retard
    }

    /// Knock count since last reset.
    pub fn knock_count(&self) -> u32 {
        self.knock_count
    }

    /// Maximum knock intensity observed.
    pub fn max_intensity(&self) -> f32 {
        self.max_intensity
    }

    /// Reset knock detection state.
    pub fn reset(&mut self) {
        self.current_retard = 0.0;
        self.knock_count = 0;
        self.max_intensity = 0.0;
    }

    /// Get configuration reference.
    pub fn config(&self) -> &KnockConfig {
        &self.cfg
    }

    /// Update configuration.
    pub fn set_config(&mut self, cfg: KnockConfig) {
        self.cfg = cfg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn knock_disabled_returns_zero() {
        let cfg = KnockConfig::default();
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(3000.0, 30.0, 100.0, 0.01);
        assert_eq!(retard, 0.0);
    }

    #[test]
    fn knock_below_min_rpm() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(500.0, 30.0, 100.0, 0.01);
        assert_eq!(retard, 0.0);
    }

    #[test]
    fn knock_above_max_rpm() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(8000.0, 30.0, 100.0, 0.01);
        assert_eq!(retard, 0.0);
    }

    #[test]
    fn knock_detected_in_window() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(3000.0, 30.0, 100.0, 0.01);
        assert!(retard > 0.0);
        assert_eq!(ctrl.knock_count(), 1);
    }

    #[test]
    fn knock_not_detected_outside_window() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(3000.0, 60.0, 100.0, 0.01);
        assert_eq!(retard, 0.0);
        assert_eq!(ctrl.knock_count(), 0);
    }

    #[test]
    fn knock_below_threshold() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let retard = ctrl.update(3000.0, 30.0, 30.0, 0.01);
        assert_eq!(retard, 0.0);
    }

    #[test]
    fn knock_retard_limited() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        cfg.max_retard = 10.0;
        let mut ctrl = KnockController::new(cfg);
        
        // Very high intensity should be limited
        let retard = ctrl.update(3000.0, 30.0, 500.0, 0.01);
        assert!(retard <= cfg.max_retard);
    }

    #[test]
    fn knock_retard_recovers() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        cfg.recovery_rate = 10.0; // Fast recovery for test
        let mut ctrl = KnockController::new(cfg);
        
        // Apply knock retard
        let _ = ctrl.update(3000.0, 30.0, 100.0, 0.01);
        let retard_after = ctrl.current_retard();
        
        // Recover
        let _ = ctrl.update(3000.0, 30.0, 0.0, 0.1);
        let retard_recovered = ctrl.current_retard();
        
        assert!(retard_recovered < retard_after);
    }

    #[test]
    fn knock_reset_clears_state() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        let mut ctrl = KnockController::new(cfg);
        
        let _ = ctrl.update(3000.0, 30.0, 100.0, 0.01);
        assert!(ctrl.knock_count() > 0);
        assert!(ctrl.current_retard() > 0.0);
        
        ctrl.reset();
        assert_eq!(ctrl.knock_count(), 0);
        assert_eq!(ctrl.current_retard(), 0.0);
    }

    #[test]
    fn knock_threshold_table_used() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        cfg.threshold_rpm_bins = [1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0];
        cfg.threshold_table = [20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0];
        let mut ctrl = KnockController::new(cfg);
        
        // At 3000 RPM, threshold should be 30.0
        let retard = ctrl.update(3000.0, 30.0, 40.0, 0.01);
        assert!(retard > 0.0, "Should detect knock at 40.0 > 30.0 threshold");
    }

    #[test]
    fn knock_recovery_logic_works() {
        let mut cfg = KnockConfig::default();
        cfg.enabled = true;
        cfg.max_retard = 10.0;
        cfg.recovery_rate = 5.0; // 5 degrees per second
        let mut ctrl = KnockController::new(cfg);
        
        // Apply maximum retard
        let _ = ctrl.update(3000.0, 30.0, 200.0, 0.01);
        assert_eq!(ctrl.current_retard(), 10.0);
        
        // Recover over 1 second
        let _ = ctrl.update(3000.0, 30.0, 0.0, 1.0);
        assert_eq!(ctrl.current_retard(), 5.0); // 10 - 5*1 = 5
    }
}
