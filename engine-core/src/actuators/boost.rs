//! Boost control — open-loop base duty + closed-loop PID correction.
//!
//! Derived from `firmware/controllers/actuators/boost_control.cpp`.

use crate::maps::interpolate2d;

/// Boost control configuration.
#[derive(Clone, Copy, Debug)]
pub struct BoostConfig {
    /// RPM axis for open-loop duty table.
    pub rpm_bins: [f32; 8],
    /// Load (TPS %) axis for open-loop duty table.
    pub load_bins: [f32; 8],
    /// Open-loop wastegate duty table [%] based on RPM and load.
    pub open_loop_duty_table: [[f32; 8]; 8],
    /// Target boost pressure axis (kPa) for closed-loop.
    pub target_kpa: f32,
    /// PID proportional gain for boost correction.
    pub kp: f32,
    /// PID integral gain.
    pub ki: f32,
    /// PID derivative gain.
    pub kd: f32,
    /// Maximum wastegate duty (0-100%).
    pub max_duty: f32,
    /// Minimum wastegate duty (0-100%).
    pub min_duty: f32,
    /// Maximum boost correction from PID (±%).
    pub max_correction: f32,
    /// RPM threshold below which boost control is disabled.
    pub min_rpm: f32,
}

impl BoostConfig {
    /// Default boost control configuration for turbocharged engine.
    pub fn default_turbo() -> Self {
        Self {
            rpm_bins: [1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0],
            load_bins: [20.0, 40.0, 60.0, 70.0, 80.0, 90.0, 95.0, 100.0],
            // Open-loop: higher duty = more boost (less wastegate opening)
            open_loop_duty_table: [
                [10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0], // 1000 RPM
                [15.0, 25.0, 35.0, 45.0, 55.0, 60.0, 65.0, 70.0], // 2000 RPM
                [20.0, 35.0, 50.0, 60.0, 70.0, 75.0, 80.0, 85.0], // 3000 RPM
                [25.0, 40.0, 55.0, 65.0, 75.0, 80.0, 85.0, 90.0], // 4000 RPM
                [30.0, 45.0, 60.0, 70.0, 78.0, 83.0, 88.0, 92.0], // 5000 RPM
                [35.0, 50.0, 62.0, 72.0, 80.0, 85.0, 90.0, 95.0], // 6000 RPM
                [40.0, 52.0, 64.0, 74.0, 82.0, 87.0, 92.0, 96.0], // 7000 RPM
                [42.0, 54.0, 66.0, 76.0, 84.0, 89.0, 94.0, 98.0], // 8000 RPM
            ],
            target_kpa: 200.0, // ~15 psi boost
            kp: 0.1,
            ki: 0.02,
            kd: 0.0,
            max_duty: 100.0,
            min_duty: 0.0,
            max_correction: 20.0, // ±20% correction from PID
            min_rpm: 1500.0,
        }
    }
}

/// Boost controller state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoostState {
    /// Boost control disabled (naturally aspirated or control off).
    Disabled,
    /// Below minimum RPM — boost control inactive.
    BelowMinRpm,
    /// Open-loop mode only (no PID correction).
    OpenLoop,
    /// Closed-loop active with PID correction.
    ClosedLoop,
}

/// Boost controller for wastegate duty management.
pub struct BoostController {
    cfg: BoostConfig,
    state: BoostState,
    /// Current wastegate solenoid duty (0-100%).
    wastegate_duty: f32,
    /// PID integral term.
    integral: f32,
    /// Previous error for derivative.
    prev_error: f32,
    /// Whether this is first update.
    first_update: bool,
}

impl BoostController {
    /// Create a new boost controller.
    pub fn new(cfg: BoostConfig) -> Self {
        Self {
            cfg,
            state: BoostState::Disabled,
            wastegate_duty: 0.0,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
        }
    }

    /// Update boost controller.
    ///
    /// # Arguments
    /// * `map_kpa` — Manifold absolute pressure in kPa
    /// * `rpm` — Engine RPM
    /// * `tps_pct` — Throttle position %
    /// * `closed_loop_enabled` — Whether to use PID correction
    /// * `dt_ms` — Time delta in milliseconds
    ///
    /// # Returns
    /// Wastegate solenoid duty (0-100%).
    pub fn update(
        &mut self,
        map_kpa: f32,
        rpm: f32,
        tps_pct: f32,
        closed_loop_enabled: bool,
        dt_ms: f32,
    ) -> f32 {
        // Determine state
        self.state = if rpm < self.cfg.min_rpm {
            BoostState::BelowMinRpm
        } else if !closed_loop_enabled {
            BoostState::OpenLoop
        } else {
            BoostState::ClosedLoop
        };

        match self.state {
            BoostState::Disabled | BoostState::BelowMinRpm => {
                self.wastegate_duty = 0.0;
                self.integral = 0.0;
                self.prev_error = 0.0;
                self.first_update = true;
            }
            BoostState::OpenLoop => {
                // Open-loop only: lookup duty from table
                let base_duty = interpolate2d(
                    &self.cfg.open_loop_duty_table,
                    &self.cfg.rpm_bins,
                    rpm,
                    &self.cfg.load_bins,
                    tps_pct,
                );
                self.wastegate_duty = base_duty.clamp(self.cfg.min_duty, self.cfg.max_duty);
                self.integral = 0.0;
                self.prev_error = 0.0;
                self.first_update = true;
            }
            BoostState::ClosedLoop => {
                // Calculate base duty from open-loop table
                let base_duty = interpolate2d(
                    &self.cfg.open_loop_duty_table,
                    &self.cfg.rpm_bins,
                    rpm,
                    &self.cfg.load_bins,
                    tps_pct,
                );

                // Calculate PID correction
                let error = self.cfg.target_kpa - map_kpa;

                if self.first_update {
                    self.integral = 0.0;
                    self.prev_error = error;
                    self.first_update = false;
                }

                // Proportional term
                let p_term = self.cfg.kp * error;

                // Integral term with anti-windup
                self.integral += self.cfg.ki * error * dt_ms;
                self.integral = self.integral.clamp(-self.cfg.max_correction, self.cfg.max_correction);

                // Derivative term
                let d_term = self.cfg.kd * (error - self.prev_error) / dt_ms.max(1.0);
                self.prev_error = error;

                // Sum correction
                let correction = (p_term + self.integral + d_term)
                    .clamp(-self.cfg.max_correction, self.cfg.max_correction);

                // Apply correction to base duty
                self.wastegate_duty = (base_duty + correction)
                    .clamp(self.cfg.min_duty, self.cfg.max_duty);
            }
        }

        self.wastegate_duty
    }

    /// Current wastegate solenoid duty (0-100%).
    pub fn duty(&self) -> f32 {
        self.wastegate_duty
    }

    /// Current controller state.
    pub fn state(&self) -> BoostState {
        self.state
    }

    /// Reset controller.
    pub fn reset(&mut self) {
        self.state = BoostState::Disabled;
        self.wastegate_duty = 0.0;
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boost_disabled_below_min_rpm() {
        let cfg = BoostConfig::default_turbo();
        let mut ctrl = BoostController::new(cfg);
        let duty = ctrl.update(100.0, 1000.0, 50.0, true, 10.0);
        assert_eq!(duty, 0.0);
        assert_eq!(ctrl.state(), BoostState::BelowMinRpm);
    }

    #[test]
    fn boost_open_loop_lookup() {
        let cfg = BoostConfig::default_turbo();
        let mut ctrl = BoostController::new(cfg);
        // 3000 RPM, 80% TPS - should return interpolated duty
        let duty = ctrl.update(100.0, 3000.0, 80.0, false, 10.0);
        assert!(duty > 0.0 && duty < 100.0);
        assert_eq!(ctrl.state(), BoostState::OpenLoop);
    }

    #[test]
    fn boost_closed_loop_adjusts_duty() {
        let cfg = BoostConfig::default_turbo();
        let mut ctrl = BoostController::new(cfg);

        // Test that PID responds to error
        let low_boost_duty = ctrl.update(150.0, 4000.0, 100.0, true, 10.0); // Below target
        let high_boost_duty = ctrl.update(220.0, 4000.0, 100.0, true, 10.0); // Above target

        // Lower boost should increase duty (to build more boost)
        // Higher boost should decrease duty (to vent more)
        assert!(low_boost_duty > high_boost_duty,
            "Controller should increase duty for low boost: {} vs {}", low_boost_duty, high_boost_duty);
    }
}
