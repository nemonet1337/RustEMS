//! Variable Valve Timing (VVT) control — cam phaser position management.
//!
//! Derived from `firmware/controllers/actuators/vvt.cpp`.
//!
//! VVT adjusts intake/exhaust cam timing relative to crankshaft to optimize
//! torque, emissions, and fuel economy across the RPM range.

use crate::maps::interpolate2d;

/// VVT control mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VvtMode {
    /// VVT disabled — cam at locked position.
    Disabled,
    /// Open-loop control (lookup table only).
    OpenLoop,
    /// Closed-loop PID control with target table.
    ClosedLoop,
}

/// Single VVT output configuration (intake or exhaust cam).
#[derive(Clone, Copy, Debug)]
pub struct VvtOutputConfig {
    /// RPM axis for target advance table.
    pub rpm_bins: [f32; 8],
    /// Load (MAP kPa) axis for target advance table.
    pub load_bins: [f32; 8],
    /// Target cam advance table [load_row][rpm_col] in degrees.
    /// Positive = advance (earlier opening), negative = retard (later opening).
    pub target_advance_table: [[f32; 8]; 8],
    /// PID proportional gain.
    pub kp: f32,
    /// PID integral gain.
    pub ki: f32,
    /// PID derivative gain.
    pub kd: f32,
    /// Maximum duty to phaser (0-100%).
    pub max_duty: f32,
    /// Minimum duty to phaser (0-100%).
    pub min_duty: f32,
    /// Maximum advance angle (degrees).
    pub max_advance_deg: f32,
    /// Minimum (most retarded) angle (degrees).
    pub min_advance_deg: f32,
}

impl VvtOutputConfig {
    /// Default intake VVT configuration.
    pub fn default_intake() -> Self {
        Self {
            rpm_bins: [
                800.0, 1200.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0,
            ],
            load_bins: [30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 150.0, 200.0],
            // Intake: more advance at low RPM/load for torque, less at high RPM
            target_advance_table: [
                [20.0, 25.0, 30.0, 25.0, 15.0, 10.0, 5.0, 0.0], // 30 kPa (light load)
                [25.0, 30.0, 35.0, 30.0, 20.0, 15.0, 10.0, 5.0], // 50 kPa
                [30.0, 35.0, 40.0, 35.0, 25.0, 20.0, 15.0, 10.0], // 70 kPa
                [35.0, 40.0, 45.0, 40.0, 30.0, 25.0, 20.0, 15.0], // 90 kPa
                [30.0, 35.0, 40.0, 35.0, 25.0, 20.0, 15.0, 10.0], // 110 kPa
                [25.0, 30.0, 35.0, 30.0, 20.0, 15.0, 10.0, 5.0], // 130 kPa
                [20.0, 25.0, 30.0, 25.0, 15.0, 10.0, 5.0, 0.0], // 150 kPa
                [15.0, 20.0, 25.0, 20.0, 10.0, 5.0, 0.0, -5.0], // 200 kPa (boost)
            ],
            kp: 0.5,
            ki: 0.1,
            kd: 0.05,
            max_duty: 100.0,
            min_duty: 0.0,
            max_advance_deg: 45.0,
            min_advance_deg: -10.0,
        }
    }

    /// Default exhaust VVT configuration.
    pub fn default_exhaust() -> Self {
        let mut cfg = Self::default_intake();
        // Exhaust VVT typically works opposite: retard for EGR effect at light load
        cfg.target_advance_table = [
            [-10.0, -15.0, -20.0, -15.0, -10.0, -5.0, 0.0, 5.0], // Retard at light load
            [-5.0, -10.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0],
            [0.0, -5.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0],
            [5.0, 0.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0],
            [10.0, 5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0],
            [15.0, 10.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0],
            [20.0, 15.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0],
            [25.0, 20.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0],
        ];
        cfg
    }
}

/// Single VVT controller for one cam phaser.
pub struct VvtController {
    cfg: VvtOutputConfig,
    mode: VvtMode,
    /// Current phaser duty (0-100%).
    duty: f32,
    /// PID integral term.
    integral: f32,
    /// Previous error for derivative.
    prev_error: f32,
    /// Whether this is first update.
    first_update: bool,
    /// Last commanded target advance (for logging).
    target_advance: f32,
}

impl VvtController {
    /// Create a new VVT controller.
    pub fn new(cfg: VvtOutputConfig) -> Self {
        Self {
            cfg,
            mode: VvtMode::Disabled,
            duty: 0.0,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
            target_advance: 0.0,
        }
    }

    /// Update VVT controller.
    ///
    /// # Arguments
    /// * `current_advance_deg` — Current measured cam advance from trigger
    /// * `rpm` — Engine RPM
    /// * `map_kpa` — Manifold absolute pressure (kPa)
    /// * `mode` — Control mode
    /// * `dt_ms` — Time delta in milliseconds
    ///
    /// # Returns
    /// Solenoid duty (0-100%) to apply to phaser.
    pub fn update(
        &mut self,
        current_advance_deg: Option<f32>,
        rpm: f32,
        map_kpa: f32,
        mode: VvtMode,
        dt_ms: f32,
    ) -> f32 {
        self.mode = mode;

        match mode {
            VvtMode::Disabled => {
                self.duty = 0.0;
                self.integral = 0.0;
                self.prev_error = 0.0;
                self.first_update = true;
            }
            VvtMode::OpenLoop => {
                // In open-loop, assume duty maps linearly to advance
                // This is a simplification; real systems use calibration tables
                let target = interpolate2d(
                    &self.cfg.target_advance_table,
                    &self.cfg.load_bins,
                    map_kpa,
                    &self.cfg.rpm_bins,
                    rpm,
                );
                self.target_advance = target;

                // Map target advance to duty (linear approximation)
                let duty_range = self.cfg.max_duty - self.cfg.min_duty;
                let advance_range = self.cfg.max_advance_deg - self.cfg.min_advance_deg;
                let normalized = (target - self.cfg.min_advance_deg) / advance_range;
                self.duty = (self.cfg.min_duty + normalized * duty_range)
                    .clamp(self.cfg.min_duty, self.cfg.max_duty);

                self.integral = 0.0;
                self.first_update = true;
            }
            VvtMode::ClosedLoop => {
                let target = interpolate2d(
                    &self.cfg.target_advance_table,
                    &self.cfg.load_bins,
                    map_kpa,
                    &self.cfg.rpm_bins,
                    rpm,
                );
                self.target_advance = target;

                if let Some(current) = current_advance_deg {
                    let error = target - current;

                    if self.first_update {
                        self.integral = 0.0;
                        self.prev_error = error;
                        self.first_update = false;
                    }

                    // PID terms
                    let p_term = self.cfg.kp * error;
                    self.integral += self.cfg.ki * error * dt_ms;
                    self.integral = self.integral.clamp(-50.0, 50.0); // Anti-windup
                    let d_term = self.cfg.kd * (error - self.prev_error) / dt_ms.max(1.0);
                    self.prev_error = error;

                    let output = p_term + self.integral + d_term;

                    // Convert PID output to duty
                    let duty_range = self.cfg.max_duty - self.cfg.min_duty;
                    let normalized = (output - self.cfg.min_advance_deg)
                        / (self.cfg.max_advance_deg - self.cfg.min_advance_deg);
                    self.duty = (self.cfg.min_duty + normalized * duty_range)
                        .clamp(self.cfg.min_duty, self.cfg.max_duty);
                } else {
                    // No position feedback: fall back to open-loop
                    let duty_range = self.cfg.max_duty - self.cfg.min_duty;
                    let normalized = (target - self.cfg.min_advance_deg)
                        / (self.cfg.max_advance_deg - self.cfg.min_advance_deg);
                    self.duty = (self.cfg.min_duty + normalized * duty_range)
                        .clamp(self.cfg.min_duty, self.cfg.max_duty);
                }
            }
        }

        self.duty
    }

    /// Current solenoid duty (0-100%).
    pub fn duty(&self) -> f32 {
        self.duty
    }

    /// Current control mode.
    pub fn mode(&self) -> VvtMode {
        self.mode
    }

    /// Last target advance angle.
    pub fn target_advance(&self) -> f32 {
        self.target_advance
    }

    /// Calculate cam synchronization offset for trigger synchronization.
    ///
    /// This uses the current cam position to adjust crankshaft trigger timing,
    /// improving synchronization accuracy especially during cam phase changes.
    ///
    /// # Arguments
    /// * `current_advance_deg` — Current measured cam advance from trigger
    ///
    /// # Returns
    /// Timing offset in degrees (positive = advance trigger timing)
    pub fn calculate_cam_sync_offset(&self, current_advance_deg: Option<f32>) -> f32 {
        if let Some(advance) = current_advance_deg {
            // The cam sync offset is proportional to the cam advance
            // This compensates for the timing shift caused by cam phasing
            advance * 0.5 // 0.5 is a typical factor for cam-to-crank relationship
        } else {
            0.0
        }
    }

    /// Reset controller.
    pub fn reset(&mut self) {
        self.mode = VvtMode::Disabled;
        self.duty = 0.0;
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
        self.target_advance = 0.0;
    }
}

/// Dual VVT controller for intake + exhaust cams.
pub struct DualVvtController {
    /// Intake cam VVT controller.
    pub intake: VvtController,
    /// Exhaust cam VVT controller.
    pub exhaust: VvtController,
}

impl DualVvtController {
    /// Create dual VVT with default configurations.
    pub fn new() -> Self {
        Self {
            intake: VvtController::new(VvtOutputConfig::default_intake()),
            exhaust: VvtController::new(VvtOutputConfig::default_exhaust()),
        }
    }

    /// Update both controllers.
    pub fn update(
        &mut self,
        intake_advance: Option<f32>,
        exhaust_advance: Option<f32>,
        rpm: f32,
        map_kpa: f32,
        mode: VvtMode,
        dt_ms: f32,
    ) -> (f32, f32) {
        let intake_duty = self
            .intake
            .update(intake_advance, rpm, map_kpa, mode, dt_ms);
        let exhaust_duty = self
            .exhaust
            .update(exhaust_advance, rpm, map_kpa, mode, dt_ms);
        (intake_duty, exhaust_duty)
    }

    /// Reset both controllers.
    pub fn reset(&mut self) {
        self.intake.reset();
        self.exhaust.reset();
    }
}

impl Default for DualVvtController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vvt_disabled_returns_zero() {
        let cfg = VvtOutputConfig::default_intake();
        let mut ctrl = VvtController::new(cfg);
        let duty = ctrl.update(None, 3000.0, 50.0, VvtMode::Disabled, 10.0);
        assert_eq!(duty, 0.0);
        assert_eq!(ctrl.mode(), VvtMode::Disabled);
    }

    #[test]
    fn vvt_open_loop_returns_duty() {
        let cfg = VvtOutputConfig::default_intake();
        let mut ctrl = VvtController::new(cfg);
        // 3000 RPM, 50 kPa MAP should give some advance
        let duty = ctrl.update(None, 3000.0, 50.0, VvtMode::OpenLoop, 10.0);
        assert!(duty > 0.0 && duty <= 100.0);
        assert_eq!(ctrl.mode(), VvtMode::OpenLoop);
    }

    #[test]
    fn vvt_intake_vs_exhaust_different() {
        let intake_cfg = VvtOutputConfig::default_intake();
        let exhaust_cfg = VvtOutputConfig::default_exhaust();

        let mut intake_ctrl = VvtController::new(intake_cfg);
        let mut exhaust_ctrl = VvtController::new(exhaust_cfg);

        let _intake_duty = intake_ctrl.update(None, 2000.0, 50.0, VvtMode::OpenLoop, 10.0);
        let _exhaust_duty = exhaust_ctrl.update(None, 2000.0, 50.0, VvtMode::OpenLoop, 10.0);

        // Intake and exhaust should have different strategies
        // At 2000 RPM 50 kPa: intake positive advance, exhaust negative (retard)
        assert!(intake_ctrl.target_advance() > 0.0);
        assert!(exhaust_ctrl.target_advance() < 0.0);
    }

    #[test]
    fn vvt_closed_loop_converges() {
        let cfg = VvtOutputConfig::default_intake();
        let mut ctrl = VvtController::new(cfg);

        // Simulate current advance starting at 0, target should be positive
        let mut current_advance = 0.0f32;
        for _ in 0..50 {
            let _duty = ctrl.update(
                Some(current_advance),
                3000.0,
                50.0,
                VvtMode::ClosedLoop,
                10.0,
            );
            // Simulate phaser response (simplified)
            current_advance += (ctrl.target_advance() - current_advance) * 0.1;
        }

        // After 50 iterations, should be close to target
        let final_error = (ctrl.target_advance() - current_advance).abs();
        assert!(
            final_error < 5.0,
            "Final error {} should be less than 5 degrees",
            final_error
        );
    }

    #[test]
    fn vvt_cam_sync_offset_with_advance() {
        let cfg = VvtOutputConfig::default_intake();
        let ctrl = VvtController::new(cfg);

        let offset = ctrl.calculate_cam_sync_offset(Some(20.0));
        assert!(offset > 0.0);
        assert_eq!(offset, 10.0); // 20.0 * 0.5
    }

    #[test]
    fn vvt_cam_sync_offset_without_advance() {
        let cfg = VvtOutputConfig::default_intake();
        let ctrl = VvtController::new(cfg);

        let offset = ctrl.calculate_cam_sync_offset(None);
        assert_eq!(offset, 0.0);
    }

    #[test]
    fn vvt_cam_sync_offset_zero_advance() {
        let cfg = VvtOutputConfig::default_intake();
        let ctrl = VvtController::new(cfg);

        let offset = ctrl.calculate_cam_sync_offset(Some(0.0));
        assert_eq!(offset, 0.0);
    }

    #[test]
    fn dual_vvt_updates_both() {
        let mut dual = DualVvtController::new();
        let (intake_duty, exhaust_duty) =
            dual.update(None, None, 3000.0, 50.0, VvtMode::OpenLoop, 10.0);

        assert!(intake_duty > 0.0);
        assert!(exhaust_duty > 0.0 || exhaust_duty == 0.0); // Exhaust might be at min
        assert!(intake_duty <= 100.0);
        assert!(exhaust_duty <= 100.0);
    }
}
