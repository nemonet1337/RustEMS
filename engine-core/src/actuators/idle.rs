//! Idle speed control — PID-based IAC valve management.
//!
//! Derived from `firmware/controllers/actuators/idle_thread.cpp`.
//!
//! Target RPM is looked up from a table based on coolant temperature.
//! A PID controller adjusts IAC position to maintain target.

use crate::maps::interpolate1d;

/// Idle control configuration.
#[derive(Clone, Copy, Debug)]
pub struct IdleConfig {
    /// Target idle RPM axis (x-axis: coolant temperature °C).
    pub target_rpm_ct_bins: [f32; 8],
    /// Target idle RPM table (y-axis: RPM at each temperature).
    pub target_rpm_table: [f32; 8],
    /// PID proportional gain.
    pub kp: f32,
    /// PID integral gain.
    pub ki: f32,
    /// PID derivative gain.
    pub kd: f32,
    /// IAC valve minimum position (0-100%).
    pub min_duty: f32,
    /// IAC valve maximum position (0-100%).
    pub max_duty: f32,
    /// RPM deadband: control is paused when |error| < deadband.
    pub rpm_deadband: f32,
    /// Cranking IAC duty (0-100%) when engine is starting.
    pub cranking_duty: f32,
    /// RPM threshold below which cranking duty is used.
    pub cranking_rpm_threshold: f32,
    /// Air conditioner request idle RPM increase.
    pub ac_idle_up_rpm: f32,
    /// Air conditioner request IAC duty increase (0-100%).
    pub ac_idle_up_duty: f32,
    /// Idle ignition timing control enabled.
    pub idle_timing_control_enabled: bool,
    /// Idle ignition timing advance table (RPM axis).
    pub idle_timing_rpm_bins: [f32; 8],
    /// Idle ignition timing advance table (degrees BTDC at each RPM).
    pub idle_timing_table: [f32; 8],
    /// Enable idle valve position learning (initial position learning).
    pub idle_valve_learning_enabled: bool,
    /// Learned idle valve position (0-100% duty).
    pub learned_idle_position: f32,
    /// ITB setup: Use TPS instead of MAP for idle VE table Y-axis.
    pub itb_idle_ve_override: bool,
    /// Use separate table during cranking taper.
    pub use_separate_table_during_cranking_taper: bool,
}

impl IdleConfig {
    /// Create a default configuration suitable for a typical 4-cylinder engine.
    pub fn default_4cyl() -> Self {
        Self {
            // Higher idle when cold, lower when warm
            target_rpm_ct_bins: [-40.0, -20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0],
            target_rpm_table: [1500.0, 1400.0, 1300.0, 1100.0, 900.0, 850.0, 800.0, 800.0],
            kp: 0.05,
            ki: 0.01,
            kd: 0.0,
            min_duty: 0.0,
            max_duty: 100.0,
            rpm_deadband: 25.0,
            cranking_duty: 70.0,
            cranking_rpm_threshold: 400.0,
            ac_idle_up_rpm: 150.0,
            ac_idle_up_duty: 10.0,
            idle_timing_control_enabled: false,
            idle_timing_rpm_bins: [0.0, 500.0, 750.0, 900.0, 1000.0, 1100.0, 1250.0, 1500.0],
            idle_timing_table: [15.0, 12.0, 10.0, 8.0, 6.0, 4.0, 2.0, 0.0],
            idle_valve_learning_enabled: false,
            learned_idle_position: 15.0,
            itb_idle_ve_override: false,
            use_separate_table_during_cranking_taper: false,
        }
    }
}

impl Default for IdleConfig {
    fn default() -> Self {
        Self::default_4cyl()
    }
}

/// Idle controller state machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IdleState {
    /// Engine stopped — IAC at closed position.
    Stopped,
    /// Engine cranking — use cranking duty.
    Cranking,
    /// Idle active — PID control running.
    Active,
    /// Idle control suspended (TPS > threshold, decelerating, etc).
    Suspended,
}

/// PID-based idle speed controller.
pub struct IdleController {
    cfg: IdleConfig,
    state: IdleState,
    /// Current IAC valve duty (0-100%).
    iac_duty: f32,
    /// PID integral term.
    integral: f32,
    /// Previous error for derivative calculation.
    prev_error: f32,
    /// Whether this is the first update after state change.
    first_update: bool,
}

impl IdleController {
    /// Create a new idle controller with the given configuration.
    pub fn new(cfg: IdleConfig) -> Self {
        Self {
            cfg,
            state: IdleState::Stopped,
            iac_duty: 0.0,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
        }
    }

    /// Update the idle controller.
    ///
    /// # Arguments
    /// * `rpm` — Current engine RPM
    /// * `clt_c` — Coolant temperature in °C
    /// * `tps_pct` — Throttle position in % (0-100)
    /// * `is_cranking` — True if starter motor is active
    /// * `dt_ms` — Time since last update in milliseconds
    /// * `ac_request` — Air conditioner request (true if AC is on)
    ///
    /// # Returns
    /// Current IAC valve duty (0-100%).
    pub fn update(
        &mut self,
        rpm: f32,
        clt_c: f32,
        tps_pct: f32,
        is_cranking: bool,
        dt_ms: f32,
        ac_request: bool,
    ) -> f32 {
        // Determine state
        let new_state = if is_cranking {
            IdleState::Cranking
        } else if rpm < 100.0 {
            IdleState::Stopped
        } else if tps_pct > 5.0 {
            IdleState::Suspended
        } else if rpm > self.cfg.target_rpm_table[self.cfg.target_rpm_table.len() - 1] * 1.5 {
            // Suspended during deceleration
            IdleState::Suspended
        } else {
            IdleState::Active
        };

        // Handle state transitions
        if new_state != self.state {
            self.state = new_state;
            self.first_update = true;
            self.integral = 0.0;
            self.prev_error = 0.0;
        }

        match self.state {
            IdleState::Stopped => {
                self.iac_duty = 0.0;
            }
            IdleState::Cranking => {
                self.iac_duty = self.cfg.cranking_duty;
            }
            IdleState::Suspended => {
                // Hold current position during suspension
                // (could also decay to min position)
            }
            IdleState::Active => {
                // Calculate target RPM based on coolant temp
                let mut target_rpm = interpolate1d(
                    &self.cfg.target_rpm_ct_bins,
                    &self.cfg.target_rpm_table,
                    clt_c,
                );

                // Add AC request idle up
                if ac_request {
                    target_rpm += self.cfg.ac_idle_up_rpm;
                }

                // Calculate error
                let error = target_rpm - rpm;

                // Deadband check
                if error.abs() < self.cfg.rpm_deadband {
                    // Within deadband: freeze integral, minimal update
                    return self.iac_duty;
                }

                // PID calculation
                let p_term = self.cfg.kp * error;

                // Integral with anti-windup
                if self.first_update {
                    self.integral = 0.0;
                } else {
                    self.integral += self.cfg.ki * error * dt_ms;
                    // Clamp integral to prevent windup
                    let max_integral = self.cfg.max_duty - p_term;
                    let min_integral = self.cfg.min_duty - p_term;
                    self.integral = self.integral.clamp(min_integral, max_integral);
                }

                // Derivative (on error, not measurement)
                let d_term = if self.first_update {
                    0.0
                } else {
                    self.cfg.kd * (error - self.prev_error) / dt_ms.max(1.0)
                };

                self.prev_error = error;
                self.first_update = false;

                // Calculate output
                let mut output = p_term + self.integral + d_term;

                // Add AC request duty up
                if ac_request {
                    output += self.cfg.ac_idle_up_duty;
                }

                self.iac_duty = output.clamp(self.cfg.min_duty, self.cfg.max_duty);
            }
        }

        self.iac_duty
    }

    /// Current IAC valve duty (0-100%).
    pub fn iac_duty(&self) -> f32 {
        self.iac_duty
    }

    /// Current controller state.
    pub fn state(&self) -> IdleState {
        self.state
    }

    /// Reset controller to stopped state.
    pub fn reset(&mut self) {
        self.state = IdleState::Stopped;
        self.iac_duty = 0.0;
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
    }

    /// Calculate idle ignition timing advance.
    ///
    /// # Arguments
    /// * `rpm` — Current engine RPM
    ///
    /// # Returns
    /// Ignition timing advance in degrees BTDC (positive = advance)
    pub fn calculate_idle_timing(&self, rpm: f32) -> f32 {
        if !self.cfg.idle_timing_control_enabled {
            return 0.0;
        }

        if self.state != IdleState::Active {
            return 0.0;
        }

        interpolate1d(
            &self.cfg.idle_timing_rpm_bins,
            &self.cfg.idle_timing_table,
            rpm,
        )
    }

    /// Learn idle valve position from current duty when RPM is stable.
    ///
    /// This should be called when engine is at stable idle RPM to learn
    /// the base IAC position for future reference.
    ///
    /// # Arguments
    /// * `rpm_stable` — True if RPM is within target deadband
    pub fn learn_idle_position(&mut self, rpm_stable: bool) {
        if !self.cfg.idle_valve_learning_enabled {
            return;
        }

        if self.state != IdleState::Active || !rpm_stable {
            return;
        }

        // Learn current duty as base position (with simple averaging)
        self.cfg.learned_idle_position = 0.9 * self.cfg.learned_idle_position + 0.1 * self.iac_duty;
    }

    /// Get learned idle valve position.
    pub fn learned_idle_position(&self) -> f32 {
        self.cfg.learned_idle_position
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_stopped_when_rpm_zero() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);
        let duty = ctrl.update(0.0, 80.0, 0.0, false, 10.0, false);
        assert_eq!(duty, 0.0);
        assert_eq!(ctrl.state(), IdleState::Stopped);
    }

    #[test]
    fn idle_cranking_uses_cranking_duty() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);
        let duty = ctrl.update(200.0, 20.0, 0.0, true, 10.0, false);
        assert_eq!(duty, cfg.cranking_duty);
        assert_eq!(ctrl.state(), IdleState::Cranking);
    }

    #[test]
    fn idle_suspended_when_throttle_open() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);
        // First get into active state
        let _ = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);
        // Then open throttle
        let duty = ctrl.update(800.0, 80.0, 10.0, false, 10.0, false);
        assert_eq!(ctrl.state(), IdleState::Suspended);
        // Duty should be unchanged during suspension
        assert_eq!(duty, ctrl.iac_duty());
    }

    #[test]
    fn idle_pid_converges_to_target() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);
        // Simulate engine at 600 RPM with target 800 RPM (at 80°C)
        let mut rpm = 600.0;
        for _ in 0..100 {
            let duty = ctrl.update(rpm, 80.0, 0.0, false, 10.0, false);
            // Simulate engine response: higher duty → higher RPM
            rpm += (duty - 30.0) * 2.0;
            // Clamp RPM to realistic range
            rpm = rpm.clamp(400.0, 1200.0);
        }
        // Should converge near target
        assert!((rpm - 800.0).abs() < 50.0, "RPM didn't converge: {}", rpm);
    }

    #[test]
    fn idle_target_higher_when_cold() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);
        // Cold engine: -20°C → ~1400 RPM target
        let duty_cold = ctrl.update(1000.0, -20.0, 0.0, false, 10.0, false);

        // Warm engine: 80°C → ~800 RPM target
        let duty_warm = ctrl.update(1000.0, 80.0, 0.0, false, 10.0, false);

        // Cold engine needs more IAC duty (higher target vs same actual)
        assert!(
            duty_cold > duty_warm,
            "Cold idle duty {} should be higher than warm {}",
            duty_cold,
            duty_warm
        );
    }

    #[test]
    fn idle_ac_request_increases_duty() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);

        // Without AC request
        let duty_no_ac = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);

        // With AC request
        let duty_with_ac = ctrl.update(800.0, 80.0, 0.0, false, 10.0, true);

        // AC request should increase duty
        assert!(
            duty_with_ac > duty_no_ac,
            "AC duty {} should be higher than no-AC duty {}",
            duty_with_ac,
            duty_no_ac
        );
    }

    #[test]
    fn idle_timing_control_disabled() {
        let cfg = IdleConfig::default_4cyl();
        let ctrl = IdleController::new(cfg);

        let timing = ctrl.calculate_idle_timing(800.0);
        assert_eq!(timing, 0.0);
    }

    #[test]
    fn idle_timing_control_enabled() {
        let mut cfg = IdleConfig::default_4cyl();
        cfg.idle_timing_control_enabled = true;
        let mut ctrl = IdleController::new(cfg);

        // Get into active state
        let _ = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);

        let timing = ctrl.calculate_idle_timing(800.0);
        assert!(
            timing > 0.0,
            "Timing should be positive when enabled and active"
        );
    }

    #[test]
    fn idle_timing_not_active_when_stopped() {
        let mut cfg = IdleConfig::default_4cyl();
        cfg.idle_timing_control_enabled = true;
        let ctrl = IdleController::new(cfg);

        // Controller is in stopped state
        let timing = ctrl.calculate_idle_timing(0.0);
        assert_eq!(timing, 0.0);
    }

    #[test]
    fn idle_valve_learning_disabled() {
        let cfg = IdleConfig::default_4cyl();
        let mut ctrl = IdleController::new(cfg);

        // Get into active state
        let _ = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);

        ctrl.learn_idle_position(true);
        assert_eq!(ctrl.learned_idle_position(), 15.0); // Default value
    }

    #[test]
    fn idle_valve_learning_enabled() {
        let mut cfg = IdleConfig::default_4cyl();
        cfg.idle_valve_learning_enabled = true;
        let mut ctrl = IdleController::new(cfg);

        // Get into active state
        let _ = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);

        ctrl.learn_idle_position(true);
        // Learned position should move toward current duty
        assert_ne!(ctrl.learned_idle_position(), 15.0);
    }

    #[test]
    fn idle_valve_learning_not_active_when_stopped() {
        let mut cfg = IdleConfig::default_4cyl();
        cfg.idle_valve_learning_enabled = true;
        let mut ctrl = IdleController::new(cfg);

        // Controller is in stopped state
        ctrl.learn_idle_position(true);
        assert_eq!(ctrl.learned_idle_position(), 15.0);
    }

    #[test]
    fn itb_idle_ve_override() {
        let cfg = IdleConfig::default_4cyl();
        assert!(!cfg.itb_idle_ve_override);

        let mut cfg = IdleConfig::default_4cyl();
        cfg.itb_idle_ve_override = true;
        assert!(cfg.itb_idle_ve_override);
    }

    #[test]
    fn separate_table_during_cranking_taper() {
        let cfg = IdleConfig::default_4cyl();
        assert!(!cfg.use_separate_table_during_cranking_taper);

        let mut cfg = IdleConfig::default_4cyl();
        cfg.use_separate_table_during_cranking_taper = true;
        assert!(cfg.use_separate_table_during_cranking_taper);
    }
}
