//! Generic PWM output and auxiliary PID control.
//!
//! Derived from `firmware/controllers/actuators/pwm_generator.cpp`.

/// Auxiliary PID output configuration.
#[derive(Clone, Copy, Debug)]
pub struct AuxPidConfig {
    /// Enable auxiliary PID control.
    pub enabled: bool,
    /// PID proportional gain.
    pub kp: f32,
    /// PID integral gain.
    pub ki: f32,
    /// PID derivative gain.
    pub kd: f32,
    /// Minimum output value (0-100% or raw units).
    pub min_output: f32,
    /// Maximum output value (0-100% or raw units).
    pub max_output: f32,
}

impl Default for AuxPidConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            kp: 1.0,
            ki: 0.1,
            kd: 0.0,
            min_output: 0.0,
            max_output: 100.0,
        }
    }
}

/// Auxiliary PID controller for generic outputs.
pub struct AuxPidController {
    cfg: AuxPidConfig,
    /// Integral term.
    integral: f32,
    /// Previous error for derivative.
    prev_error: f32,
    /// First update flag.
    first_update: bool,
    /// Current output.
    output: f32,
}

impl AuxPidController {
    /// Create new auxiliary PID controller.
    pub fn new(cfg: AuxPidConfig) -> Self {
        Self {
            cfg,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
            output: 0.0,
        }
    }

    /// Update auxiliary PID controller.
    ///
    /// # Arguments
    /// * `setpoint` — Target value
    /// * `process_variable` — Current measured value
    /// * `dt_s` — Time delta in seconds
    ///
    /// # Returns
    /// Output value (clamped to min/max)
    pub fn update(&mut self, setpoint: f32, process_variable: f32, dt_s: f32) -> f32 {
        if !self.cfg.enabled {
            self.output = 0.0;
            return 0.0;
        }

        let error = setpoint - process_variable;

        if self.first_update {
            self.integral = 0.0;
            self.prev_error = error;
            self.first_update = false;
        }

        // PID terms
        let p_term = self.cfg.kp * error;
        self.integral += self.cfg.ki * error * dt_s;
        self.integral = self.integral.clamp(-100.0, 100.0); // Anti-windup
        let d_term = if dt_s > 0.0 {
            self.cfg.kd * (error - self.prev_error) / dt_s
        } else {
            0.0
        };
        self.prev_error = error;

        let output = p_term + self.integral + d_term;
        self.output = output.clamp(self.cfg.min_output, self.cfg.max_output);

        self.output
    }

    /// Current output value.
    pub fn output(&self) -> f32 {
        self.output
    }

    /// Reset controller.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
        self.output = 0.0;
    }

    /// Get configuration reference.
    pub fn config(&self) -> &AuxPidConfig {
        &self.cfg
    }

    /// Update configuration.
    pub fn set_config(&mut self, cfg: AuxPidConfig) {
        self.cfg = cfg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aux_pid_disabled_returns_zero() {
        let cfg = AuxPidConfig::default();
        let mut ctrl = AuxPidController::new(cfg);
        
        let output = ctrl.update(10.0, 5.0, 0.01);
        assert_eq!(output, 0.0);
    }

    #[test]
    fn aux_pid_enabled_produces_output() {
        let mut cfg = AuxPidConfig::default();
        cfg.enabled = true;
        let mut ctrl = AuxPidController::new(cfg);
        
        let output = ctrl.update(10.0, 5.0, 0.01);
        assert!(output > 0.0);
    }

    #[test]
    fn aux_pid_converges_to_setpoint() {
        let mut cfg = AuxPidConfig::default();
        cfg.enabled = true;
        cfg.kp = 1.0;
        cfg.ki = 0.5;
        let mut ctrl = AuxPidController::new(cfg);
        
        let setpoint = 10.0;
        let mut process_variable = 0.0;

        // Model a stable first-order plant: the measured variable relaxes
        // toward the commanded output each step. A correctly tuned PID drives
        // this kind of process to the setpoint.
        for _ in 0..200 {
            let output = ctrl.update(setpoint, process_variable, 0.05);
            process_variable += (output - process_variable) * 0.3;
        }

        let final_error = (setpoint - process_variable).abs();
        assert!(final_error < 1.0, "Final error {} should be less than 1", final_error);
    }

    #[test]
    fn aux_pid_output_clamped() {
        let mut cfg = AuxPidConfig::default();
        cfg.enabled = true;
        cfg.kp = 100.0; // Very high gain
        cfg.max_output = 50.0;
        let mut ctrl = AuxPidController::new(cfg);
        
        let output = ctrl.update(10.0, 0.0, 0.01);
        assert!(output <= cfg.max_output);
    }

    #[test]
    fn aux_pid_reset_clears_state() {
        let mut cfg = AuxPidConfig::default();
        cfg.enabled = true;
        let mut ctrl = AuxPidController::new(cfg);
        
        let _ = ctrl.update(10.0, 5.0, 0.01);
        assert!(ctrl.output() > 0.0);
        
        ctrl.reset();
        assert_eq!(ctrl.output(), 0.0);
    }
}
