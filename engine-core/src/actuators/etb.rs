//! Electronic Throttle Body (ETB / drive-by-wire) control.
//!
//! Closed-loop PID positioning of a motorised throttle plate from an
//! accelerator-pedal target, with dual-TPS plausibility monitoring and a
//! fail-safe limp strategy (de-energise → spring-return) per common DBW
//! safety practice.
//!
//! Boards with dual H-bridges (Huge / Proteus / uaEFI) instantiate two
//! controllers, one per throttle body.

use crate::maps::interpolation::interpolate1d;

/// Number of points in the pedal→throttle mapping curve.
pub const ETB_MAP_POINTS: usize = 8;

/// ETB fault conditions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EtbFault {
    /// Primary and secondary TPS disagree beyond the plausibility threshold.
    TpsImplausible,
    /// Position error stayed large for too long (jam / broken linkage).
    PositionTimeout,
}

/// ETB controller configuration.
#[derive(Clone, Copy, Debug)]
pub struct EtbConfig {
    /// Proportional gain (% duty per % position error).
    pub kp: f32,
    /// Integral gain.
    pub ki: f32,
    /// Derivative gain.
    pub kd: f32,
    /// Pedal-position axis for the throttle target curve (%).
    pub pedal_bins: [f32; ETB_MAP_POINTS],
    /// Throttle-target curve values (%), mapped from pedal position.
    pub throttle_map: [f32; ETB_MAP_POINTS],
    /// Maximum allowed |TPS1 − TPS2| before a plausibility fault (%).
    pub plausibility_threshold_pct: f32,
    /// Consecutive implausible samples before latching the fault.
    pub plausibility_count: u8,
    /// |position error| considered "not following" (%).
    pub position_error_limit_pct: f32,
    /// Time the position error may exceed the limit before faulting (ms).
    pub position_timeout_ms: u32,
    /// Output duty clamp (±%, H-bridge bidirectional).
    pub max_duty_pct: f32,
    /// Integral term clamp (±% duty).
    pub integral_limit: f32,
}

impl Default for EtbConfig {
    fn default() -> Self {
        Self {
            kp: 6.0,
            ki: 12.0,
            kd: 0.05,
            pedal_bins: [0.0, 5.0, 15.0, 30.0, 50.0, 70.0, 90.0, 100.0],
            // Progressive map: gentle tip-in, full authority at full pedal.
            throttle_map: [0.0, 2.0, 8.0, 20.0, 40.0, 65.0, 90.0, 100.0],
            plausibility_threshold_pct: 8.0,
            plausibility_count: 5,
            position_error_limit_pct: 15.0,
            position_timeout_ms: 500,
            max_duty_pct: 95.0,
            integral_limit: 40.0,
        }
    }
}

/// ETB controller output for one update step.
#[derive(Clone, Copy, Debug)]
pub struct EtbOutput {
    /// Motor duty (±%): positive opens, negative closes, 0 = spring return.
    pub duty_pct: f32,
    /// Throttle target after the pedal map (%).
    pub target_pct: f32,
    /// Latched fault, if any (output is forced to 0 while faulted).
    pub fault: Option<EtbFault>,
}

/// Closed-loop electronic throttle controller.
pub struct EtbController {
    cfg: EtbConfig,
    integral: f32,
    last_error: f32,
    implausible_samples: u8,
    error_timer_ms: f32,
    fault: Option<EtbFault>,
}

impl EtbController {
    /// Create a controller with the given configuration.
    pub fn new(cfg: EtbConfig) -> Self {
        Self {
            cfg,
            integral: 0.0,
            last_error: 0.0,
            implausible_samples: 0,
            error_timer_ms: 0.0,
            fault: None,
        }
    }

    /// Latched fault, if any.
    pub fn fault(&self) -> Option<EtbFault> {
        self.fault
    }

    /// Clear a latched fault (after key cycle / diagnostic clear).
    pub fn clear_fault(&mut self) {
        self.fault = None;
        self.implausible_samples = 0;
        self.error_timer_ms = 0.0;
        self.integral = 0.0;
    }

    /// Run one control step.
    ///
    /// * `pedal_pct` — accelerator pedal position (0–100 %)
    /// * `tps1_pct`  — primary throttle position sensor (0–100 %)
    /// * `tps2_pct`  — optional secondary TPS (0–100 %, same orientation)
    /// * `dt_s`      — time step in seconds
    pub fn update(
        &mut self,
        pedal_pct: f32,
        tps1_pct: f32,
        tps2_pct: Option<f32>,
        dt_s: f32,
    ) -> EtbOutput {
        let dt_s = dt_s.clamp(0.0001, 0.1);
        let target = interpolate1d(
            &self.cfg.pedal_bins,
            &self.cfg.throttle_map,
            pedal_pct.clamp(0.0, 100.0),
        );

        // ── Dual-TPS plausibility ───────────────────────────────────────────
        if let Some(tps2) = tps2_pct {
            let delta = tps1_pct - tps2;
            let delta = if delta < 0.0 { -delta } else { delta };
            if delta > self.cfg.plausibility_threshold_pct {
                self.implausible_samples = self.implausible_samples.saturating_add(1);
                if self.implausible_samples >= self.cfg.plausibility_count {
                    self.fault = Some(EtbFault::TpsImplausible);
                }
            } else {
                self.implausible_samples = 0;
            }
        }

        if self.fault.is_some() {
            // Fail-safe: de-energise; throttle springs back to its limp stop.
            self.integral = 0.0;
            return EtbOutput { duty_pct: 0.0, target_pct: target, fault: self.fault };
        }

        // ── Position-following watchdog ─────────────────────────────────────
        let error = target - tps1_pct;
        let abs_error = if error < 0.0 { -error } else { error };
        if abs_error > self.cfg.position_error_limit_pct {
            self.error_timer_ms += dt_s * 1000.0;
            if self.error_timer_ms >= self.cfg.position_timeout_ms as f32 {
                self.fault = Some(EtbFault::PositionTimeout);
                self.integral = 0.0;
                return EtbOutput { duty_pct: 0.0, target_pct: target, fault: self.fault };
            }
        } else {
            self.error_timer_ms = 0.0;
        }

        // ── PID ─────────────────────────────────────────────────────────────
        self.integral = (self.integral + error * self.cfg.ki * dt_s)
            .clamp(-self.cfg.integral_limit, self.cfg.integral_limit);
        let derivative = (error - self.last_error) / dt_s;
        self.last_error = error;

        let duty = (error * self.cfg.kp + self.integral + derivative * self.cfg.kd)
            .clamp(-self.cfg.max_duty_pct, self.cfg.max_duty_pct);

        EtbOutput { duty_pct: duty, target_pct: target, fault: None }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pedal_map_is_progressive() {
        let mut etb = EtbController::new(EtbConfig::default());
        let out = etb.update(50.0, 40.0, None, 0.01);
        assert!((out.target_pct - 40.0).abs() < 0.01);
        let out_low = etb.update(15.0, 8.0, None, 0.01);
        assert!(out_low.target_pct < 10.0);
    }

    #[test]
    fn drives_toward_target() {
        let mut etb = EtbController::new(EtbConfig::default());
        // Throttle below target → positive (opening) duty
        let out = etb.update(100.0, 50.0, Some(50.0), 0.01);
        assert!(out.duty_pct > 0.0);
        // Throttle above target → negative (closing) duty
        let mut etb2 = EtbController::new(EtbConfig::default());
        let out2 = etb2.update(0.0, 50.0, Some(50.0), 0.01);
        assert!(out2.duty_pct < 0.0);
    }

    #[test]
    fn converges_with_simple_plant() {
        let mut etb = EtbController::new(EtbConfig::default());
        let mut pos = 0.0f32;
        for _ in 0..2000 {
            let out = etb.update(50.0, pos, Some(pos), 0.005);
            // 1st-order actuator model: duty moves the plate
            pos = (pos + out.duty_pct * 0.01).clamp(0.0, 100.0);
        }
        assert!((pos - 40.0).abs() < 3.0, "pos = {pos}");
    }

    #[test]
    fn implausible_tps_latches_fault_and_kills_output() {
        let mut etb = EtbController::new(EtbConfig::default());
        for _ in 0..4 {
            let out = etb.update(50.0, 40.0, Some(60.0), 0.01);
            assert!(out.fault.is_none());
            let _ = out;
        }
        let out = etb.update(50.0, 40.0, Some(60.0), 0.01);
        assert_eq!(out.fault, Some(EtbFault::TpsImplausible));
        assert_eq!(out.duty_pct, 0.0);
        // Fault is latched even when readings agree again
        let out = etb.update(50.0, 40.0, Some(40.0), 0.01);
        assert_eq!(out.fault, Some(EtbFault::TpsImplausible));

        etb.clear_fault();
        let out = etb.update(50.0, 40.0, Some(40.0), 0.01);
        assert!(out.fault.is_none());
    }

    #[test]
    fn position_timeout_faults_on_jam() {
        let mut etb = EtbController::new(EtbConfig::default());
        // Plate stuck at 0 while target is high → timeout after 500 ms
        let mut faulted = false;
        for _ in 0..120 {
            let out = etb.update(100.0, 0.0, Some(0.0), 0.005);
            if out.fault == Some(EtbFault::PositionTimeout) {
                faulted = true;
                break;
            }
        }
        assert!(faulted);
    }
}
