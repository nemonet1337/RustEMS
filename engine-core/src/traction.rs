//! Traction control — wheel-slip detection with progressive torque reduction.
//!
//! Compares driven and undriven wheel speeds; above the slip threshold the
//! controller requests progressive ignition retard, and above the cut
//! threshold it requests spark cut, ramping back in smoothly once grip is
//! recovered.

/// Traction control configuration.
#[derive(Clone, Copy, Debug)]
pub struct TractionConfig {
    /// Master enable.
    pub enabled: bool,
    /// Slip (%) where intervention starts.
    pub slip_threshold_pct: f32,
    /// Slip (%) where full intervention (spark cut) is reached.
    pub slip_cut_pct: f32,
    /// Maximum ignition retard applied at full intervention (degrees).
    pub max_retard_deg: f32,
    /// Minimum undriven wheel speed for activation (km/h) — avoids false
    /// triggers at walking pace.
    pub min_speed_kmh: f32,
    /// Recovery ramp rate for the retard (degrees per second).
    pub recovery_deg_per_s: f32,
}

impl Default for TractionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            slip_threshold_pct: 8.0,
            slip_cut_pct: 25.0,
            max_retard_deg: 12.0,
            min_speed_kmh: 5.0,
            recovery_deg_per_s: 20.0,
        }
    }
}

/// One traction-control decision.
#[derive(Clone, Copy, Debug, Default)]
pub struct TractionAction {
    /// Measured slip (%).
    pub slip_pct: f32,
    /// Ignition retard to apply (degrees, ≥ 0).
    pub retard_deg: f32,
    /// Request spark cut this cycle.
    pub spark_cut: bool,
    /// Intervention in progress (retard or cut).
    pub active: bool,
}

/// Wheel-slip traction controller.
pub struct TractionController {
    cfg: TractionConfig,
    current_retard: f32,
}

impl TractionController {
    /// Create a controller with the given configuration.
    pub fn new(cfg: TractionConfig) -> Self {
        Self {
            cfg,
            current_retard: 0.0,
        }
    }

    /// Run one step.
    ///
    /// * `undriven_kmh` — reference (undriven axle) wheel speed
    /// * `driven_kmh`   — driven axle wheel speed
    /// * `dt_s`         — time step in seconds
    pub fn update(&mut self, undriven_kmh: f32, driven_kmh: f32, dt_s: f32) -> TractionAction {
        let dt_s = dt_s.clamp(0.0001, 0.5);
        if !self.cfg.enabled || undriven_kmh < self.cfg.min_speed_kmh {
            self.current_retard = 0.0;
            return TractionAction::default();
        }

        let slip_pct = ((driven_kmh - undriven_kmh) / undriven_kmh * 100.0).max(0.0);

        if slip_pct >= self.cfg.slip_cut_pct {
            // Severe slip: full retard and spark cut.
            self.current_retard = self.cfg.max_retard_deg;
            return TractionAction {
                slip_pct,
                retard_deg: self.current_retard,
                spark_cut: true,
                active: true,
            };
        }

        if slip_pct > self.cfg.slip_threshold_pct {
            // Progressive retard proportional to slip depth.
            let span = self.cfg.slip_cut_pct - self.cfg.slip_threshold_pct;
            let depth = (slip_pct - self.cfg.slip_threshold_pct) / span;
            let demand = depth * self.cfg.max_retard_deg;
            // Retard applies immediately, recovery ramps out below.
            if demand > self.current_retard {
                self.current_retard = demand;
            }
        } else {
            // Grip recovered: ramp retard out smoothly.
            self.current_retard =
                (self.current_retard - self.cfg.recovery_deg_per_s * dt_s).max(0.0);
        }

        TractionAction {
            slip_pct,
            retard_deg: self.current_retard,
            spark_cut: false,
            active: self.current_retard > 0.0,
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_action_without_slip() {
        let mut tc = TractionController::new(TractionConfig::default());
        let a = tc.update(60.0, 61.0, 0.01); // 1.7 % slip — under threshold
        assert!(!a.active);
        assert_eq!(a.retard_deg, 0.0);
        assert!(!a.spark_cut);
    }

    #[test]
    fn progressive_retard_with_slip() {
        let mut tc = TractionController::new(TractionConfig::default());
        // ~16.5 % slip: between threshold (8) and cut (25)
        let a = tc.update(60.0, 70.0, 0.01);
        assert!(a.active);
        assert!(a.retard_deg > 0.0 && a.retard_deg < 12.0);
        assert!(!a.spark_cut);
    }

    #[test]
    fn severe_slip_cuts_spark() {
        let mut tc = TractionController::new(TractionConfig::default());
        let a = tc.update(60.0, 80.0, 0.01); // 33 % slip
        assert!(a.spark_cut);
        assert_eq!(a.retard_deg, 12.0);
    }

    #[test]
    fn retard_ramps_out_after_recovery() {
        let mut tc = TractionController::new(TractionConfig::default());
        let _ = tc.update(60.0, 75.0, 0.01); // build retard
        let first = tc.update(60.0, 60.0, 0.1); // grip recovered
        assert!(first.retard_deg > 0.0);
        let mut last = first.retard_deg;
        for _ in 0..20 {
            let a = tc.update(60.0, 60.0, 0.1);
            assert!(a.retard_deg <= last);
            last = a.retard_deg;
        }
        assert_eq!(last, 0.0);
    }

    #[test]
    fn inactive_below_min_speed_and_when_disabled() {
        let mut tc = TractionController::new(TractionConfig::default());
        let a = tc.update(2.0, 10.0, 0.01); // huge slip but walking pace
        assert!(!a.active);

        let mut tc = TractionController::new(TractionConfig {
            enabled: false,
            ..TractionConfig::default()
        });
        let a = tc.update(60.0, 90.0, 0.01);
        assert!(!a.active);
        assert!(!a.spark_cut);
    }
}
