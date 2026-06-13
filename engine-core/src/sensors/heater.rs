//! Wideband lambda sensor heater control.
//!
//! Implements the standard three-phase heater strategy for LSU-type wideband
//! sensors:
//!
//! 1. **Condensation** — low fixed power after engine start so water droplets
//!    in the exhaust cannot crack the hot ceramic element.
//! 2. **Ramp-up** — effective heater voltage ramps up at a bounded rate.
//! 3. **Closed-loop hold** — full target voltage; duty is battery-compensated
//!    (`duty = (V_target / V_batt)² · 100`) so heat output stays constant as
//!    the supply varies.

/// Heater controller configuration.
#[derive(Clone, Copy, Debug)]
pub struct HeaterConfig {
    /// Effective heater voltage during the condensation phase (V).
    pub condensation_volts: f32,
    /// Condensation phase duration after engine start (ms).
    pub condensation_ms: u32,
    /// Ramp rate for the effective voltage (V/s).
    pub ramp_volts_per_s: f32,
    /// Target effective voltage in the hold phase (V, LSU 4.9 nominal ≈ 9 V).
    pub target_volts: f32,
    /// Maximum output duty (%).
    pub max_duty_pct: f32,
}

impl Default for HeaterConfig {
    fn default() -> Self {
        Self {
            condensation_volts: 2.0,
            condensation_ms: 5_000,
            ramp_volts_per_s: 0.4,
            target_volts: 9.0,
            max_duty_pct: 100.0,
        }
    }
}

/// Heater phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaterPhase {
    /// Engine off — heater off.
    Off,
    /// Gentle warm-up to evaporate condensation.
    Condensation,
    /// Voltage ramping toward target.
    Ramp,
    /// At target — battery-compensated hold.
    Hold,
}

/// Wideband heater duty controller.
pub struct HeaterController {
    cfg: HeaterConfig,
    phase: HeaterPhase,
    elapsed_ms: f32,
    effective_volts: f32,
}

impl HeaterController {
    /// Create a controller with the given configuration.
    pub fn new(cfg: HeaterConfig) -> Self {
        Self {
            cfg,
            phase: HeaterPhase::Off,
            elapsed_ms: 0.0,
            effective_volts: 0.0,
        }
    }

    /// Current phase.
    pub fn phase(&self) -> HeaterPhase {
        self.phase
    }

    /// Run one step. Returns the heater PWM duty (%).
    ///
    /// * `engine_running` — heater only runs with the engine turning
    ///   (protects the element and the battery)
    /// * `battery_volts`  — supply voltage for duty compensation
    /// * `dt_s`           — time step in seconds
    pub fn update(&mut self, engine_running: bool, battery_volts: f32, dt_s: f32) -> f32 {
        let dt_s = dt_s.clamp(0.0001, 0.5);
        if !engine_running {
            self.phase = HeaterPhase::Off;
            self.elapsed_ms = 0.0;
            self.effective_volts = 0.0;
            return 0.0;
        }

        self.elapsed_ms += dt_s * 1000.0;
        match self.phase {
            HeaterPhase::Off => {
                self.phase = HeaterPhase::Condensation;
                self.elapsed_ms = 0.0;
                self.effective_volts = self.cfg.condensation_volts;
            }
            HeaterPhase::Condensation => {
                self.effective_volts = self.cfg.condensation_volts;
                if self.elapsed_ms >= self.cfg.condensation_ms as f32 {
                    self.phase = HeaterPhase::Ramp;
                }
            }
            HeaterPhase::Ramp => {
                self.effective_volts += self.cfg.ramp_volts_per_s * dt_s;
                if self.effective_volts >= self.cfg.target_volts {
                    self.effective_volts = self.cfg.target_volts;
                    self.phase = HeaterPhase::Hold;
                }
            }
            HeaterPhase::Hold => {
                self.effective_volts = self.cfg.target_volts;
            }
        }

        duty_for_volts(self.effective_volts, battery_volts, self.cfg.max_duty_pct)
    }
}

/// Battery-compensated duty for a requested effective RMS voltage.
#[inline]
fn duty_for_volts(target_v: f32, battery_v: f32, max_duty: f32) -> f32 {
    if battery_v < 4.0 {
        return 0.0;
    }
    let ratio = target_v / battery_v;
    (ratio * ratio * 100.0).clamp(0.0, max_duty)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn off_when_engine_stopped() {
        let mut h = HeaterController::new(HeaterConfig::default());
        assert_eq!(h.update(false, 14.0, 0.1), 0.0);
        assert_eq!(h.phase(), HeaterPhase::Off);
    }

    #[test]
    fn starts_in_condensation_with_low_duty() {
        let mut h = HeaterController::new(HeaterConfig::default());
        let duty = h.update(true, 14.0, 0.1);
        assert_eq!(h.phase(), HeaterPhase::Condensation);
        // 2 V on 14 V supply ≈ 2 % duty
        assert!(duty < 5.0, "duty = {duty}");
    }

    #[test]
    fn ramps_then_holds_at_target() {
        let mut h = HeaterController::new(HeaterConfig::default());
        // Run 40 s of simulated time at 100 ms steps
        let mut duty = 0.0;
        for _ in 0..400 {
            duty = h.update(true, 14.0, 0.1);
        }
        assert_eq!(h.phase(), HeaterPhase::Hold);
        // 9 V on 14 V supply ≈ 41 % duty
        assert!(
            (duty - (9.0f32 / 14.0).powi(2) * 100.0).abs() < 1.0,
            "duty = {duty}"
        );
    }

    #[test]
    fn duty_compensates_for_battery_sag() {
        let mut h = HeaterController::new(HeaterConfig::default());
        for _ in 0..400 {
            let _ = h.update(true, 14.0, 0.1);
        }
        let duty_14 = h.update(true, 14.0, 0.1);
        let duty_11 = h.update(true, 11.0, 0.1);
        assert!(duty_11 > duty_14);
    }

    #[test]
    fn engine_stop_resets_to_condensation_on_restart() {
        let mut h = HeaterController::new(HeaterConfig::default());
        for _ in 0..400 {
            let _ = h.update(true, 14.0, 0.1);
        }
        assert_eq!(h.phase(), HeaterPhase::Hold);
        let _ = h.update(false, 14.0, 0.1);
        let _ = h.update(true, 14.0, 0.1);
        assert_eq!(h.phase(), HeaterPhase::Condensation);
    }
}
