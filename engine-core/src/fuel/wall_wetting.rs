//! Wall wetting (port fuel film) compensation using the Aquino model.
//!
//! Models fuel film dynamics on intake port walls. During cold running,
//! a fraction of injected fuel coats the wall and evaporates slowly,
//! causing transient AFR errors without compensation.
//!
//! Reference: firmware/controllers/algo/fuel/wall_fuel.cpp (Aquino 1981)

use crate::config::MAX_CYLINDERS;
use libm::expf;

/// Wall wetting model configuration.
///
/// Parameters are temperature-interpolated between cold and warm values.
#[derive(Clone, Copy, Debug)]
pub struct WallWettingConfig {
    /// Fraction of injected fuel that coats the wall at cold temperature (0–1).
    pub x_cold: f32,
    /// Fraction of injected fuel that coats the wall at warm temperature (0–1).
    pub x_warm: f32,
    /// Wall film evaporation time constant (seconds) at cold temperature.
    pub tau_cold_s: f32,
    /// Wall film evaporation time constant (seconds) at warm temperature.
    pub tau_warm_s: f32,
    /// Coolant temperature (°C) for "cold" parameter values.
    pub cold_temp_c: f32,
    /// Coolant temperature (°C) for "warm" parameter values.
    pub warm_temp_c: f32,
}

impl Default for WallWettingConfig {
    fn default() -> Self {
        Self {
            x_cold: 0.45,
            x_warm: 0.04,
            tau_cold_s: 5.0,
            tau_warm_s: 0.5,
            cold_temp_c: -20.0,
            warm_temp_c: 80.0,
        }
    }
}

impl WallWettingConfig {
    /// Configuration tuned for a small displacement engine (≤ 500 cc/cyl).
    pub fn small_engine() -> Self {
        Self {
            x_cold: 0.35,
            x_warm: 0.03,
            tau_cold_s: 3.5,
            tau_warm_s: 0.4,
            ..Self::default()
        }
    }
}

/// Wall wetting controller (per cylinder).
///
/// Maintains the wall film mass state and calculates the corrected
/// injection pulse needed to deliver the desired fuel mass to the cylinder.
#[derive(Clone, Copy, Debug)]
pub struct WallWettingController {
    cfg: WallWettingConfig,
    /// Current wall film fuel mass (grams).
    film_g: f32,
}

impl WallWettingController {
    /// Create a new controller with the given configuration.
    pub fn new(cfg: WallWettingConfig) -> Self {
        Self { cfg, film_g: 0.0 }
    }

    /// Wall fraction X interpolated to the current coolant temperature.
    fn x_for_temp(&self, clt_c: f32) -> f32 {
        let t = ((clt_c - self.cfg.cold_temp_c)
            / (self.cfg.warm_temp_c - self.cfg.cold_temp_c))
            .clamp(0.0, 1.0);
        self.cfg.x_cold + (self.cfg.x_warm - self.cfg.x_cold) * t
    }

    /// Evaporation time constant τ interpolated to the current coolant temperature.
    fn tau_for_temp(&self, clt_c: f32) -> f32 {
        let t = ((clt_c - self.cfg.cold_temp_c)
            / (self.cfg.warm_temp_c - self.cfg.cold_temp_c))
            .clamp(0.0, 1.0);
        (self.cfg.tau_cold_s + (self.cfg.tau_warm_s - self.cfg.tau_cold_s) * t).max(0.001)
    }

    /// Calculate the corrected injection pulse (grams) to deliver `desired_g`
    /// to the cylinder, and update the internal wall film state.
    ///
    /// Uses the discrete Aquino model:
    /// - `evap_frac = 1 − exp(−dt / τ)` — fraction of film that evaporates this cycle
    /// - `injected = (desired − film × evap_frac) / (1 − X)`
    /// - `film_new = film × (1 − evap_frac) + X × injected`
    ///
    /// # Arguments
    /// * `desired_g` — target fuel mass to deliver to cylinder (grams)
    /// * `clt_c` — coolant temperature (°C) for parameter interpolation
    /// * `dt_s` — time since last injection event (seconds)
    ///
    /// # Returns
    /// Corrected injection mass (grams) to command to the injector.
    pub fn compensate(&mut self, desired_g: f32, clt_c: f32, dt_s: f32) -> f32 {
        let x = self.x_for_temp(clt_c);
        let tau = self.tau_for_temp(clt_c);

        // Fraction of film that evaporates in this time step
        let evap_frac = 1.0 - expf(-(dt_s / tau));

        // Fuel delivered to cylinder from existing film
        let film_evap = self.film_g * evap_frac;

        // Solve for injection mass needed to deliver desired_g
        let denominator = 1.0 - x;
        let injected = if denominator > 0.001 {
            (desired_g - film_evap) / denominator
        } else {
            desired_g
        };
        let injected = injected.max(0.0);

        // Update wall film state
        let film_new = self.film_g * (1.0 - evap_frac) + x * injected;
        self.film_g = film_new.max(0.0);

        injected
    }

    /// Current wall film mass in grams (for diagnostics).
    pub fn film_mass_g(&self) -> f32 {
        self.film_g
    }

    /// Reset film to zero (e.g., after extended engine-off period).
    pub fn reset(&mut self) {
        self.film_g = 0.0;
    }

    /// Get the current wall film fraction X for the given coolant temperature.
    pub fn wall_fraction(&self, clt_c: f32) -> f32 {
        self.x_for_temp(clt_c)
    }

    /// Get the current evaporation time constant for the given coolant temperature.
    pub fn tau_s(&self, clt_c: f32) -> f32 {
        self.tau_for_temp(clt_c)
    }
}

/// Multi-cylinder wall wetting controller (up to `MAX_CYLINDERS`).
///
/// Each cylinder maintains its own independent wall film state.
#[derive(Clone, Copy, Debug)]
pub struct MultiCylWallWetting {
    controllers: [WallWettingController; MAX_CYLINDERS],
    num_cylinders: u8,
}

impl MultiCylWallWetting {
    /// Create a new multi-cylinder controller.
    pub fn new(cfg: WallWettingConfig, num_cylinders: u8) -> Self {
        let ctrl = WallWettingController::new(cfg);
        Self {
            controllers: [ctrl; MAX_CYLINDERS],
            num_cylinders: num_cylinders.min(MAX_CYLINDERS as u8),
        }
    }

    /// Apply compensation for the specified cylinder.
    pub fn compensate(&mut self, cylinder: u8, desired_g: f32, clt_c: f32, dt_s: f32) -> f32 {
        if cylinder < self.num_cylinders {
            self.controllers[cylinder as usize].compensate(desired_g, clt_c, dt_s)
        } else {
            desired_g
        }
    }

    /// Get wall film mass for a specific cylinder.
    pub fn film_mass_g(&self, cylinder: u8) -> f32 {
        if cylinder < self.num_cylinders {
            self.controllers[cylinder as usize].film_mass_g()
        } else {
            0.0
        }
    }

    /// Reset all cylinder films (e.g., after cold soak).
    pub fn reset_all(&mut self) {
        for ctrl in &mut self.controllers {
            ctrl.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_compensation_warm_engine() {
        let cfg = WallWettingConfig {
            x_warm: 0.0, // No film at warm
            ..WallWettingConfig::default()
        };
        let mut ctrl = WallWettingController::new(cfg);
        // At warm temp with x=0, injected should equal desired
        let injected = ctrl.compensate(0.01, 80.0, 0.1);
        assert!((injected - 0.01).abs() < 0.001);
    }

    #[test]
    fn cold_engine_injects_more() {
        let cfg = WallWettingConfig::default();
        let mut ctrl = WallWettingController::new(cfg);
        // At cold with x=0.45, first injection needs extra fuel for film
        let injected = ctrl.compensate(0.01, -20.0, 0.1);
        assert!(injected > 0.01, "Cold engine should inject more than desired");
    }

    #[test]
    fn film_builds_and_evaporates() {
        let cfg = WallWettingConfig {
            x_cold: 0.5,
            tau_cold_s: 1.0,
            cold_temp_c: 20.0,
            warm_temp_c: 80.0,
            ..WallWettingConfig::default()
        };
        let mut ctrl = WallWettingController::new(cfg);
        // Inject a few times to build film
        for _ in 0..5 {
            ctrl.compensate(0.01, 20.0, 0.1);
        }
        let film_after_injection = ctrl.film_mass_g();
        assert!(film_after_injection > 0.0, "Film should accumulate");

        // After many cycles without injection, film evaporates
        for _ in 0..100 {
            ctrl.compensate(0.0, 20.0, 0.1);
        }
        let film_after_evap = ctrl.film_mass_g();
        assert!(
            film_after_evap < film_after_injection,
            "Film should evaporate over time"
        );
    }

    #[test]
    fn x_interpolation() {
        let cfg = WallWettingConfig::default();
        let ctrl = WallWettingController::new(cfg);

        let x_cold = ctrl.wall_fraction(-20.0);
        let x_warm = ctrl.wall_fraction(80.0);
        let x_mid = ctrl.wall_fraction(30.0);

        assert!((x_cold - cfg.x_cold).abs() < 0.001);
        assert!((x_warm - cfg.x_warm).abs() < 0.001);
        assert!(x_mid > x_warm && x_mid < x_cold);
    }

    #[test]
    fn tau_interpolation() {
        let cfg = WallWettingConfig::default();
        let ctrl = WallWettingController::new(cfg);

        let tau_cold = ctrl.tau_s(-20.0);
        let tau_warm = ctrl.tau_s(80.0);

        assert!((tau_cold - cfg.tau_cold_s).abs() < 0.001);
        assert!((tau_warm - cfg.tau_warm_s).abs() < 0.001);
        assert!(tau_cold > tau_warm);
    }

    #[test]
    fn reset_clears_film() {
        let cfg = WallWettingConfig::default();
        let mut ctrl = WallWettingController::new(cfg);

        ctrl.compensate(0.01, -20.0, 0.1);
        assert!(ctrl.film_mass_g() > 0.0);

        ctrl.reset();
        assert!((ctrl.film_mass_g()).abs() < 0.0001);
    }

    #[test]
    fn multi_cyl_independent() {
        let cfg = WallWettingConfig::default();
        let mut multi = MultiCylWallWetting::new(cfg, 4);

        // Only inject into cylinder 0
        multi.compensate(0, 0.01, -20.0, 0.1);

        // Cylinder 0 should have film, cylinder 1 should not
        assert!(multi.film_mass_g(0) > 0.0);
        assert!((multi.film_mass_g(1)).abs() < 0.0001);
    }
}
