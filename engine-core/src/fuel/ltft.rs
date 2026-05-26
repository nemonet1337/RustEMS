//! Long Term Fuel Trim (LTFT) implementation.
//!
//! LTFT adjusts fuel trim based on long-term oxygen sensor feedback.
//! Learned values are persisted to non-volatile storage.

#![cfg(feature = "fuel-fi")]

use crate::sensors::{SensorData, LambdaSensor, LambdaSensorConfig};

/// Number of load/RPM cells for LTFT learning.
const LTFT_CELLS: usize = 16;

/// LTFT cell configuration.
#[derive(Clone, Copy, Debug)]
pub struct LtftCell {
    /// Learned trim value (percentage, e.g., 1.05 = +5% fuel).
    pub trim: f32,
    /// Number of samples contributing to this trim.
    pub sample_count: u32,
    /// True if this cell is valid (has enough samples).
    pub valid: bool,
}

impl Default for LtftCell {
    fn default() -> Self {
        Self {
            trim: 1.0,
            sample_count: 0,
            valid: false,
        }
    }
}

/// LTFT learning configuration.
#[derive(Clone, Copy, Debug)]
pub struct LtftConfig {
    /// Minimum samples required to mark a cell as valid.
    pub min_samples: u32,
    /// Maximum trim adjustment per sample (percentage).
    pub max_adjustment_pct: f32,
    /// Trim learning rate (0.0-1.0, higher = faster learning).
    pub learning_rate: f32,
    /// Maximum allowed trim (percentage, e.g., 1.30 = +30%).
    pub max_trim: f32,
    /// Minimum allowed trim (percentage, e.g., 0.70 = -30%).
    pub min_trim: f32,
}

impl Default for LtftConfig {
    fn default() -> Self {
        Self {
            min_samples: 50,
            max_adjustment_pct: 0.5,
            learning_rate: 0.1,
            max_trim: 1.30,
            min_trim: 0.70,
        }
    }
}

/// LTFT learning state.
#[derive(Clone, Debug)]
pub struct LtftState {
    /// Trim cells indexed by (load_index * 4 + rpm_index).
    /// Load: 0=low, 1=medium, 2=high, 3=WOT
    /// RPM: 0=idle, 1=low, 2=medium, 3=high
    cells: [LtftCell; LTFT_CELLS],
    /// Configuration.
    cfg: LtftConfig,
}

impl LtftState {
    /// Create a new LTFT state with default configuration.
    pub fn new() -> Self {
        Self::with_config(LtftConfig::default())
    }

    /// Create a new LTFT state with custom configuration.
    pub fn with_config(cfg: LtftConfig) -> Self {
        Self {
            cells: [LtftCell::default(); LTFT_CELLS],
            cfg,
        }
    }

    /// Get the LTFT cell index for given operating conditions.
    ///
    /// Returns index 0-15.
    fn get_cell_index(rpm: f32, load_pct: f32) -> usize {
        let rpm_idx = if rpm < 800.0 {
            0 // idle
        } else if rpm < 2500.0 {
            1 // low
        } else if rpm < 4500.0 {
            2 // medium
        } else {
            3 // high
        };

        let load_idx = if load_pct < 30.0 {
            0 // low load
        } else if load_pct < 60.0 {
            1 // medium load
        } else if load_pct < 90.0 {
            2 // high load
        } else {
            3 // WOT
        };

        load_idx * 4 + rpm_idx
    }

    /// Update LTFT based on oxygen sensor feedback.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor data including lambda voltage and load
    /// * `target_lambda` - Target lambda for current conditions
    ///
    /// # Returns
    /// The current trim value for the operating cell.
    pub fn update(&mut self, sensors: &SensorData, target_lambda: f32) -> f32 {
        let Some(rpm) = sensors.rpm else { return 1.0 };
        let Some(load_pct) = sensors.load_pct else { return 1.0 };
        let Some(lambda_voltage) = sensors.lambda1_voltage else { return 1.0 };

        // Convert voltage to lambda value (wideband sensor)
        let lambda_sensor = LambdaSensor::new(LambdaSensorConfig::default());
        let Some(measured_lambda) = lambda_sensor.voltage_to_lambda(lambda_voltage) else { return 1.0 };

        // Skip learning during unstable conditions
        if rpm < 500.0 || load_pct < 10.0 {
            return 1.0;
        }

        let cell_idx = Self::get_cell_index(rpm, load_pct);
        let cell = &mut self.cells[cell_idx];

        // Calculate error: lambda_ratio = measured / target
        // If measured > target (lean), need more fuel (trim > 1.0)
        let lambda_error = measured_lambda / target_lambda;

        // Adjust trim based on error
        let adjustment = (lambda_error - 1.0).clamp(
            -self.cfg.max_adjustment_pct / 100.0,
            self.cfg.max_adjustment_pct / 100.0,
        );

        // Apply learning rate
        let new_trim = cell.trim + adjustment * self.cfg.learning_rate;

        // Clamp to limits
        cell.trim = new_trim.clamp(self.cfg.min_trim, self.cfg.max_trim);
        cell.sample_count += 1;

        // Mark as valid if enough samples
        if cell.sample_count >= self.cfg.min_samples {
            cell.valid = true;
        }

        cell.trim
    }

    /// Get the current trim for given operating conditions.
    ///
    /// Returns 1.0 (no trim) if cell is not valid.
    pub fn get_trim(&self, rpm: f32, load_pct: f32) -> f32 {
        let cell_idx = Self::get_cell_index(rpm, load_pct);
        let cell = &self.cells[cell_idx];

        if cell.valid {
            cell.trim
        } else {
            1.0
        }
    }

    /// Reset all LTFT cells to default.
    pub fn reset(&mut self) {
        self.cells = [LtftCell::default(); LTFT_CELLS];
    }

    /// Get the configuration.
    pub fn config(&self) -> LtftConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: LtftConfig) {
        self.cfg = cfg;
    }

    /// Get all cells for persistence.
    pub fn cells(&self) -> &[LtftCell; LTFT_CELLS] {
        &self.cells
    }

    /// Restore cells from persisted data.
    pub fn restore_cells(&mut self, cells: [LtftCell; LTFT_CELLS]) {
        self.cells = cells;
    }
}

impl Default for LtftState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ltft_cell_index() {
        // Idle RPM, low load
        assert_eq!(LtftState::get_cell_index(600.0, 20.0), 0);
        // Low RPM, medium load
        assert_eq!(LtftState::get_cell_index(1500.0, 50.0), 5);
        // High RPM, WOT
        assert_eq!(LtftState::get_cell_index(6000.0, 95.0), 15);
    }

    #[test]
    fn ltft_learning_lean_condition() {
        let mut ltft = LtftState::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(2000.0);
        sensors.load_pct = Some(40.0);
        sensors.lambda1_voltage = Some(0.5); // Lean (2.5V = stoich, 0V = 2.0 lambda)

        // Update multiple times to accumulate samples
        for _ in 0..100 {
            ltft.update(&sensors, 1.0);
        }

        let trim = ltft.get_trim(2000.0, 40.0);
        assert!(trim > 1.0, "Lean condition should increase trim");
        assert!(trim < 1.30, "Trim should be clamped to max");
    }

    #[test]
    fn ltft_learning_rich_condition() {
        let mut ltft = LtftState::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(2000.0);
        sensors.load_pct = Some(40.0);
        sensors.lambda1_voltage = Some(4.5); // Rich (2.5V = stoich, 5V = 0.5 lambda)

        for _ in 0..100 {
            ltft.update(&sensors, 1.0);
        }

        let trim = ltft.get_trim(2000.0, 40.0);
        assert!(trim < 1.0, "Rich condition should decrease trim");
        assert!(trim > 0.70, "Trim should be clamped to min");
    }

    #[test]
    fn ltft_reset() {
        let mut ltft = LtftState::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(2000.0);
        sensors.load_pct = Some(40.0);
        sensors.lambda1_voltage = Some(0.5); // Lean

        for _ in 0..100 {
            ltft.update(&sensors, 1.0);
        }

        ltft.reset();
        let trim = ltft.get_trim(2000.0, 40.0);
        assert_eq!(trim, 1.0, "Reset should return trim to default");
    }

    #[test]
    fn ltft_unstable_conditions() {
        let mut ltft = LtftState::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(300.0); // Below idle threshold
        sensors.load_pct = Some(40.0);
        sensors.lambda1_voltage = Some(0.5); // Lean

        let trim = ltft.update(&sensors, 1.0);
        assert_eq!(trim, 1.0, "Unstable RPM should skip learning");
    }

    #[test]
    fn ltft_cell_persistence() {
        let mut ltft = LtftState::new();
        let mut sensors = SensorData::default();
        sensors.rpm = Some(2000.0);
        sensors.load_pct = Some(40.0);
        sensors.lambda1_voltage = Some(0.5); // Lean

        for _ in 0..100 {
            ltft.update(&sensors, 1.0);
        }

        let cells = *ltft.cells();
        let mut ltft2 = LtftState::new();
        ltft2.restore_cells(cells);

        assert_eq!(ltft.get_trim(2000.0, 40.0), ltft2.get_trim(2000.0, 40.0));
    }
}
