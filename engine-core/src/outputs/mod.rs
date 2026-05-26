//! Generic output control — fuel pump, cooling fan, and auxiliary outputs.
//!
//! Derived from `firmware/controllers/actuators/`.

/// Fuel pump control state machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FuelPumpState {
    /// Pump is off.
    Off,
    /// Pump is priming (initial startup).
    Priming {
        /// Remaining prime duration in milliseconds.
        remaining_ms: u32,
    },
    /// Pump is running normally.
    Running,
}

/// Fuel pump controller configuration.
#[derive(Clone, Copy, Debug)]
pub struct FuelPumpConfig {
    /// Prime duration in milliseconds (pump runs at key-on).
    pub prime_duration_ms: u32,
    /// RPM threshold below which pump is turned off (safety).
    pub min_rpm: f32,
    /// Delay after RPM drops below min before turning off.
    pub shutdown_delay_ms: u32,
}

impl Default for FuelPumpConfig {
    fn default() -> Self {
        Self {
            prime_duration_ms: 2000, // 2 second prime
            min_rpm: 50.0,           // Turn off below 50 RPM
            shutdown_delay_ms: 500,  // 0.5s delay before shutdown
        }
    }
}

/// Fuel pump controller.
pub struct FuelPumpController {
    cfg: FuelPumpConfig,
    state: FuelPumpState,
    /// Time since last RPM drop (for delayed shutdown).
    shutdown_timer_ms: u32,
}

impl FuelPumpController {
    /// Create a new fuel pump controller.
    pub fn new(cfg: FuelPumpConfig) -> Self {
        Self {
            cfg,
            state: FuelPumpState::Off,
            shutdown_timer_ms: 0,
        }
    }

    /// Call at key-on to start priming.
    pub fn on_key_on(&mut self) {
        self.state = FuelPumpState::Priming {
            remaining_ms: self.cfg.prime_duration_ms,
        };
        self.shutdown_timer_ms = 0;
    }

    /// Update pump state based on RPM and time.
    ///
    /// # Arguments
    /// * `rpm` — Current engine RPM
    /// * `dt_ms` — Time since last update in milliseconds
    ///
    /// # Returns
    /// `true` if pump should be energized.
    pub fn update(&mut self, rpm: f32, dt_ms: u32) -> bool {
        match self.state {
            FuelPumpState::Off => false,
            FuelPumpState::Priming { remaining_ms } => {
                if remaining_ms <= dt_ms {
                    // Prime complete, transition to running
                    if rpm >= self.cfg.min_rpm {
                        self.state = FuelPumpState::Running;
                    } else {
                        self.state = FuelPumpState::Off;
                    }
                    false
                } else {
                    self.state = FuelPumpState::Priming {
                        remaining_ms: remaining_ms - dt_ms,
                    };
                    true
                }
            }
            FuelPumpState::Running => {
                if rpm < self.cfg.min_rpm {
                    // RPM dropped, start shutdown timer
                    self.shutdown_timer_ms += dt_ms;
                    if self.shutdown_timer_ms >= self.cfg.shutdown_delay_ms {
                        self.state = FuelPumpState::Off;
                        self.shutdown_timer_ms = 0;
                        false
                    } else {
                        true // Keep running during delay
                    }
                } else {
                    self.shutdown_timer_ms = 0; // Reset timer
                    true
                }
            }
        }
    }

    /// Current pump state.
    pub fn state(&self) -> FuelPumpState {
        self.state
    }

    /// Reset to off state (e.g., on key-off).
    pub fn reset(&mut self) {
        self.state = FuelPumpState::Off;
        self.shutdown_timer_ms = 0;
    }
}

/// Cooling fan control mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FanMode {
    /// Fan is off.
    Off,
    /// Fan is on.
    On,
    /// PWM duty for variable speed fan (0-100%).
    Pwm(f32),
}

/// Cooling fan controller.
pub struct FanController {
    /// Turn-on temperature threshold (°C).
    on_temp: f32,
    /// Turn-off temperature threshold (°C, hysteresis).
    off_temp: f32,
    state: FanMode,
}

impl FanController {
    /// Create a new fan controller.
    pub fn new(on_temp: f32, off_temp: f32) -> Self {
        assert!(on_temp > off_temp, "on_temp must be > off_temp");
        Self {
            on_temp,
            off_temp,
            state: FanMode::Off,
        }
    }

    /// Create default controller for a typical engine (90°C on, 85°C off).
    pub fn default_engine() -> Self {
        Self::new(90.0, 85.0)
    }

    /// Update fan state based on coolant temperature.
    pub fn update(&mut self, clt_c: f32) -> FanMode {
        match self.state {
            FanMode::Off => {
                if clt_c >= self.on_temp {
                    self.state = FanMode::On;
                }
            }
            FanMode::On | FanMode::Pwm(_) => {
                if clt_c <= self.off_temp {
                    self.state = FanMode::Off;
                }
            }
        }
        self.state
    }

    /// Current fan mode.
    pub fn mode(&self) -> FanMode {
        self.state
    }

    /// Reset to off.
    pub fn reset(&mut self) {
        self.state = FanMode::Off;
    }
}

/// Dual fan controller (for dual-fan setups).
pub struct DualFanController {
    primary: FanController,
    secondary: FanController,
    /// Temperature at which secondary fan engages.
    secondary_on_temp: f32,
}

impl DualFanController {
    /// Create dual fan controller.
    pub fn new(primary_on: f32, primary_off: f32, secondary_on: f32) -> Self {
        Self {
            primary: FanController::new(primary_on, primary_off),
            secondary: FanController::new(secondary_on, primary_off),
            secondary_on_temp: secondary_on,
        }
    }

    /// Create default dual fan setup (primary 90/85°C, secondary 95°C).
    pub fn default_dual() -> Self {
        Self::new(90.0, 85.0, 95.0)
    }

    /// Update both fans based on temperature.
    /// Returns (primary_mode, secondary_mode).
    pub fn update(&mut self, clt_c: f32) -> (FanMode, FanMode) {
        let primary = self.primary.update(clt_c);
        let secondary = if clt_c >= self.secondary_on_temp {
            self.secondary.update(clt_c)
        } else {
            self.secondary.reset();
            FanMode::Off
        };
        (primary, secondary)
    }

    /// Reset both fans.
    pub fn reset(&mut self) {
        self.primary.reset();
        self.secondary.reset();
    }
}

/// Tachometer (tacho) output controller.
/// Drives a square-wave signal proportional to engine RPM.
pub struct TachometerOutput {
    /// Pulses per revolution (typically 2 or 4 for wasted/sequential spark).
    pulses_per_rev: u32,
    /// Current output state (for simulation/testing).
    pub active: bool,
}

impl TachometerOutput {
    /// Create a new tachometer output.
    ///
    /// * `pulses_per_rev` — Number of tacho pulses per engine revolution
    ///   (e.g. 2 for wasted spark, 4 for full sequential on 4-cylinder).
    pub fn new(pulses_per_rev: u32) -> Self {
        Self {
            pulses_per_rev,
            active: false,
        }
    }

    /// Default tachometer for a 4-cylinder wasted-spark engine (2 pulses/rev).
    pub fn default_4cyl_wasted() -> Self {
        Self::new(2)
    }

    /// Update tacho output based on RPM.
    ///
    /// Returns the frequency in Hz that the tacho output should toggle at.
    /// Returns `None` when engine is stopped (RPM = 0).
    pub fn update(&mut self, rpm: f32) -> Option<f32> {
        if rpm <= 0.0 {
            self.active = false;
            None
        } else {
            self.active = true;
            // Frequency = RPM/60 * pulses_per_rev
            Some(rpm / 60.0 * self.pulses_per_rev as f32)
        }
    }

    /// Current frequency in Hz.
    pub fn frequency_hz(&self, rpm: f32) -> Option<f32> {
        if rpm <= 0.0 {
            None
        } else {
            Some(rpm / 60.0 * self.pulses_per_rev as f32)
        }
    }
}

/// Generic PWM output controller with 8x8 lookup table.
/// Maps two input axes (e.g. RPM vs TPS) to a PWM duty cycle.
pub struct GenericPwmOutput {
    /// 8x8 lookup table for duty cycle (0-100%).
    /// Row axis: input_a (e.g. RPM), Column axis: input_b (e.g. TPS).
    table: [[f32; 8]; 8],
    /// Axis breakpoints for input_a (row axis).
    axis_a: [f32; 8],
    /// Axis breakpoints for input_b (column axis).
    axis_b: [f32; 8],
    /// Current output duty cycle (0-100%).
    pub current_duty: f32,
}

impl GenericPwmOutput {
    /// Create a new generic PWM output with default linear axes.
    pub fn new() -> Self {
        Self {
            table: [[0.0; 8]; 8],
            axis_a: [0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0],
            axis_b: [0.0, 12.5, 25.0, 37.5, 50.0, 62.5, 75.0, 100.0],
            current_duty: 0.0,
        }
    }

    /// Set the lookup table value.
    pub fn set_table_value(&mut self, row: usize, col: usize, duty: f32) {
        assert!(row < 8 && col < 8, "row and col must be 0..7");
        self.table[row][col] = duty.clamp(0.0, 100.0);
    }

    /// Set axis breakpoints.
    pub fn set_axis_a(&mut self, values: [f32; 8]) {
        self.axis_a = values;
    }

    /// Set axis B breakpoints.
    pub fn set_axis_b(&mut self, values: [f32; 8]) {
        self.axis_b = values;
    }

    /// Find index for interpolation: returns (idx, next_idx, ratio).
    fn find_index(axis: &[f32; 8], value: f32) -> (usize, usize, f32) {
        if value <= axis[0] {
            return (0, 0, 0.0);
        }
        if value >= axis[7] {
            return (7, 7, 0.0);
        }
        for i in 0..7 {
            if value >= axis[i] && value < axis[i + 1] {
                let ratio = (value - axis[i]) / (axis[i + 1] - axis[i]);
                return (i, i + 1, ratio);
            }
        }
        (7, 7, 0.0)
    }

    /// Update duty cycle based on two input values using bilinear interpolation.
    pub fn update(&mut self, input_a: f32, input_b: f32) -> f32 {
        let (ra, ra_next, ra_ratio) = Self::find_index(&self.axis_a, input_a);
        let (ca, ca_next, ca_ratio) = Self::find_index(&self.axis_b, input_b);

        // Bilinear interpolation
        let v00 = self.table[ra][ca];
        let v01 = self.table[ra][ca_next];
        let v10 = self.table[ra_next][ca];
        let v11 = self.table[ra_next][ca_next];

        let v0 = v00 + (v01 - v00) * ca_ratio;
        let v1 = v10 + (v11 - v10) * ca_ratio;

        self.current_duty = v0 + (v1 - v0) * ra_ratio;
        self.current_duty = self.current_duty.clamp(0.0, 100.0);
        self.current_duty
    }

    /// Current duty cycle.
    pub fn duty(&self) -> f32 {
        self.current_duty
    }
}

impl Default for GenericPwmOutput {
    fn default() -> Self {
        Self::new()
    }
}

/// AC (air conditioning) interlocked fan controller.
/// Activates the cooling fan when AC clutch is engaged or coolant is hot.
pub struct AcFanController {
    base_fan: FanController,
    /// AC clutch engaged flag.
    ac_engaged: bool,
    /// Fan-on temperature when AC is engaged (typically lower than base).
    ac_on_temp: f32,
}

impl AcFanController {
    /// Create a new AC-interlocked fan controller.
    ///
    /// * `base_on_temp` — Normal fan-on temperature
    /// * `base_off_temp` — Normal fan-off temperature (hysteresis)
    /// * `ac_on_temp` — Fan-on temperature when AC is engaged (usually lower)
    pub fn new(base_on_temp: f32, base_off_temp: f32, ac_on_temp: f32) -> Self {
        assert!(base_on_temp > base_off_temp, "on_temp must be > off_temp");
        Self {
            base_fan: FanController::new(base_on_temp, base_off_temp),
            ac_engaged: false,
            ac_on_temp,
        }
    }

    /// Default AC fan controller (90/85°C base, 80°C AC-on).
    pub fn default_engine() -> Self {
        Self::new(90.0, 85.0, 80.0)
    }

    /// Set AC clutch engagement state.
    pub fn set_ac_engaged(&mut self, engaged: bool) {
        self.ac_engaged = engaged;
    }

    /// Update fan state based on coolant temperature and AC state.
    ///
    /// When AC is engaged, the fan activates at a lower temperature threshold.
    pub fn update(&mut self, clt_c: f32) -> FanMode {
        let effective_on_temp = if self.ac_engaged {
            self.ac_on_temp
        } else {
            self.base_fan.on_temp
        };

        // Use base fan logic but with adjusted on_temp when AC is on
        match self.base_fan.mode() {
            FanMode::Off => {
                if clt_c >= effective_on_temp {
                    self.base_fan.state = FanMode::On;
                }
            }
            FanMode::On | FanMode::Pwm(_) => {
                if clt_c <= self.base_fan.off_temp {
                    self.base_fan.state = FanMode::Off;
                }
            }
        }
        self.base_fan.state
    }

    /// Current fan mode.
    pub fn mode(&self) -> FanMode {
        self.base_fan.mode()
    }

    /// Reset to off.
    pub fn reset(&mut self) {
        self.base_fan.reset();
        self.ac_engaged = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fuel_pump_primes_then_stops() {
        let cfg = FuelPumpConfig {
            prime_duration_ms: 100,
            min_rpm: 50.0,
            shutdown_delay_ms: 50,
        };
        let mut pump = FuelPumpController::new(cfg);

        // Key on: priming starts
        pump.on_key_on();
        assert!(matches!(pump.state(), FuelPumpState::Priming { .. }));
        assert!(pump.update(0.0, 50)); // Still priming

        // Prime complete, no RPM: should turn off
        assert!(!pump.update(0.0, 60));
        assert_eq!(pump.state(), FuelPumpState::Off);
    }

    #[test]
    fn fuel_pump_runs_with_engine() {
        let cfg = FuelPumpConfig {
            prime_duration_ms: 50,
            min_rpm: 50.0,
            shutdown_delay_ms: 100,
        };
        let mut pump = FuelPumpController::new(cfg);

        pump.on_key_on();
        pump.update(0.0, 60); // Complete prime

        // Engine starts during prime
        assert!(pump.update(800.0, 10));
        assert_eq!(pump.state(), FuelPumpState::Running);

        // Keep running
        assert!(pump.update(800.0, 10));

        // RPM drops briefly, still running (delay)
        assert!(pump.update(0.0, 50));
        assert_eq!(pump.state(), FuelPumpState::Running);

        // After delay, turn off
        assert!(!pump.update(0.0, 60));
        assert_eq!(pump.state(), FuelPumpState::Off);
    }

    #[test]
    fn fan_hysteresis() {
        let mut fan = FanController::new(90.0, 85.0);

        // Below threshold: off
        assert_eq!(fan.update(80.0), FanMode::Off);

        // Cross threshold: on
        assert_eq!(fan.update(95.0), FanMode::On);

        // Drop but stay above off_temp: still on
        assert_eq!(fan.update(87.0), FanMode::On);

        // Drop below off_temp: off
        assert_eq!(fan.update(84.0), FanMode::Off);
    }

    #[test]
    fn fan_default_engine() {
        let fan = FanController::default_engine();
        assert_eq!(fan.mode(), FanMode::Off);
    }

    #[test]
    fn fan_reset() {
        let mut fan = FanController::new(90.0, 85.0);
        fan.update(95.0);
        assert_eq!(fan.mode(), FanMode::On);

        fan.reset();
        assert_eq!(fan.mode(), FanMode::Off);
    }

    // Dual fan controller tests
    #[test]
    fn dual_fan_basic_operation() {
        let mut fans = DualFanController::new(90.0, 85.0, 95.0);

        // Below all thresholds: both off
        let (p, s) = fans.update(80.0);
        assert_eq!(p, FanMode::Off);
        assert_eq!(s, FanMode::Off);

        // Above primary, below secondary: primary on
        let (p, s) = fans.update(92.0);
        assert_eq!(p, FanMode::On);
        assert_eq!(s, FanMode::Off);

        // Above secondary: both on
        let (p, s) = fans.update(97.0);
        assert_eq!(p, FanMode::On);
        assert_eq!(s, FanMode::On);
    }

    #[test]
    fn dual_fan_hysteresis() {
        let mut fans = DualFanController::new(90.0, 85.0, 95.0);

        // Heat up
        fans.update(97.0);
        let (p, s) = fans.update(97.0);
        assert_eq!(p, FanMode::On);
        assert_eq!(s, FanMode::On);

        // Cool down below secondary: secondary off immediately
        let (p, s) = fans.update(93.0);
        assert_eq!(p, FanMode::On);
        assert_eq!(s, FanMode::Off);

        // Cool down below primary off: primary off
        let (p, s) = fans.update(83.0);
        assert_eq!(p, FanMode::Off);
        assert_eq!(s, FanMode::Off);
    }

    // Tachometer tests
    #[test]
    fn tachometer_basic() {
        let mut tacho = TachometerOutput::new(2);

        // Engine stopped
        assert_eq!(tacho.update(0.0), None);
        assert!(!tacho.active);

        // 6000 RPM, 2 pulses/rev = 200 Hz
        assert_eq!(tacho.update(6000.0), Some(200.0));
        assert!(tacho.active);

        // 3000 RPM = 100 Hz
        assert_eq!(tacho.update(3000.0), Some(100.0));
    }

    #[test]
    fn tachometer_different_configs() {
        let mut tacho4 = TachometerOutput::new(4); // 4 pulses/rev

        // 3000 RPM, 4 pulses/rev = 200 Hz
        assert_eq!(tacho4.update(3000.0), Some(200.0));

        let mut tacho_default = TachometerOutput::default_4cyl_wasted();
        // 3000 RPM, 2 pulses/rev = 100 Hz
        assert_eq!(tacho_default.update(3000.0), Some(100.0));
    }

    #[test]
    fn tachometer_frequency_hz() {
        let tacho = TachometerOutput::new(2);
        assert_eq!(tacho.frequency_hz(6000.0), Some(200.0));
        assert_eq!(tacho.frequency_hz(0.0), None);
    }

    // Generic PWM (GPPWM) tests
    #[test]
    fn gppwm_basic_lookup() {
        let mut gppwm = GenericPwmOutput::new();

        // Set a simple value
        gppwm.set_table_value(0, 0, 50.0);

        // Test at exact breakpoint (0 RPM, 0% TPS)
        let duty = gppwm.update(0.0, 0.0);
        assert_eq!(duty, 50.0);
        assert_eq!(gppwm.duty(), 50.0);
    }

    #[test]
    fn gppwm_interpolation() {
        let mut gppwm = GenericPwmOutput::new();

        // Set up a gradient table
        for i in 0..8 {
            for j in 0..8 {
                gppwm.set_table_value(i, j, (i * 10 + j) as f32);
            }
        }

        // Test interpolation at midpoint
        let duty = gppwm.update(1500.0, 18.75); // Halfway between breakpoints
        assert!(duty > 0.0 && duty < 100.0);
    }

    #[test]
    fn gppwm_clamp() {
        let mut gppwm = GenericPwmOutput::new();
        gppwm.set_table_value(0, 0, 150.0); // Should clamp to 100
        assert_eq!(gppwm.table[0][0], 100.0);
    }

    #[test]
    fn gppwm_axis_customization() {
        let mut gppwm = GenericPwmOutput::new();

        let custom_axis_a = [0.0, 500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 4000.0];
        gppwm.set_axis_a(custom_axis_a);
        assert_eq!(gppwm.axis_a[1], 500.0);
    }

    // AC Fan controller tests
    #[test]
    fn ac_fan_normal_operation() {
        let mut ac_fan = AcFanController::new(90.0, 85.0, 80.0);

        // AC off, normal temp: fan off
        assert_eq!(ac_fan.update(87.0), FanMode::Off);

        // AC off, hot temp: fan on
        assert_eq!(ac_fan.update(92.0), FanMode::On);
    }

    #[test]
    fn ac_fan_ac_engaged() {
        let mut ac_fan = AcFanController::new(90.0, 85.0, 80.0);

        // AC off, 87°C: fan off (normal threshold 90°C)
        assert_eq!(ac_fan.update(87.0), FanMode::Off);

        // AC on, 87°C: fan on (AC threshold 80°C)
        ac_fan.set_ac_engaged(true);
        assert_eq!(ac_fan.update(87.0), FanMode::On);
    }

    #[test]
    fn ac_fan_reset() {
        let mut ac_fan = AcFanController::new(90.0, 85.0, 80.0);
        ac_fan.set_ac_engaged(true);
        ac_fan.update(95.0);
        assert_eq!(ac_fan.mode(), FanMode::On);

        ac_fan.reset();
        assert_eq!(ac_fan.mode(), FanMode::Off);
        assert!(!ac_fan.ac_engaged);
    }

    #[test]
    fn ac_fan_default() {
        let _ac_fan = AcFanController::default_engine();
    }

    #[test]
    fn dual_fan_default() {
        let _fans = DualFanController::default_dual();
        // Just verify it creates successfully
    }

    #[test]
    fn dual_fan_reset() {
        let mut fans = DualFanController::new(90.0, 85.0, 95.0);
        fans.update(97.0);
        fans.reset();
        let (p, s) = fans.update(80.0);
        assert_eq!(p, FanMode::Off);
        assert_eq!(s, FanMode::Off);
    }

    // Fuel pump additional tests
    #[test]
    fn fuel_pump_reset() {
        let cfg = FuelPumpConfig::default();
        let mut pump = FuelPumpController::new(cfg);

        pump.on_key_on();
        assert!(matches!(pump.state(), FuelPumpState::Priming { .. }));

        pump.reset();
        assert_eq!(pump.state(), FuelPumpState::Off);
    }

    #[test]
    fn fuel_pump_prime_with_running_engine() {
        let cfg = FuelPumpConfig {
            prime_duration_ms: 100,
            min_rpm: 50.0,
            shutdown_delay_ms: 50,
        };
        let mut pump = FuelPumpController::new(cfg);

        pump.on_key_on();

        // Engine starts during prime
        assert!(pump.update(1000.0, 50));
        assert!(matches!(pump.state(), FuelPumpState::Priming { .. }));

        // Prime completes with engine running -> transition to running
        assert!(pump.update(1000.0, 60));
        assert_eq!(pump.state(), FuelPumpState::Running);
    }

    #[test]
    fn fuel_pump_stays_off_when_engine_stopped() {
        let cfg = FuelPumpConfig {
            prime_duration_ms: 50,
            min_rpm: 50.0,
            shutdown_delay_ms: 100,
        };
        let mut pump = FuelPumpController::new(cfg);

        pump.on_key_on();
        pump.update(0.0, 60); // Complete prime without engine
        assert_eq!(pump.state(), FuelPumpState::Off);

        // Should stay off
        assert!(!pump.update(0.0, 10));
        assert_eq!(pump.state(), FuelPumpState::Off);
    }

    #[test]
    fn fuel_pump_rpm_bouncing() {
        let cfg = FuelPumpConfig {
            prime_duration_ms: 50,
            min_rpm: 50.0,
            shutdown_delay_ms: 100,
        };
        let mut pump = FuelPumpController::new(cfg);

        pump.on_key_on();
        pump.update(1000.0, 60); // Prime + running

        // RPM bouncing around threshold
        assert!(pump.update(1000.0, 10)); // Running
        assert!(pump.update(30.0, 10)); // Below threshold, timer starts
        assert!(pump.update(1000.0, 10)); // Back above, timer resets
        assert!(pump.update(30.0, 10)); // Below again
        assert!(pump.update(30.0, 50)); // Still in delay
        assert!(!pump.update(30.0, 60)); // Delay exceeded, turn off
    }
}
