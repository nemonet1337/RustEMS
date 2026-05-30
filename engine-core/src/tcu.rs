//! Transmission Control Unit (TCU).
//!
//! Manages automatic transmission control including:
//! - Gear selection
//! - Shift timing
//! - Clutch control (for DCT/automated manuals)
//! - Torque converter lockup
//! - Transmission protection

use crate::sensors::SensorData;

/// Gear position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gear {
    /// Park.
    Park,
    /// Reverse.
    Reverse,
    /// Neutral.
    Neutral,
    /// Drive.
    Drive,
    /// Manual gear 1.
    First,
    /// Manual gear 2.
    Second,
    /// Manual gear 3.
    Third,
    /// Manual gear 4.
    Fourth,
    /// Manual gear 5.
    Fifth,
    /// Manual gear 6.
    Sixth,
}

impl Gear {
    /// Get gear ratio (simplified, actual ratios depend on transmission).
    pub fn ratio(&self) -> f32 {
        match self {
            Gear::Park | Gear::Neutral => 0.0,
            Gear::Reverse => -3.5,
            Gear::Drive => 1.0, // Varies with actual gear
            Gear::First => 3.5,
            Gear::Second => 2.0,
            Gear::Third => 1.3,
            Gear::Fourth => 1.0,
            Gear::Fifth => 0.8,
            Gear::Sixth => 0.7,
        }
    }

    /// Check if gear is forward driving gear.
    pub fn is_forward(&self) -> bool {
        matches!(self, Gear::Drive | Gear::First | Gear::Second | Gear::Third | Gear::Fourth | Gear::Fifth | Gear::Sixth)
    }

    /// Check if gear is reverse.
    pub fn is_reverse(&self) -> bool {
        matches!(self, Gear::Reverse)
    }

    /// Check if transmission is in neutral.
    pub fn is_neutral(&self) -> bool {
        matches!(self, Gear::Neutral)
    }
}

/// Shift mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftMode {
    /// Automatic mode (ECU selects gears).
    Automatic,
    /// Manual mode (driver selects gears).
    Manual,
    /// Sport mode (higher shift points).
    Sport,
    /// Economy mode (lower shift points).
    Economy,
}

/// Shift direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftDirection {
    /// Upshift to higher gear.
    Up,
    /// Downshift to lower gear.
    Down,
}

/// TCU state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcuState {
    /// Normal operation.
    Normal,
    /// Shift in progress.
    Shifting,
    /// Protection mode (limp mode for transmission).
    Protection,
    /// Fault detected.
    Fault,
}

/// TCU configuration.
#[derive(Clone, Copy, Debug)]
pub struct TcuConfig {
    /// RPM threshold for upshift (automatic mode).
    pub upshift_rpm: f32,
    /// RPM threshold for downshift (automatic mode).
    pub downshift_rpm: f32,
    /// Minimum vehicle speed for upshift (km/h).
    pub min_upshift_speed: f32,
    /// Maximum vehicle speed for first gear (km/h).
    pub max_first_gear_speed: f32,
    /// Torque converter lockup speed (km/h).
    pub lockup_speed: f32,
    /// Shift duration (ms).
    pub shift_duration_ms: u32,
}

impl Default for TcuConfig {
    fn default() -> Self {
        Self {
            upshift_rpm: 6000.0,
            downshift_rpm: 2500.0,
            min_upshift_speed: 20.0,
            max_first_gear_speed: 40.0,
            lockup_speed: 60.0,
            shift_duration_ms: 200,
        }
    }
}

/// Transmission Control Unit.
pub struct Tcu {
    cfg: TcuConfig,
    current_gear: Gear,
    target_gear: Gear,
    mode: ShiftMode,
    state: TcuState,
    shift_timer_ms: u32,
}

impl Tcu {
    /// Create a new TCU with default configuration.
    pub fn new() -> Self {
        Self::with_config(TcuConfig::default())
    }

    /// Create a new TCU with custom configuration.
    pub fn with_config(cfg: TcuConfig) -> Self {
        Self {
            cfg,
            current_gear: Gear::Neutral,
            target_gear: Gear::Neutral,
            // Default to manual control; automatic shifting is opted into via
            // set_mode(ShiftMode::Automatic).
            mode: ShiftMode::Manual,
            state: TcuState::Normal,
            shift_timer_ms: 0,
        }
    }

    /// Set shift mode.
    pub fn set_mode(&mut self, mode: ShiftMode) {
        self.mode = mode;
    }

    /// Request gear change.
    ///
    /// # Arguments
    /// * `gear` - Target gear
    pub fn request_gear(&mut self, gear: Gear) {
        if self.state == TcuState::Normal {
            self.target_gear = gear;
            if gear != self.current_gear {
                // Engaging from a disengaged state (Park/Neutral) is immediate;
                // a gear-to-gear shift takes shift_duration_ms.
                if self.current_gear.is_neutral() || matches!(self.current_gear, Gear::Park) {
                    self.current_gear = gear;
                } else {
                    self.state = TcuState::Shifting;
                    self.shift_timer_ms = self.cfg.shift_duration_ms;
                }
            }
        }
    }

    /// Request upshift.
    pub fn upshift(&mut self) {
        let next_gear = match self.current_gear {
            Gear::First => Gear::Second,
            Gear::Second => Gear::Third,
            Gear::Third => Gear::Fourth,
            Gear::Fourth => Gear::Fifth,
            Gear::Fifth => Gear::Sixth,
            Gear::Sixth => Gear::Sixth, // Already in highest gear
            _ => return, // Can't upshift from Park/Reverse/Neutral
        };
        self.request_gear(next_gear);
    }

    /// Request downshift.
    pub fn downshift(&mut self) {
        let prev_gear = match self.current_gear {
            Gear::Second => Gear::First,
            Gear::Third => Gear::Second,
            Gear::Fourth => Gear::Third,
            Gear::Fifth => Gear::Fourth,
            Gear::Sixth => Gear::Fifth,
            Gear::First => Gear::First, // Already in lowest gear
            _ => return, // Can't downshift from Park/Reverse/Neutral
        };
        self.request_gear(prev_gear);
    }

    /// Update TCU state based on sensor data.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor data
    /// * `dt_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// Current gear ratio.
    pub fn update(&mut self, sensors: &SensorData, dt_ms: u32) -> f32 {
        let rpm = sensors.rpm.unwrap_or(0.0);

        // Handle shift in progress
        if self.state == TcuState::Shifting {
            self.shift_timer_ms = self.shift_timer_ms.saturating_sub(dt_ms);

            if self.shift_timer_ms == 0 {
                self.current_gear = self.target_gear;
                self.state = TcuState::Normal;
            }
        }

        // Automatic mode gear selection (simplified, RPM-based only)
        if self.mode == ShiftMode::Automatic && self.state == TcuState::Normal {
            if self.current_gear.is_forward() && rpm > 0.0 {
                // Upshift
                if rpm > self.cfg.upshift_rpm {
                    self.upshift();
                }
                // Downshift
                else if rpm < self.cfg.downshift_rpm {
                    self.downshift();
                }
            }
        }

        self.current_gear.ratio()
    }

    /// Get current gear.
    pub fn current_gear(&self) -> Gear {
        self.current_gear
    }

    /// Get target gear (during shift).
    pub fn target_gear(&self) -> Gear {
        self.target_gear
    }

    /// Get current shift mode.
    pub fn mode(&self) -> ShiftMode {
        self.mode
    }

    /// Get current TCU state.
    pub fn state(&self) -> TcuState {
        self.state
    }

    /// Check if shift is in progress.
    pub fn is_shifting(&self) -> bool {
        self.state == TcuState::Shifting
    }

    /// Get the configuration.
    pub fn config(&self) -> TcuConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: TcuConfig) {
        self.cfg = cfg;
    }
}

impl Default for Tcu {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gear_ratios() {
        assert!(Gear::First.ratio() > Gear::Second.ratio());
        assert!(Gear::Second.ratio() > Gear::Third.ratio());
        assert!(Gear::Reverse.ratio() < 0.0);
        assert_eq!(Gear::Neutral.ratio(), 0.0);
    }

    #[test]
    fn gear_classification() {
        assert!(Gear::First.is_forward());
        assert!(Gear::Drive.is_forward());
        assert!(Gear::Reverse.is_reverse());
        assert!(Gear::Neutral.is_neutral());
    }

    #[test]
    fn upshift_sequence() {
        let mut tcu = Tcu::new();
        let sensors = SensorData {
            rpm: Some(6500.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        tcu.request_gear(Gear::First);
        tcu.update(&sensors, 10);

        assert_eq!(tcu.current_gear(), Gear::First);

        tcu.upshift();
        assert!(tcu.is_shifting());
        assert_eq!(tcu.target_gear(), Gear::Second);

        // Complete shift
        for _ in 0..25 {
            tcu.update(&sensors, 10);
        }

        assert_eq!(tcu.current_gear(), Gear::Second);
        assert!(!tcu.is_shifting());
    }

    #[test]
    fn downshift_sequence() {
        let mut tcu = Tcu::new();
        let sensors = SensorData {
            rpm: Some(2000.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        tcu.request_gear(Gear::Third);
        tcu.update(&sensors, 10);

        tcu.downshift();
        assert_eq!(tcu.target_gear(), Gear::Second);
    }

    #[test]
    fn automatic_mode_upshift() {
        let mut tcu = Tcu::new();
        let sensors = SensorData {
            rpm: Some(6500.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        tcu.request_gear(Gear::First);
        tcu.update(&sensors, 10);

        // Automatic upshift at high RPM
        tcu.set_mode(ShiftMode::Automatic);
        tcu.update(&sensors, 10);

        assert_eq!(tcu.target_gear(), Gear::Second);
    }

    #[test]
    fn automatic_mode_downshift() {
        let mut tcu = Tcu::new();
        let sensors = SensorData {
            rpm: Some(2000.0),
            load_pct: None,
            clt_celsius: None,
            iat_celsius: None,
            tps_pct: None,
            map_kpa: None,
            battery_volts: None,
            maf_voltage: None,
            fuel_level_pct: None,
            oil_pressure_kpa: None,
            lambda1_voltage: None,
            lambda2_voltage: None,
        };

        tcu.request_gear(Gear::Third);
        tcu.update(&sensors, 10);

        tcu.set_mode(ShiftMode::Automatic);
        tcu.update(&sensors, 10);

        assert_eq!(tcu.target_gear(), Gear::Second);
    }

    #[test]
    fn cannot_shift_from_park() {
        let mut tcu = Tcu::new();
        let sensors = SensorData::default();

        tcu.request_gear(Gear::Park);
        tcu.update(&sensors, 10);

        tcu.upshift();
        assert_eq!(tcu.target_gear(), Gear::Park);
    }

    #[test]
    fn shift_mode_change() {
        let mut tcu = Tcu::new();
        tcu.set_mode(ShiftMode::Sport);
        assert_eq!(tcu.mode(), ShiftMode::Sport);
    }
}
