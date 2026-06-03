//! Simulator implementations for PWM and relay outputs.

use rusefi_core::hal::{PwmOutput, RelayOutput};

/// Simulated PWM output — records the commanded duty cycle.
#[derive(Debug, Default)]
pub struct SimPwmOutput {
    duty: f32,
    /// History of duty cycle commands (for test assertions).
    pub history: Vec<f32>,
}

impl SimPwmOutput {
    /// Create a new simulated PWM output at 0% duty.
    pub fn new() -> Self {
        Self::default()
    }
}

impl PwmOutput for SimPwmOutput {
    fn set_duty(&mut self, duty_pct: f32) {
        self.duty = duty_pct.clamp(0.0, 100.0);
        self.history.push(self.duty);
    }

    fn duty(&self) -> f32 {
        self.duty
    }
}

/// Simulated relay output — records on/off state.
#[derive(Debug, Default)]
pub struct SimRelayOutput {
    on: bool,
    /// Total toggle count (for test assertions).
    pub toggle_count: u32,
}

impl SimRelayOutput {
    /// Create a new simulated relay output in the off state.
    pub fn new() -> Self {
        Self::default()
    }
}

impl RelayOutput for SimRelayOutput {
    fn on(&mut self) {
        if !self.on {
            self.toggle_count += 1;
        }
        self.on = true;
    }

    fn off(&mut self) {
        if self.on {
            self.toggle_count += 1;
        }
        self.on = false;
    }

    fn is_on(&self) -> bool {
        self.on
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pwm_clamps_duty() {
        let mut pwm = SimPwmOutput::new();
        pwm.set_duty(150.0);
        assert_eq!(pwm.duty(), 100.0);
        pwm.set_duty(-10.0);
        assert_eq!(pwm.duty(), 0.0);
    }

    #[test]
    fn relay_toggles() {
        let mut relay = SimRelayOutput::new();
        assert!(!relay.is_on());
        relay.on();
        assert!(relay.is_on());
        assert_eq!(relay.toggle_count, 1);
        relay.off();
        assert!(!relay.is_on());
        assert_eq!(relay.toggle_count, 2);
    }
}
