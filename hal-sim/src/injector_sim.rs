//! Simulator injector output — records open/close events for verification.
//!
//! This implementation is intended for PC-side simulation and unit tests.
//! It is **not** `no_std`.

#![cfg(feature = "fuel-fi")]

use rusefi_core::hal::InjectorOutput;

/// A logged injector event from the simulator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InjectorEvent {
    /// Injector opened for the given cylinder.
    Open(u8),
    /// Injector closed for the given cylinder.
    Close(u8),
}

/// Simulator implementation of [`InjectorOutput`].
///
/// Logs all injector open/close events into a `Vec` for post-run analysis.
pub struct SimInjectorOutput {
    pub events: Vec<InjectorEvent>,
    /// Current state of each injector (true = open).
    pub state: [bool; 4],
}

impl SimInjectorOutput {
    /// Create a new injector output recorder.
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            state: [false; 4],
        }
    }

    /// Clear all recorded events and reset state.
    pub fn clear(&mut self) {
        self.events.clear();
        self.state = [false; 4];
    }

    /// Return true if the injector for the given cylinder is currently open.
    pub fn is_open(&self, cylinder: u8) -> bool {
        self.state.get(cylinder as usize).copied().unwrap_or(false)
    }
}

impl Default for SimInjectorOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl InjectorOutput for SimInjectorOutput {
    fn open(&mut self, cylinder: u8) {
        if cylinder as usize >= self.state.len() {
            return; // Ignore invalid cylinder
        }
        self.state[cylinder as usize] = true;
        self.events.push(InjectorEvent::Open(cylinder));
    }

    fn close(&mut self, cylinder: u8) {
        if cylinder as usize >= self.state.len() {
            return; // Ignore invalid cylinder
        }
        self.state[cylinder as usize] = false;
        self.events.push(InjectorEvent::Close(cylinder));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_close_events() {
        let mut inj = SimInjectorOutput::new();
        inj.open(0);
        assert!(inj.is_open(0));
        inj.close(0);
        assert!(!inj.is_open(0));
        assert_eq!(inj.events.len(), 2);
        assert_eq!(inj.events[0], InjectorEvent::Open(0));
        assert_eq!(inj.events[1], InjectorEvent::Close(0));
    }

    #[test]
    fn multiple_cylinders() {
        let mut inj = SimInjectorOutput::new();
        inj.open(0);
        inj.open(1);
        inj.close(0);
        assert!(inj.is_open(1));
        assert!(!inj.is_open(0));
    }

    #[test]
    fn out_of_range_no_panic() {
        let mut inj = SimInjectorOutput::new();
        inj.open(255); // should not panic
        inj.close(255);
        assert_eq!(inj.events.len(), 0); // no events recorded for invalid cylinder
    }
}
