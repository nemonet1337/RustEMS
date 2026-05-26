//! Simulator ignition output: records coil charge/fire events for verification.

use rusefi_core::hal::IgnitionOutput;

/// A logged ignition event from the simulator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IgnitionEvent {
    /// Coil started charging for the given cylinder.
    Charge(u8),
    /// Coil fired for the given cylinder.
    Fire(u8),
}

/// Simulator implementation of [`IgnitionOutput`].
///
/// Logs all coil charge and fire events into a `Vec` for post-run analysis.
pub struct SimIgnitionOutput {
    pub events: Vec<IgnitionEvent>,
}

impl SimIgnitionOutput {
    /// Create a new output recorder.
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    /// Clear all recorded events.
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// Return all fire events in order.
    pub fn fire_events(&self) -> impl Iterator<Item = u8> + '_ {
        self.events.iter().filter_map(|e| {
            if let IgnitionEvent::Fire(cyl) = e {
                Some(*cyl)
            } else {
                None
            }
        })
    }
}

impl Default for SimIgnitionOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl IgnitionOutput for SimIgnitionOutput {
    fn coil_charge(&mut self, cylinder: u8) {
        self.events.push(IgnitionEvent::Charge(cylinder));
    }

    fn coil_fire(&mut self, cylinder: u8) {
        self.events.push(IgnitionEvent::Fire(cylinder));
    }
}
