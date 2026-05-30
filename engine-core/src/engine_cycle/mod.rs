//! Engine cycle management for sequential injection and ignition.
//!
//! Tracks cylinder positions and schedules fuel/ignition events based on
//! crank angle and cam phase for full sequential operation.

use crate::trigger::{CyclePosition, TriggerState};
use libm::fmodf;

/// Normalize angle to 0-720° range (Euclidean modulus).
#[inline]
fn normalize_angle_720(deg: f32) -> f32 {
    let r = fmodf(deg, 720.0);
    if r < 0.0 {
        r + 720.0
    } else {
        r
    }
}

/// Cylinder state in the 4-stroke engine cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CylinderState {
    /// Intake stroke (180-360° BTDC compression).
    Intake,
    /// Compression stroke (0-180° BTDC TDC).
    Compression,
    /// Power stroke (180-0° ATDC TDC).
    Power,
    /// Exhaust stroke (360-180° BTDC intake).
    Exhaust,
}

impl CylinderState {
    /// Get cylinder state from absolute crank angle (0-720°).
    pub fn from_absolute_angle(absolute_deg: f32) -> Self {
        // Normalize to 0-720. Boundaries are half-open ([start, end)) so that
        // exact multiples of 180° advance into the next stroke.
        let deg = normalize_angle_720(absolute_deg);
        if deg < 180.0 {
            CylinderState::Intake
        } else if deg < 360.0 {
            CylinderState::Compression
        } else if deg < 540.0 {
            CylinderState::Power
        } else {
            CylinderState::Exhaust
        }
    }
}

/// Sequential injection controller.
///
/// Tracks which cylinder is on intake stroke for scheduling sequential injection.
#[derive(Clone, Copy, Debug)]
pub struct SequentialController {
    /// Offset angle for each cylinder's TDC (0-based).
    /// Cylinder N fires at `tdc_offsets[N]` degrees.
    tdc_offsets: [f32; 4], // Max 4 cylinders for now
    /// Current TDC offset for each cylinder (cached).
    num_cylinders: u8,
}

impl SequentialController {
    /// Create a new sequential controller with the given firing order.
    ///
    /// # Arguments
    /// * `firing_order` - Array of 0-based cylinder indices in firing order
    pub fn new(firing_order: &[u8]) -> Self {
        let num_cyl = firing_order.len().min(4) as u8;

        // Calculate TDC offsets based on firing order
        // For a 4-stroke engine, firing events are spaced 720°/num_cylinders apart
        let mut tdc_offsets = [0.0f32; 4];
        let fire_interval = 720.0 / num_cyl as f32;

        for (i, &cyl) in firing_order.iter().enumerate() {
            if cyl < 4 {
                // Cylinder's TDC power stroke occurs at i * fire_interval
                tdc_offsets[cyl as usize] = i as f32 * fire_interval;
            }
        }

        Self {
            tdc_offsets,
            num_cylinders: num_cyl,
        }
    }

    /// Get the TDC angle offset for a specific cylinder.
    pub fn tdc_angle_for_cylinder(&self, cylinder: u8) -> f32 {
        if cylinder < self.num_cylinders {
            self.tdc_offsets[cylinder as usize]
        } else {
            0.0
        }
    }

    /// Calculate the current crank angle (0-720°) from trigger state.
    pub fn current_crank_angle(&self, trigger: &TriggerState) -> f32 {
        normalize_angle_720(trigger.angle_deg)
    }

    /// Get the cylinder currently on intake stroke.
    ///
    /// Returns `Some(cylinder_index)` if sequential mode is ready,
    /// `None` otherwise.
    pub fn current_intake_cylinder(&self, trigger: &TriggerState) -> Option<u8> {
        if !trigger.is_sequential_ready() {
            return None;
        }

        let cycle_pos = trigger.cycle_position?;

        // Find cylinder on intake stroke. The intake window for a cylinder is
        // the 180° span starting at its TDC offset: [tdc, tdc + 180°).
        for cyl in 0..self.num_cylinders {
            let tdc = self.tdc_offsets[cyl as usize];
            let intake_start = tdc;
            let intake_end = normalize_angle_720(tdc + 180.0);

            let current_angle = self.current_crank_angle(trigger);

            // Check if current angle is in this cylinder's intake window
            let in_intake = if intake_start > intake_end {
                // Wraps around 720° boundary
                current_angle >= intake_start || current_angle < intake_end
            } else {
                current_angle >= intake_start && current_angle < intake_end
            };

            if in_intake && cycle_pos == CyclePosition::Intake {
                return Some(cyl);
            }
        }

        None
    }

    /// Get the next cylinder to fire (power stroke).
    pub fn next_firing_cylinder(&self, trigger: &TriggerState) -> Option<u8> {
        if !trigger.is_sequential_ready() {
            return None;
        }

        let current_angle = self.current_crank_angle(trigger);

        // Find cylinder whose TDC is closest and upcoming
        let mut next_cyl = None;
        let mut min_angle_to_tdc = 720.0f32;

        for cyl in 0..self.num_cylinders {
            let tdc = self.tdc_offsets[cyl as usize];
            let angle_to_tdc = normalize_angle_720(tdc - current_angle);

            if angle_to_tdc < min_angle_to_tdc {
                min_angle_to_tdc = angle_to_tdc;
                next_cyl = Some(cyl);
            }
        }

        next_cyl
    }
}

/// Full sequential injection scheduler.
///
/// Determines which cylinder should be injecting fuel based on the
/// current engine position.
pub struct SequentialInjection {
    controller: SequentialController,
    /// True when sequential mode is active (cam synced).
    sequential_active: bool,
    /// Bank angle for staged injection (injection starts X degrees BTDC intake).
    injection_start_deg: f32,
}

impl SequentialInjection {
    /// Create a new sequential injection scheduler.
    ///
    /// # Arguments
    /// * `firing_order` - Cylinder firing order (0-based indices)
    /// * `injection_start_deg` - Crank angle BTDC intake to start injection
    pub fn new(firing_order: &[u8], injection_start_deg: f32) -> Self {
        Self {
            controller: SequentialController::new(firing_order),
            sequential_active: false,
            injection_start_deg,
        }
    }

    /// Update with new trigger state and determine injection scheduling.
    ///
    /// Returns `Some(cylinder)` if injection should start for that cylinder.
    pub fn update(&mut self, trigger: &TriggerState) -> Option<u8> {
        // Check if we have full sequential sync
        self.sequential_active = trigger.is_sequential_ready();

        if !self.sequential_active {
            return None;
        }

        let current_angle = normalize_angle_720(trigger.angle_deg + self.injection_start_deg);

        for cyl in 0..self.controller.num_cylinders {
            let tdc = self.controller.tdc_offsets[cyl as usize];
            let intake_start = tdc;
            let intake_end = normalize_angle_720(tdc + 180.0);

            let in_window = if intake_start > intake_end {
                current_angle >= intake_start || current_angle < intake_end
            } else {
                current_angle >= intake_start && current_angle < intake_end
            };

            if in_window {
                return Some(cyl);
            }
        }

        None
    }

    /// Returns true if sequential injection is active.
    pub fn is_sequential(&self) -> bool {
        self.sequential_active
    }

    /// Switch to batch injection mode (fallback when cam sync is lost).
    pub fn set_batch_mode(&mut self) {
        self.sequential_active = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trigger::SyncState;

    #[test]
    fn sequential_4cyl_firing_order() {
        // 1-3-4-2 firing order (0-based: 0, 2, 3, 1)
        let controller = SequentialController::new(&[0, 2, 3, 1]);

        // Cylinder 0 TDC at 0°
        assert!((controller.tdc_angle_for_cylinder(0) - 0.0).abs() < 0.1);
        // Cylinder 2 TDC at 180°
        assert!((controller.tdc_angle_for_cylinder(2) - 180.0).abs() < 0.1);
        // Cylinder 3 TDC at 360°
        assert!((controller.tdc_angle_for_cylinder(3) - 360.0).abs() < 0.1);
        // Cylinder 1 TDC at 540°
        assert!((controller.tdc_angle_for_cylinder(1) - 540.0).abs() < 0.1);
    }

    #[test]
    fn cylinder_state_from_angle() {
        assert_eq!(CylinderState::from_absolute_angle(0.0), CylinderState::Intake);
        assert_eq!(CylinderState::from_absolute_angle(90.0), CylinderState::Intake);
        assert_eq!(CylinderState::from_absolute_angle(180.0), CylinderState::Compression);
        assert_eq!(CylinderState::from_absolute_angle(270.0), CylinderState::Compression);
        assert_eq!(CylinderState::from_absolute_angle(360.0), CylinderState::Power);
        assert_eq!(CylinderState::from_absolute_angle(450.0), CylinderState::Power);
        assert_eq!(CylinderState::from_absolute_angle(540.0), CylinderState::Exhaust);
        assert_eq!(CylinderState::from_absolute_angle(630.0), CylinderState::Exhaust);
    }

    #[test]
    fn current_intake_cylinder_uses_absolute_cycle_angle() {
        let controller = SequentialController::new(&[0, 2, 3, 1]);
        let trigger = TriggerState {
            tooth_index: 9,
            sync: SyncState::FullSync,
            rpm: Some(1500.0),
            angle_deg: 90.0,
            cam_phase: false,
            cycle_position: Some(CyclePosition::Intake),
            current_cylinder: None,
        };

        assert_eq!(controller.current_intake_cylinder(&trigger), Some(0));
    }

    #[test]
    fn sequential_injection_applies_start_offset() {
        let mut scheduler = SequentialInjection::new(&[0, 2, 3, 1], 30.0);
        let trigger = TriggerState {
            tooth_index: 9,
            sync: SyncState::FullSync,
            rpm: Some(2000.0),
            angle_deg: 520.0,
            cam_phase: true,
            cycle_position: Some(CyclePosition::Exhaust),
            current_cylinder: None,
        };

        assert_eq!(scheduler.update(&trigger), Some(1));
        assert!(scheduler.is_sequential());
    }
}
