//! Structured fault (DTC) store and asynchronous event queue (RDP).
//!
//! Sourced from `protection::ProtectionMonitor` and sensor validation; served
//! by `Diagnostics.GetFaults` / `ClearFaults` and pushed as `Kind::Event`
//! frames (see `docs/api/05-telemetry-control-diagnostics.md` §3).

/// Maximum number of distinct fault records retained.
pub const MAX_FAULTS: usize = 16;
/// Maximum queued (not yet pushed) events.
pub const MAX_EVENTS: usize = 8;

// ─── Fault codes ─────────────────────────────────────────────────────────────

/// Well-known fault codes (`Fault.code`).
pub mod fault_code {
    /// CLT sensor short / out-of-range low.
    pub const SENSOR_CLT: u16 = 0x0001;
    /// IAT sensor short / out-of-range.
    pub const SENSOR_IAT: u16 = 0x0002;
    /// MAP reading implausible.
    pub const MAP_IMPLAUSIBLE: u16 = 0x0003;
    /// Engine over-rev.
    pub const OVERREV: u16 = 0x0010;
    /// Overboost.
    pub const OVERBOOST: u16 = 0x0011;
    /// Low oil pressure.
    pub const LOW_OIL_PRESSURE: u16 = 0x0012;
    /// Coolant over-temperature.
    pub const OVER_TEMP: u16 = 0x0013;
    /// Trigger synchronisation lost.
    pub const TRIGGER_SYNC_LOSS: u16 = 0x0020;
    /// Lambda reading implausible.
    pub const LAMBDA_IMPLAUSIBLE: u16 = 0x0021;
    /// Battery voltage low.
    pub const BATTERY_LOW: u16 = 0x0030;
    /// Battery voltage high.
    pub const BATTERY_HIGH: u16 = 0x0031;
}

/// Fault severity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Severity {
    /// Informational only.
    Info = 0,
    /// Degraded but driveable.
    Warn = 1,
    /// Protection action taken.
    Critical = 2,
}

/// One stored fault record.
#[derive(Clone, Copy, Debug)]
pub struct FaultRecord {
    /// Unique fault code (see [`fault_code`]).
    pub code: u16,
    /// Severity at last occurrence.
    pub severity: Severity,
    /// Still present right now.
    pub active: bool,
    /// Occurrence count.
    pub count: u16,
    /// First occurrence (ms since boot).
    pub first_ts_ms: u32,
    /// Latest occurrence (ms since boot).
    pub last_ts_ms: u32,
    /// Context-dependent detail (sensor id, cylinder, …).
    pub detail: u16,
}

/// Fixed-capacity fault store.
#[derive(Default)]
pub struct FaultStore {
    records: heapless::Vec<FaultRecord, MAX_FAULTS>,
}

impl FaultStore {
    /// Create an empty store.
    pub const fn new() -> Self {
        Self { records: heapless::Vec::new() }
    }

    /// Record a fault occurrence. Returns `true` when this is a new
    /// (or re-activated) fault — the caller should emit a `FaultSet` event.
    pub fn raise(&mut self, code: u16, severity: Severity, detail: u16, now_ms: u32) -> bool {
        if let Some(rec) = self.records.iter_mut().find(|r| r.code == code) {
            let newly_active = !rec.active;
            rec.active = true;
            rec.severity = severity;
            rec.count = rec.count.saturating_add(1);
            rec.last_ts_ms = now_ms;
            rec.detail = detail;
            return newly_active;
        }
        let rec = FaultRecord {
            code,
            severity,
            active: true,
            count: 1,
            first_ts_ms: now_ms,
            last_ts_ms: now_ms,
            detail,
        };
        self.records.push(rec).is_ok()
    }

    /// Mark a fault as no longer present (kept in the store for history).
    /// Returns `true` if the fault was active — the caller should emit a
    /// `FaultCleared` event.
    pub fn resolve(&mut self, code: u16) -> bool {
        if let Some(rec) = self.records.iter_mut().find(|r| r.code == code && r.active) {
            rec.active = false;
            return true;
        }
        false
    }

    /// Clear stored faults by index bitmask (`bit i` clears `records[i]`).
    /// Active faults are retained (they would immediately re-raise).
    /// Returns the number of records removed.
    pub fn clear_mask(&mut self, mask: u32) -> u16 {
        let mut cleared = 0u16;
        let mut idx = 0usize;
        self.records.retain(|rec| {
            let keep = rec.active || mask & (1 << idx.min(31)) == 0;
            if !keep {
                cleared += 1;
            }
            idx += 1;
            keep
        });
        cleared
    }

    /// Iterate over stored fault records.
    pub fn iter(&self) -> impl Iterator<Item = &FaultRecord> {
        self.records.iter()
    }

    /// Number of stored records.
    pub fn len(&self) -> usize {
        self.records.len()
    }

    /// True when no faults are stored.
    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }
}

// ─── Asynchronous events ─────────────────────────────────────────────────────

/// Event kinds pushed as `Kind::Event` (see docs §3.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EventKind {
    /// New fault raised (`a` = code, `b` = detail).
    FaultSet = 0,
    /// Fault resolved (`a` = code).
    FaultCleared = 1,
    /// Knock detected (`a` = cylinder, `b` = retard milli-degrees).
    Knock = 2,
    /// Protection ignition/fuel cut (`a` = reason).
    ProtectionCut = 3,
    /// Limp mode transition (`a` = reason).
    LimpMode = 4,
    /// Trigger sync gained (`a`=1) or lost (`a`=0); `b` = tooth count.
    SyncState = 5,
    /// Config changed by another client (`a` = source).
    ConfigChanged = 6,
}

/// One queued event.
#[derive(Clone, Copy, Debug)]
pub struct EventRecord {
    /// Event kind.
    pub kind: EventKind,
    /// Timestamp (ms since boot).
    pub ts_ms: u32,
    /// Kind-dependent payload A.
    pub a: i32,
    /// Kind-dependent payload B.
    pub b: i32,
}

/// FIFO queue of pending push events. Oldest events are dropped on overflow.
#[derive(Default)]
pub struct EventQueue {
    queue: heapless::Deque<EventRecord, MAX_EVENTS>,
}

impl EventQueue {
    /// Create an empty queue.
    pub const fn new() -> Self {
        Self { queue: heapless::Deque::new() }
    }

    /// Queue an event, dropping the oldest entry when full.
    pub fn push(&mut self, kind: EventKind, ts_ms: u32, a: i32, b: i32) {
        let rec = EventRecord { kind, ts_ms, a, b };
        if self.queue.push_back(rec).is_err() {
            let _ = self.queue.pop_front();
            let _ = self.queue.push_back(rec);
        }
    }

    /// Take the next pending event.
    pub fn pop(&mut self) -> Option<EventRecord> {
        self.queue.pop_front()
    }

    /// Number of queued events.
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// True when no events are queued.
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raise_and_count() {
        let mut store = FaultStore::new();
        assert!(store.raise(fault_code::OVER_TEMP, Severity::Critical, 0, 100));
        // Second raise of an already-active fault: no new event
        assert!(!store.raise(fault_code::OVER_TEMP, Severity::Critical, 0, 200));
        let rec = store.iter().next().unwrap();
        assert_eq!(rec.count, 2);
        assert_eq!(rec.first_ts_ms, 100);
        assert_eq!(rec.last_ts_ms, 200);
    }

    #[test]
    fn resolve_then_reraise_signals_event() {
        let mut store = FaultStore::new();
        assert!(store.raise(fault_code::BATTERY_LOW, Severity::Warn, 0, 0));
        assert!(store.resolve(fault_code::BATTERY_LOW));
        assert!(!store.resolve(fault_code::BATTERY_LOW));
        assert!(store.raise(fault_code::BATTERY_LOW, Severity::Warn, 0, 10));
    }

    #[test]
    fn clear_mask_keeps_active() {
        let mut store = FaultStore::new();
        store.raise(fault_code::OVERREV, Severity::Warn, 0, 0);
        store.raise(fault_code::BATTERY_LOW, Severity::Warn, 0, 0);
        store.resolve(fault_code::OVERREV); // index 0 inactive
        let cleared = store.clear_mask(0b11);
        assert_eq!(cleared, 1);
        assert_eq!(store.len(), 1);
        assert_eq!(store.iter().next().unwrap().code, fault_code::BATTERY_LOW);
    }

    #[test]
    fn event_queue_drops_oldest() {
        let mut q = EventQueue::new();
        for i in 0..(MAX_EVENTS as i32 + 2) {
            q.push(EventKind::Knock, i as u32, i, 0);
        }
        assert_eq!(q.len(), MAX_EVENTS);
        assert_eq!(q.pop().unwrap().a, 2); // 0 and 1 dropped
    }
}
