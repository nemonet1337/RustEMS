//! Catalog-driven subscription telemetry (RDP).
//!
//! Replaces the fixed 20-byte `OutputChannels` polling block with named,
//! self-describing channels and rate-limited packed streams
//! (see `docs/api/05-telemetry-control-diagnostics.md`).

use crate::comms::output::OutputChannels;
use crate::params::{fnv1a, FNV_OFFSET};

// ─── Channel catalog ─────────────────────────────────────────────────────────

/// Physical wire encoding for one telemetry channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireType {
    /// Unsigned 16-bit, `physical = raw * scale`.
    U16,
    /// Signed 16-bit, `physical = raw * scale`.
    I16,
    /// Single bit, packed into trailing flag bytes.
    Bit,
}

impl WireType {
    /// Bytes occupied in the packed frame (bits are packed separately).
    pub const fn wire_len(self) -> usize {
        match self {
            WireType::U16 | WireType::I16 => 2,
            WireType::Bit => 0,
        }
    }
}

/// Static metadata for one telemetry channel (RDP `ChannelDesc`).
pub struct ChannelMeta {
    /// Stable wire ID.
    pub id: u16,
    /// Machine-readable key, e.g. `"rpm"`.
    pub key: &'static str,
    /// Human-readable label.
    pub label: &'static str,
    /// Physical unit string.
    pub unit: &'static str,
    /// `physical = raw * scale` for integer wire types.
    pub scale: f32,
    /// Wire encoding.
    pub wire: WireType,
    /// UI grouping index.
    pub group: u8,
}

/// Static telemetry channel catalog served by `Descriptor.GetTelemetryCatalog`.
pub const TELEMETRY_CATALOG: &[ChannelMeta] = &[
    ChannelMeta {
        id: 1,
        key: "rpm",
        label: "Engine speed",
        unit: "RPM",
        scale: 1.0,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 2,
        key: "clt_c",
        label: "Coolant temp",
        unit: "°C",
        scale: 0.1,
        wire: WireType::I16,
        group: 1,
    },
    ChannelMeta {
        id: 3,
        key: "iat_c",
        label: "Intake air temp",
        unit: "°C",
        scale: 0.1,
        wire: WireType::I16,
        group: 1,
    },
    ChannelMeta {
        id: 4,
        key: "map_kpa",
        label: "Manifold pressure",
        unit: "kPa",
        scale: 0.1,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 5,
        key: "tps_pct",
        label: "Throttle position",
        unit: "%",
        scale: 0.1,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 6,
        key: "battery_v",
        label: "Battery voltage",
        unit: "V",
        scale: 0.01,
        wire: WireType::U16,
        group: 2,
    },
    ChannelMeta {
        id: 7,
        key: "lambda",
        label: "Lambda",
        unit: "λ",
        scale: 0.001,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 8,
        key: "inj_pulse_ms",
        label: "Injector pulse",
        unit: "ms",
        scale: 0.01,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 9,
        key: "advance_deg",
        label: "Ignition advance",
        unit: "deg",
        scale: 0.1,
        wire: WireType::I16,
        group: 0,
    },
    ChannelMeta {
        id: 10,
        key: "oil_pressure",
        label: "Oil pressure",
        unit: "kPa",
        scale: 0.1,
        wire: WireType::U16,
        group: 2,
    },
    ChannelMeta {
        id: 11,
        key: "fuel_level",
        label: "Fuel level",
        unit: "%",
        scale: 0.1,
        wire: WireType::U16,
        group: 2,
    },
    ChannelMeta {
        id: 12,
        key: "knock_retard",
        label: "Knock retard",
        unit: "deg",
        scale: 0.1,
        wire: WireType::I16,
        group: 0,
    },
    ChannelMeta {
        id: 13,
        key: "cl_correction",
        label: "Closed-loop trim",
        unit: "x",
        scale: 0.001,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 14,
        key: "ltft",
        label: "Long-term fuel trim",
        unit: "x",
        scale: 0.001,
        wire: WireType::U16,
        group: 0,
    },
    ChannelMeta {
        id: 15,
        key: "iac_duty",
        label: "Idle valve duty",
        unit: "%",
        scale: 0.1,
        wire: WireType::U16,
        group: 3,
    },
    ChannelMeta {
        id: 16,
        key: "boost_duty",
        label: "Wastegate duty",
        unit: "%",
        scale: 0.1,
        wire: WireType::U16,
        group: 3,
    },
    ChannelMeta {
        id: 32,
        key: "spark_cut",
        label: "Spark cut",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
    ChannelMeta {
        id: 33,
        key: "sequential",
        label: "Sequential mode",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
    ChannelMeta {
        id: 34,
        key: "dfco",
        label: "DFCO active",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
    ChannelMeta {
        id: 35,
        key: "fuel_pump",
        label: "Fuel pump relay",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
    ChannelMeta {
        id: 36,
        key: "fan",
        label: "Cooling fan relay",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
    ChannelMeta {
        id: 37,
        key: "limp",
        label: "Limp mode",
        unit: "",
        scale: 1.0,
        wire: WireType::Bit,
        group: 4,
    },
];

/// Find the catalog entry for a channel.
pub fn channel_meta(id: u16) -> Option<&'static ChannelMeta> {
    TELEMETRY_CATALOG.iter().find(|c| c.id == id)
}

/// Read a channel's current physical value from a telemetry snapshot.
pub fn channel_value(oc: &OutputChannels, id: u16) -> Option<f32> {
    let v = match id {
        1 => oc.rpm,
        2 => oc.clt_c,
        3 => oc.iat_c,
        4 => oc.map_kpa,
        5 => oc.tps_pct,
        6 => oc.battery_v,
        7 => oc.lambda,
        8 => oc.inj_pulse_ms,
        9 => oc.advance_deg,
        10 => oc.oil_pressure_kpa,
        11 => oc.fuel_level_pct,
        12 => oc.knock_retard_deg,
        13 => oc.cl_correction,
        14 => oc.ltft_correction,
        15 => oc.iac_duty_pct,
        16 => oc.boost_duty_pct,
        32 => bool_f(oc.spark_cut),
        33 => bool_f(oc.sequential),
        34 => bool_f(oc.dfco_active),
        35 => bool_f(oc.fuel_pump_on),
        36 => bool_f(oc.fan_on),
        37 => bool_f(oc.limp_active),
        _ => return None,
    };
    Some(v)
}

#[inline]
fn bool_f(b: bool) -> f32 {
    if b {
        1.0
    } else {
        0.0
    }
}

/// Deterministic hash of the telemetry catalog (folded into `schema_hash`).
pub fn telemetry_hash() -> u32 {
    let mut h = FNV_OFFSET;
    for c in TELEMETRY_CATALOG {
        h = fnv1a(h, &c.id.to_le_bytes());
        h = fnv1a(h, c.key.as_bytes());
        h = fnv1a(h, &c.scale.to_bits().to_le_bytes());
        h = fnv1a(h, &[c.wire as u8, c.group]);
    }
    h
}

// ─── Subscription streams ────────────────────────────────────────────────────

/// Maximum number of concurrent telemetry streams.
pub const MAX_STREAMS: usize = 4;
/// Maximum channels per stream.
pub const MAX_STREAM_CHANNELS: usize = 16;
/// Maximum supported push rate.
pub const MAX_RATE_HZ: u16 = 100;

struct Stream {
    channels: heapless::Vec<u16, MAX_STREAM_CHANNELS>,
    interval_ms: u32,
    next_due_ms: u32,
    seq: u16,
}

/// Subscribe failure reasons.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubscribeError {
    /// All stream slots are in use.
    NoFreeStream,
    /// A channel ID does not exist (carries the offending ID).
    BadChannel(u16),
    /// More channels requested than [`MAX_STREAM_CHANNELS`].
    TooManyChannels,
}

/// Manager for active telemetry subscriptions.
#[derive(Default)]
pub struct TelemetryStreams {
    slots: [Option<Stream>; MAX_STREAMS],
}

impl TelemetryStreams {
    /// Create an empty stream table.
    pub const fn new() -> Self {
        Self {
            slots: [None, None, None, None],
        }
    }

    /// Register a new stream. Returns `(stream_id, actual_rate_hz)`.
    pub fn subscribe(
        &mut self,
        channels: &[u16],
        rate_hz: u16,
        now_ms: u32,
    ) -> Result<(u8, u16), SubscribeError> {
        if channels.len() > MAX_STREAM_CHANNELS {
            return Err(SubscribeError::TooManyChannels);
        }
        let mut layout = heapless::Vec::new();
        for &ch in channels {
            if channel_meta(ch).is_none() {
                return Err(SubscribeError::BadChannel(ch));
            }
            let _ = layout.push(ch);
        }
        let rate = rate_hz.clamp(1, MAX_RATE_HZ);
        let slot = self
            .slots
            .iter_mut()
            .enumerate()
            .find(|(_, s)| s.is_none())
            .ok_or(SubscribeError::NoFreeStream)?;
        let interval_ms = 1000 / rate as u32;
        *slot.1 = Some(Stream {
            channels: layout,
            interval_ms,
            next_due_ms: now_ms,
            seq: 0,
        });
        // Stream IDs are 1-based on the wire (0 is reserved).
        Ok((slot.0 as u8 + 1, rate))
    }

    /// Remove a stream. Returns `false` for an unknown ID.
    pub fn unsubscribe(&mut self, stream_id: u8) -> bool {
        let Some(idx) = (stream_id as usize).checked_sub(1) else {
            return false;
        };
        match self.slots.get_mut(idx) {
            Some(slot @ Some(_)) => {
                *slot = None;
                true
            }
            _ => false,
        }
    }

    /// Drop all streams (e.g. on transport disconnect).
    pub fn clear(&mut self) {
        for slot in self.slots.iter_mut() {
            *slot = None;
        }
    }

    /// Number of active streams.
    pub fn active(&self) -> usize {
        self.slots.iter().filter(|s| s.is_some()).count()
    }

    /// Encode the next due `TelemFrame` body into `out`, if any stream is due.
    ///
    /// Body layout (after the 3-byte message header, which the caller writes):
    /// `stream_id(u8) seq(u16 LE) ts_ms(u32 LE) data...` where `data` packs the
    /// integer channels in layout order followed by bit channels packed
    /// LSB-first into trailing bytes.
    pub fn encode_due_frame(
        &mut self,
        oc: &OutputChannels,
        now_ms: u32,
        out: &mut [u8],
    ) -> Option<usize> {
        for (idx, slot) in self.slots.iter_mut().enumerate() {
            let Some(stream) = slot else { continue };
            if now_ms < stream.next_due_ms {
                continue;
            }
            stream.next_due_ms = now_ms.wrapping_add(stream.interval_ms);
            let frame_len = 7 + packed_data_len(&stream.channels);
            if out.len() < frame_len {
                return None;
            }
            out[0] = idx as u8 + 1;
            out[1..3].copy_from_slice(&stream.seq.to_le_bytes());
            out[3..7].copy_from_slice(&now_ms.to_le_bytes());
            stream.seq = stream.seq.wrapping_add(1);

            let mut pos = 7usize;
            let mut bit_acc = 0u8;
            let mut bit_count = 0u8;
            for &ch in stream.channels.iter() {
                let Some(meta) = channel_meta(ch) else {
                    continue;
                };
                let value = channel_value(oc, ch).unwrap_or(0.0);
                match meta.wire {
                    WireType::U16 => {
                        let raw = (value / meta.scale).clamp(0.0, 65_535.0) as u16;
                        out[pos..pos + 2].copy_from_slice(&raw.to_le_bytes());
                        pos += 2;
                    }
                    WireType::I16 => {
                        let raw = (value / meta.scale).clamp(-32_768.0, 32_767.0) as i16;
                        out[pos..pos + 2].copy_from_slice(&raw.to_le_bytes());
                        pos += 2;
                    }
                    WireType::Bit => {
                        if value != 0.0 {
                            bit_acc |= 1 << bit_count;
                        }
                        bit_count += 1;
                        if bit_count == 8 {
                            out[pos] = bit_acc;
                            pos += 1;
                            bit_acc = 0;
                            bit_count = 0;
                        }
                    }
                }
            }
            if bit_count > 0 {
                out[pos] = bit_acc;
                pos += 1;
            }
            return Some(pos);
        }
        None
    }
}

/// Length of the packed `data` section for a channel layout.
pub fn packed_data_len(channels: &[u16]) -> usize {
    let mut bytes = 0usize;
    let mut bits = 0usize;
    for &ch in channels {
        if let Some(meta) = channel_meta(ch) {
            match meta.wire {
                WireType::Bit => bits += 1,
                w => bytes += w.wire_len(),
            }
        }
    }
    bytes + bits.div_ceil(8)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_ids_unique() {
        for (i, a) in TELEMETRY_CATALOG.iter().enumerate() {
            for b in TELEMETRY_CATALOG.iter().skip(i + 1) {
                assert_ne!(a.id, b.id);
            }
        }
    }

    #[test]
    fn every_channel_readable() {
        let oc = OutputChannels::zeroed();
        for c in TELEMETRY_CATALOG {
            assert!(
                channel_value(&oc, c.id).is_some(),
                "channel {} unreadable",
                c.key
            );
        }
    }

    #[test]
    fn subscribe_and_frame_layout() {
        let mut streams = TelemetryStreams::new();
        let (id, rate) = streams.subscribe(&[1, 2, 32, 34], 25, 0).unwrap();
        assert_eq!(id, 1);
        assert_eq!(rate, 25);

        let mut oc = OutputChannels::zeroed();
        oc.rpm = 3000.0;
        oc.clt_c = 85.5;
        oc.dfco_active = true;

        let mut buf = [0u8; 64];
        let n = streams.encode_due_frame(&oc, 0, &mut buf).unwrap();
        // 7 header + 2 (rpm) + 2 (clt) + 1 (2 bits)
        assert_eq!(n, 12);
        assert_eq!(buf[0], 1); // stream id
        assert_eq!(u16::from_le_bytes([buf[7], buf[8]]), 3000);
        assert_eq!(i16::from_le_bytes([buf[9], buf[10]]), 855);
        assert_eq!(buf[11], 0b10); // spark_cut=0 (bit0), dfco=1 (bit1)
    }

    #[test]
    fn rate_limits_frames() {
        let mut streams = TelemetryStreams::new();
        let (_, rate) = streams.subscribe(&[1], 10, 0).unwrap();
        assert_eq!(rate, 10);
        let oc = OutputChannels::zeroed();
        let mut buf = [0u8; 32];

        assert!(streams.encode_due_frame(&oc, 0, &mut buf).is_some());
        // 100 ms interval: not due at t=50
        assert!(streams.encode_due_frame(&oc, 50, &mut buf).is_none());
        assert!(streams.encode_due_frame(&oc, 100, &mut buf).is_some());
    }

    #[test]
    fn seq_increments() {
        let mut streams = TelemetryStreams::new();
        let _ = streams.subscribe(&[1], 100, 0).unwrap();
        let oc = OutputChannels::zeroed();
        let mut buf = [0u8; 32];
        let _ = streams.encode_due_frame(&oc, 0, &mut buf).unwrap();
        let s0 = u16::from_le_bytes([buf[1], buf[2]]);
        let _ = streams.encode_due_frame(&oc, 10, &mut buf).unwrap();
        let s1 = u16::from_le_bytes([buf[1], buf[2]]);
        assert_eq!(s1, s0.wrapping_add(1));
    }

    #[test]
    fn bad_channel_rejected() {
        let mut streams = TelemetryStreams::new();
        assert_eq!(
            streams.subscribe(&[1, 999], 10, 0),
            Err(SubscribeError::BadChannel(999))
        );
    }

    #[test]
    fn unsubscribe_frees_slot() {
        let mut streams = TelemetryStreams::new();
        let (id, _) = streams.subscribe(&[1], 10, 0).unwrap();
        assert_eq!(streams.active(), 1);
        assert!(streams.unsubscribe(id));
        assert_eq!(streams.active(), 0);
        assert!(!streams.unsubscribe(id));
    }

    #[test]
    fn stream_slots_exhaust() {
        let mut streams = TelemetryStreams::new();
        for _ in 0..MAX_STREAMS {
            streams.subscribe(&[1], 10, 0).unwrap();
        }
        assert_eq!(
            streams.subscribe(&[1], 10, 0),
            Err(SubscribeError::NoFreeStream)
        );
    }
}
