//! Live output channels (telemetry) served to TunerStudio gauges.
//!
//! Values are packed into a fixed big-endian byte block. Scaling is chosen so
//! every field fits in 16 bits; the matching offsets/scales must be declared in
//! the generated TunerStudio INI `[OutputChannels]` section.

/// Number of bytes in the serialized output-channel block.
pub const OUTPUT_CHANNELS_LEN: usize = 20;

/// Snapshot of live engine telemetry.
#[derive(Clone, Copy, Debug, Default)]
pub struct OutputChannels {
    /// Engine speed (RPM).
    pub rpm: f32,
    /// Coolant temperature (°C).
    pub clt_c: f32,
    /// Intake air temperature (°C).
    pub iat_c: f32,
    /// Manifold absolute pressure (kPa).
    pub map_kpa: f32,
    /// Throttle position (%).
    pub tps_pct: f32,
    /// Battery voltage (V).
    pub battery_v: f32,
    /// Measured lambda (1.0 = stoichiometric).
    pub lambda: f32,
    /// Injector pulse width (ms).
    pub inj_pulse_ms: f32,
    /// Ignition advance (° BTDC).
    pub advance_deg: f32,
    /// True when spark is cut by the RPM limiter.
    pub spark_cut: bool,
    /// True when sequential injection is active.
    pub sequential: bool,
}

#[inline]
fn put_u16(buf: &mut [u8], at: usize, v: u16) {
    buf[at..at + 2].copy_from_slice(&v.to_be_bytes());
}

#[inline]
fn put_i16(buf: &mut [u8], at: usize, v: i16) {
    buf[at..at + 2].copy_from_slice(&v.to_be_bytes());
}

impl OutputChannels {
    /// A const all-zero instance (usable to initialise a `static`).
    pub const fn zeroed() -> Self {
        Self {
            rpm: 0.0,
            clt_c: 0.0,
            iat_c: 0.0,
            map_kpa: 0.0,
            tps_pct: 0.0,
            battery_v: 0.0,
            lambda: 0.0,
            inj_pulse_ms: 0.0,
            advance_deg: 0.0,
            spark_cut: false,
            sequential: false,
        }
    }

    /// Serialize into the fixed big-endian layout. Returns the number of bytes
    /// written, or `None` if `buf` is shorter than [`OUTPUT_CHANNELS_LEN`].
    pub fn write_to(&self, buf: &mut [u8]) -> Option<usize> {
        if buf.len() < OUTPUT_CHANNELS_LEN {
            return None;
        }
        put_u16(buf, 0, self.rpm.clamp(0.0, 65_535.0) as u16);
        put_i16(buf, 2, (self.clt_c * 10.0).clamp(-32_768.0, 32_767.0) as i16);
        put_i16(buf, 4, (self.iat_c * 10.0).clamp(-32_768.0, 32_767.0) as i16);
        put_u16(buf, 6, (self.map_kpa * 10.0).clamp(0.0, 65_535.0) as u16);
        put_u16(buf, 8, (self.tps_pct * 10.0).clamp(0.0, 65_535.0) as u16);
        put_u16(buf, 10, (self.battery_v * 100.0).clamp(0.0, 65_535.0) as u16);
        put_u16(buf, 12, (self.lambda * 1000.0).clamp(0.0, 65_535.0) as u16);
        put_u16(buf, 14, (self.inj_pulse_ms * 100.0).clamp(0.0, 65_535.0) as u16);
        put_i16(buf, 16, (self.advance_deg * 10.0).clamp(-32_768.0, 32_767.0) as i16);
        let mut flags = 0u8;
        if self.spark_cut {
            flags |= 0x01;
        }
        if self.sequential {
            flags |= 0x02;
        }
        buf[18] = flags;
        buf[19] = 0; // reserved/padding
        Some(OUTPUT_CHANNELS_LEN)
    }

    /// Serialize into a fresh fixed-size array.
    pub fn to_bytes(&self) -> [u8; OUTPUT_CHANNELS_LEN] {
        let mut buf = [0u8; OUTPUT_CHANNELS_LEN];
        let _ = self.write_to(&mut buf);
        buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_scales_and_packs() {
        let oc = OutputChannels {
            rpm: 3500.0,
            clt_c: 87.5,
            iat_c: -10.0,
            map_kpa: 95.0,
            tps_pct: 42.0,
            battery_v: 13.8,
            lambda: 0.95,
            inj_pulse_ms: 3.2,
            advance_deg: 22.5,
            spark_cut: false,
            sequential: true,
        };
        let b = oc.to_bytes();
        assert_eq!(u16::from_be_bytes([b[0], b[1]]), 3500);
        assert_eq!(i16::from_be_bytes([b[2], b[3]]), 875);
        assert_eq!(i16::from_be_bytes([b[4], b[5]]), -100);
        assert_eq!(u16::from_be_bytes([b[6], b[7]]), 950);
        assert_eq!(u16::from_be_bytes([b[8], b[9]]), 420);
        assert_eq!(u16::from_be_bytes([b[10], b[11]]), 1380);
        assert_eq!(u16::from_be_bytes([b[12], b[13]]), 950);
        assert_eq!(u16::from_be_bytes([b[14], b[15]]), 320);
        assert_eq!(i16::from_be_bytes([b[16], b[17]]), 225);
        assert_eq!(b[18], 0x02); // sequential flag set, spark_cut clear
    }

    #[test]
    fn spark_cut_flag_sets_bit0() {
        let oc = OutputChannels {
            spark_cut: true,
            ..Default::default()
        };
        assert_eq!(oc.to_bytes()[18], 0x01);
    }

    #[test]
    fn short_buffer_is_rejected() {
        let oc = OutputChannels::default();
        let mut small = [0u8; 4];
        assert_eq!(oc.write_to(&mut small), None);
    }
}
