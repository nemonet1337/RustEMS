//! CAN dashboard/display protocols.
//!
//! Encodes engine sensor data into CAN frames for aftermarket dash displays.
//! Supported protocols:
//! - Haltech IC-7 / Nexus / Pro dash (0x360–0x36F)
//! - BMW instrument cluster (E-series, 0x0AA / 0x0D5)
//! - Honda K-series display (0x204 / 0x309)
//!
//! Reference: various reverse-engineered CAN logs and rusEFI CAN database.

use crate::hal::{CanBus, CanFrame};
use crate::sensors::SensorData;

// ─── Haltech protocol ─────────────────────────────────────────────────────────

/// Haltech CAN dash protocol base ID (IC-7 / Nexus / Pro).
const HALTECH_BASE_ID: u16 = 0x360;

/// Haltech frame 0x360: RPM, TPS, IAT, MAP.
fn haltech_frame_360(sensors: &SensorData) -> CanFrame {
    let rpm_raw = sensors.rpm.unwrap_or(0.0).clamp(0.0, 16383.0);
    let rpm_u16 = (rpm_raw * 4.0) as u16; // scale: 0.25 RPM/bit

    let tps_raw = sensors.tps_pct.unwrap_or(0.0).clamp(0.0, 100.0);
    let tps_u8 = (tps_raw * 2.0) as u8; // scale: 0.5%/bit

    let iat_raw = sensors.iat_celsius.unwrap_or(0.0).clamp(-40.0, 215.0);
    let iat_u8 = (iat_raw + 40.0) as u8; // offset: -40°C

    let map_raw = sensors.map_kpa.unwrap_or(0.0).clamp(0.0, 400.0);
    let map_u16 = (map_raw * 10.0) as u16; // scale: 0.1 kPa/bit

    let data = [
        (rpm_u16 >> 8) as u8,
        rpm_u16 as u8,
        tps_u8,
        iat_u8,
        (map_u16 >> 8) as u8,
        map_u16 as u8,
        0,
        0,
    ];
    CanFrame::standard(HALTECH_BASE_ID, &data)
}

/// Haltech frame 0x361: Lambda1, Lambda2, battery voltage.
fn haltech_frame_361(sensors: &SensorData) -> CanFrame {
    let lambda1_raw = sensors.lambda1_voltage.unwrap_or(0.0).clamp(0.0, 2.0);
    let lambda1_u16 = (lambda1_raw * 1000.0) as u16; // scale: 0.001 λ/bit

    let lambda2_raw = sensors.lambda2_voltage.unwrap_or(0.0).clamp(0.0, 2.0);
    let lambda2_u16 = (lambda2_raw * 1000.0) as u16;

    let vbatt_raw = sensors.battery_volts.unwrap_or(0.0).clamp(0.0, 20.0);
    let vbatt_u8 = (vbatt_raw * 10.0) as u8; // scale: 0.1V/bit

    let data = [
        (lambda1_u16 >> 8) as u8,
        lambda1_u16 as u8,
        (lambda2_u16 >> 8) as u8,
        lambda2_u16 as u8,
        vbatt_u8,
        0,
        0,
        0,
    ];
    CanFrame::standard(HALTECH_BASE_ID + 1, &data)
}

/// Haltech frame 0x362: CLT, oil pressure, fuel pressure.
fn haltech_frame_362(sensors: &SensorData) -> CanFrame {
    let clt_raw = sensors.clt_celsius.unwrap_or(0.0).clamp(-40.0, 215.0);
    let clt_u8 = (clt_raw + 40.0) as u8;

    let oil_raw = sensors.oil_pressure_kpa.unwrap_or(0.0).clamp(0.0, 1000.0);
    let oil_u16 = (oil_raw * 10.0) as u16; // scale: 0.1 kPa/bit

    let data = [
        clt_u8,
        0,
        (oil_u16 >> 8) as u8,
        oil_u16 as u8,
        0,
        0,
        0,
        0,
    ];
    CanFrame::standard(HALTECH_BASE_ID + 2, &data)
}

// ─── BMW E-series protocol ────────────────────────────────────────────────────

/// BMW E-series CAN instrument cluster ID for RPM / ignition status.
const BMW_CLUSTER_ID_0AA: u16 = 0x0AA;
/// BMW E-series CAN ID for coolant temperature.
const BMW_CLUSTER_ID_0D5: u16 = 0x0D5;

/// BMW E-series frame 0x0AA: RPM and running status.
fn bmw_frame_0aa(sensors: &SensorData) -> CanFrame {
    let rpm = sensors.rpm.unwrap_or(0.0).clamp(0.0, 8191.0);
    // BMW RPM encoding: raw_value = rpm * 6.4, split into bytes 4-5
    let rpm_raw = (rpm * 6.4) as u16;

    let running = if sensors.rpm.unwrap_or(0.0) > 400.0 { 0x01u8 } else { 0x00u8 };

    let data = [
        running,
        0x00,
        0x00,
        0x00,
        (rpm_raw >> 8) as u8,
        rpm_raw as u8,
        0x00,
        0x00,
    ];
    CanFrame::standard(BMW_CLUSTER_ID_0AA, &data)
}

/// BMW E-series frame 0x0D5: coolant temperature.
fn bmw_frame_0d5(sensors: &SensorData) -> CanFrame {
    // BMW CLT encoding: raw = (temp + 48.373) / 0.75 (cluster uses Fahrenheit-based scale)
    let clt_c = sensors.clt_celsius.unwrap_or(0.0);
    let clt_raw = ((clt_c + 48.373) / 0.75).clamp(0.0, 255.0) as u8;

    let data = [clt_raw, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    CanFrame::standard(BMW_CLUSTER_ID_0D5, &data)
}

// ─── Honda K-series protocol ──────────────────────────────────────────────────

/// Honda K-series CAN ID for RPM / speed.
const HONDA_ID_204: u16 = 0x204;
/// Honda K-series CAN ID for engine status.
const HONDA_ID_309: u16 = 0x309;

/// Honda K-series frame 0x204: RPM and vehicle speed.
fn honda_frame_204(sensors: &SensorData) -> CanFrame {
    // Honda RPM: raw = rpm * 4
    let rpm = sensors.rpm.unwrap_or(0.0).clamp(0.0, 16000.0);
    let rpm_raw = (rpm * 4.0) as u16;

    let data = [
        (rpm_raw >> 8) as u8,
        rpm_raw as u8,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    ];
    CanFrame::standard(HONDA_ID_204, &data)
}

/// Honda K-series frame 0x309: CLT and TPS.
fn honda_frame_309(sensors: &SensorData) -> CanFrame {
    let clt_c = sensors.clt_celsius.unwrap_or(0.0);
    // Honda CLT: raw = temp + 40 (offset encoding)
    let clt_raw = (clt_c + 40.0).clamp(0.0, 255.0) as u8;

    let tps_pct = sensors.tps_pct.unwrap_or(0.0).clamp(0.0, 100.0);
    let tps_raw = (tps_pct * 2.55) as u8; // 0–255

    let data = [clt_raw, tps_raw, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    CanFrame::standard(HONDA_ID_309, &data)
}

// ─── Protocol enum and encoder ────────────────────────────────────────────────

/// Supported CAN dash protocols.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DashProtocol {
    /// Haltech IC-7 / Nexus / Pro Dash (0x360–0x362).
    Haltech,
    /// BMW E-series instrument cluster (0x0AA, 0x0D5).
    BmwEseries,
    /// Honda K-series display (0x204, 0x309).
    HondaKseries,
}

/// CAN dash encoder — transmits sensor data using the configured protocol.
pub struct DashCanEncoder {
    protocol: DashProtocol,
}

impl DashCanEncoder {
    /// Create a new dash encoder for the given protocol.
    pub fn new(protocol: DashProtocol) -> Self {
        Self { protocol }
    }

    /// Transmit all frames for the current protocol.
    ///
    /// Call this periodically (e.g., every 10 ms) from the control loop.
    ///
    /// # Returns
    /// Number of frames successfully queued for transmission.
    pub fn transmit_all<CB: CanBus>(&self, can: &mut CB, sensors: &SensorData) -> u8 {
        match self.protocol {
            DashProtocol::Haltech => self.transmit_haltech(can, sensors),
            DashProtocol::BmwEseries => self.transmit_bmw(can, sensors),
            DashProtocol::HondaKseries => self.transmit_honda(can, sensors),
        }
    }

    fn transmit_haltech<CB: CanBus>(&self, can: &mut CB, sensors: &SensorData) -> u8 {
        let mut sent = 0u8;
        if can.transmit(&haltech_frame_360(sensors)) { sent += 1; }
        if can.transmit(&haltech_frame_361(sensors)) { sent += 1; }
        if can.transmit(&haltech_frame_362(sensors)) { sent += 1; }
        sent
    }

    fn transmit_bmw<CB: CanBus>(&self, can: &mut CB, sensors: &SensorData) -> u8 {
        let mut sent = 0u8;
        if can.transmit(&bmw_frame_0aa(sensors)) { sent += 1; }
        if can.transmit(&bmw_frame_0d5(sensors)) { sent += 1; }
        sent
    }

    fn transmit_honda<CB: CanBus>(&self, can: &mut CB, sensors: &SensorData) -> u8 {
        let mut sent = 0u8;
        if can.transmit(&honda_frame_204(sensors)) { sent += 1; }
        if can.transmit(&honda_frame_309(sensors)) { sent += 1; }
        sent
    }

    /// Get the configured protocol.
    pub fn protocol(&self) -> DashProtocol {
        self.protocol
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::SensorData;

    struct MockCan {
        frames: heapless::Vec<CanFrame, 8>,
    }

    impl MockCan {
        fn new() -> Self {
            Self { frames: heapless::Vec::new() }
        }
    }

    impl CanBus for MockCan {
        fn transmit(&mut self, frame: &CanFrame) -> bool {
            self.frames.push(*frame).is_ok()
        }
        fn receive(&mut self) -> Option<CanFrame> {
            None
        }
    }

    fn sensor_snapshot() -> SensorData {
        SensorData {
            rpm: Some(3000.0),
            load_pct: Some(60.0),
            clt_celsius: Some(85.0),
            iat_celsius: Some(25.0),
            tps_pct: Some(50.0),
            map_kpa: Some(80.0),
            battery_volts: Some(14.2),
            ..Default::default()
        }
    }

    #[test]
    fn haltech_transmits_3_frames() {
        let mut can = MockCan::new();
        let encoder = DashCanEncoder::new(DashProtocol::Haltech);
        let sent = encoder.transmit_all(&mut can, &sensor_snapshot());
        assert_eq!(sent, 3);
        assert_eq!(can.frames[0].id, 0x360);
        assert_eq!(can.frames[1].id, 0x361);
        assert_eq!(can.frames[2].id, 0x362);
    }

    #[test]
    fn haltech_rpm_encoding() {
        let sensors = SensorData { rpm: Some(3000.0), ..Default::default() };
        let frame = haltech_frame_360(&sensors);
        // RPM = 3000 * 4 = 12000 = 0x2EE0
        let rpm_decoded = ((frame.data[0] as u16) << 8 | frame.data[1] as u16) as f32 / 4.0;
        assert!((rpm_decoded - 3000.0).abs() < 1.0);
    }

    #[test]
    fn bmw_transmits_2_frames() {
        let mut can = MockCan::new();
        let encoder = DashCanEncoder::new(DashProtocol::BmwEseries);
        let sent = encoder.transmit_all(&mut can, &sensor_snapshot());
        assert_eq!(sent, 2);
        assert_eq!(can.frames[0].id, 0x0AA);
        assert_eq!(can.frames[1].id, 0x0D5);
    }

    #[test]
    fn honda_transmits_2_frames() {
        let mut can = MockCan::new();
        let encoder = DashCanEncoder::new(DashProtocol::HondaKseries);
        let sent = encoder.transmit_all(&mut can, &sensor_snapshot());
        assert_eq!(sent, 2);
        assert_eq!(can.frames[0].id, 0x204);
        assert_eq!(can.frames[1].id, 0x309);
    }

    #[test]
    fn haltech_clt_encoding() {
        let sensors = SensorData { clt_celsius: Some(85.0), ..Default::default() };
        let frame = haltech_frame_362(&sensors);
        // CLT = 85 + 40 = 125
        assert_eq!(frame.data[0], 125);
    }
}
