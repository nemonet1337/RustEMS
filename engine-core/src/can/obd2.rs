//! OBD-II standard protocol implementation.
//!
//! Implements standard OBD-II PIDs for engine monitoring:
//! - PID 0x01-0x0A: Live data (RPM, coolant temp, etc.)
//! - PID 0x20-0x2A: Additional live data
//! - PID 0x41-0x4A: Fuel system status
//!
//! Reference: SAE J1979 standard

use crate::hal::CanFrame;
use crate::sensors::SensorData;

/// OBD-II service IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ObdService {
    /// Show current data.
    ShowCurrentData = 0x01,
    /// Show stored diagnostic trouble codes.
    ShowDtc = 0x03,
    /// Clear diagnostic trouble codes.
    ClearDtc = 0x04,
    /// Test results, oxygen sensor.
    OxygenSensor = 0x05,
    /// On-board monitoring.
    OnBoardMonitoring = 0x06,
    /// Show pending DTCs.
    ShowPendingDtc = 0x07,
    /// Control vehicle systems.
    ControlSystems = 0x08,
    /// Show engine data.
    EngineData = 0x09,
}

/// Common OBD-II PIDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ObdPid {
    /// PIDs supported (bitmask).
    PidsSupported01_20 = 0x00,
    /// Monitor status.
    MonitorStatus = 0x01,
    /// Freeze DTC.
    FreezeDtc = 0x02,
    /// Fuel system status.
    FuelSystemStatus = 0x03,
    /// Calculated engine load.
    CalculatedLoad = 0x04,
    /// Engine coolant temperature.
    CoolantTemp = 0x05,
    /// Short term fuel trim (bank 1).
    ShortTermFuelTrimB1 = 0x06,
    /// Long term fuel trim (bank 1).
    LongTermFuelTrimB1 = 0x07,
    /// Short term fuel trim (bank 2).
    ShortTermFuelTrimB2 = 0x08,
    /// Long term fuel trim (bank 2).
    LongTermFuelTrimB2 = 0x09,
    /// Fuel pressure (gauge).
    FuelPressure = 0x0A,
    /// Intake manifold absolute pressure.
    Map = 0x0B,
    /// Engine RPM.
    Rpm = 0x0C,
    /// Vehicle speed.
    Speed = 0x0D,
    /// Timing advance.
    TimingAdvance = 0x0E,
    /// Intake air temperature.
    Iat = 0x0F,
    /// MAF air flow rate.
    Maf = 0x10,
    /// Throttle position.
    Tps = 0x11,
    /// Commanded secondary air status.
    SecondaryAirStatus = 0x12,
    /// Oxygen sensors present.
    O2SensorsPresent = 0x13,
    /// Oxygen sensor voltage (bank 1, sensor 1).
    O2SensorB1S1 = 0x14,
}

/// Extended engine data passed to the OBD-II handler for PIDs not in SensorData.
#[derive(Clone, Copy, Debug, Default)]
pub struct ObdEngineState {
    /// Current ignition timing advance (degrees BTDC). Used for PID 0x0E.
    pub timing_advance_deg: f32,
    /// Short term fuel trim bank 1 (percent, -100 to +100). Used for PID 0x06.
    pub stft_b1_pct: f32,
    /// Long term fuel trim bank 1 (percent, -100 to +100). Used for PID 0x07.
    pub ltft_b1_pct: f32,
    /// Vehicle speed (km/h). Used for PID 0x0D.
    pub vehicle_speed_kph: f32,
    /// MAF air flow (g/s). Used for PID 0x10.
    pub maf_g_per_s: f32,
}

/// OBD-II response builder.
pub struct ObdResponseBuilder {
    can_id: u32,
    data: heapless::Vec<u8, 8>,
}

impl ObdResponseBuilder {
    /// Create a new response builder for the given CAN ID.
    pub fn new(can_id: u32) -> Self {
        Self {
            can_id,
            data: heapless::Vec::new(),
        }
    }

    /// Set the service ID (response format: 0x40 + service).
    pub fn service(mut self, service: ObdService) -> Self {
        let _ = self.data.push(service as u8 + 0x40);
        self
    }

    /// Add a PID byte.
    pub fn pid(mut self, pid: ObdPid) -> Self {
        let _ = self.data.push(pid as u8);
        self
    }

    /// Add a data byte.
    pub fn byte(mut self, value: u8) -> Self {
        let _ = self.data.push(value);
        self
    }

    /// Build the CAN frame.
    pub fn build(self) -> Option<CanFrame> {
        if self.data.is_empty() {
            return None;
        }
        Some(CanFrame::standard(self.can_id as u16, &self.data))
    }
}

/// OBD-II query handler.
pub struct ObdHandler {
    response_can_id: u32,
}

impl ObdHandler {
    /// Create a new OBD-II handler.
    ///
    /// `response_can_id` is the CAN ID used for responses.
    pub fn new(response_can_id: u32) -> Self {
        Self { response_can_id }
    }

    /// Process an OBD-II query and return a response frame if applicable.
    pub fn handle_query(&self, query: &CanFrame, sensors: &SensorData) -> Option<CanFrame> {
        self.handle_query_with_state(query, sensors, &ObdEngineState::default())
    }

    /// Process an OBD-II query with extended engine state (timing, fuel trims, etc.).
    pub fn handle_query_with_state(
        &self,
        query: &CanFrame,
        sensors: &SensorData,
        engine_state: &ObdEngineState,
    ) -> Option<CanFrame> {
        if query.data.len() < 2 {
            return None;
        }

        let service = query.data[0];
        let pid = query.data[1];

        match service {
            0x01 => self.handle_show_current_data(pid, sensors, engine_state),
            _ => None,
        }
    }

    /// Handle service 0x01 (show current data) queries.
    fn handle_show_current_data(
        &self,
        pid: u8,
        sensors: &SensorData,
        engine_state: &ObdEngineState,
    ) -> Option<CanFrame> {
        match pid {
            0x00 => self.pids_supported_response(),
            0x03 => self.fuel_system_status_response(sensors),
            0x04 => self.calculated_load_response(sensors),
            0x05 => self.coolant_temp_response(sensors),
            0x06 => self.stft_response(engine_state),
            0x07 => self.ltft_response(engine_state),
            0x0B => self.map_response(sensors),
            0x0C => self.rpm_response(sensors),
            0x0D => self.speed_response(engine_state),
            0x0E => self.timing_advance_response(engine_state),
            0x0F => self.iat_response(sensors),
            0x10 => self.maf_response(engine_state),
            0x11 => self.tps_response(sensors),
            0x13 => self.o2_sensors_present_response(),
            0x14 => self.o2_sensor_b1s1_response(sensors),
            _ => None,
        }
    }

    /// PIDs supported response (PID 0x00).
    fn pids_supported_response(&self) -> Option<CanFrame> {
        // Bitmask for supported PIDs 0x01-0x20 (bit 31 = PID 0x01, bit 0 = PID 0x20)
        // Supported: 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x13, 0x14
        // Bit positions (31 - (pid - 1)):
        //   0x03 → bit 28, 0x04 → bit 27, 0x05 → bit 26, 0x06 → bit 25, 0x07 → bit 24
        //   0x0B → bit 20, 0x0C → bit 19, 0x0D → bit 18, 0x0E → bit 17, 0x0F → bit 16
        //   0x10 → bit 15, 0x11 → bit 14, 0x13 → bit 12, 0x14 → bit 11
        let bitmask: u32 = (1 << 28) | (1 << 27) | (1 << 26) | (1 << 25) | (1 << 24) // 0x03-0x07
            | (1 << 20) | (1 << 19) | (1 << 18) | (1 << 17) | (1 << 16) // 0x0B-0x0F
            | (1 << 15) | (1 << 14) | (1 << 12) | (1 << 11); // 0x10, 0x11, 0x13, 0x14
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::PidsSupported01_20)
            .byte((bitmask >> 24) as u8)
            .byte((bitmask >> 16) as u8)
            .byte((bitmask >> 8) as u8)
            .byte(bitmask as u8)
            .build()
    }

    /// Calculated engine load response (PID 0x04).
    fn calculated_load_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let load_pct = sensors.load_pct.unwrap_or(0.0).clamp(0.0, 100.0);
        let value = (load_pct / 100.0 * 255.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::CalculatedLoad)
            .byte(value)
            .build()
    }

    /// Coolant temperature response (PID 0x05).
    fn coolant_temp_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let temp_c = sensors.clt_celsius.unwrap_or(0.0);
        let value = (temp_c + 40.0).clamp(0.0, 255.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::CoolantTemp)
            .byte(value)
            .build()
    }

    /// MAP response (PID 0x0B).
    fn map_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let map_kpa = sensors.map_kpa.unwrap_or(0.0);
        let value = map_kpa as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Map)
            .byte(value)
            .build()
    }

    /// RPM response (PID 0x0C).
    fn rpm_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let rpm = sensors.rpm.unwrap_or(0.0);
        let value = (rpm * 4.0) as u16;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Rpm)
            .byte((value >> 8) as u8)
            .byte(value as u8)
            .build()
    }

    /// IAT response (PID 0x0F).
    fn iat_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let temp_c = sensors.iat_celsius.unwrap_or(0.0);
        let value = (temp_c + 40.0).clamp(0.0, 255.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Iat)
            .byte(value)
            .build()
    }

    /// TPS response (PID 0x11).
    fn tps_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        let tps_pct = sensors.tps_pct.unwrap_or(0.0).clamp(0.0, 100.0);
        let value = (tps_pct / 100.0 * 255.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Tps)
            .byte(value)
            .build()
    }

    /// Fuel system status response (PID 0x03).
    fn fuel_system_status_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        // 0x01 = open loop, 0x02 = closed loop, 0x04 = open loop (fault)
        let status = if sensors.rpm.unwrap_or(0.0) > 0.0 {
            0x02u8
        } else {
            0x01u8
        };
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::FuelSystemStatus)
            .byte(status)
            .byte(0x00) // Bank 2 status (not used)
            .build()
    }

    /// Short term fuel trim bank 1 response (PID 0x06).
    fn stft_response(&self, engine_state: &ObdEngineState) -> Option<CanFrame> {
        // Encoding: value = (trim_pct / 100.0 + 1.0) * 128.0
        // Range: -100% to +99.2%, 0% = 128
        let trim = engine_state.stft_b1_pct.clamp(-100.0, 99.2);
        let value = ((trim / 100.0 + 1.0) * 128.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::ShortTermFuelTrimB1)
            .byte(value)
            .build()
    }

    /// Long term fuel trim bank 1 response (PID 0x07).
    fn ltft_response(&self, engine_state: &ObdEngineState) -> Option<CanFrame> {
        let trim = engine_state.ltft_b1_pct.clamp(-100.0, 99.2);
        let value = ((trim / 100.0 + 1.0) * 128.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::LongTermFuelTrimB1)
            .byte(value)
            .build()
    }

    /// Vehicle speed response (PID 0x0D).
    fn speed_response(&self, engine_state: &ObdEngineState) -> Option<CanFrame> {
        let speed = engine_state.vehicle_speed_kph.clamp(0.0, 255.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Speed)
            .byte(speed)
            .build()
    }

    /// Timing advance response (PID 0x0E).
    fn timing_advance_response(&self, engine_state: &ObdEngineState) -> Option<CanFrame> {
        // Encoding: value = (advance + 64) * 2, range -64 to +63.5°
        let advance = engine_state.timing_advance_deg.clamp(-64.0, 63.5);
        let value = ((advance + 64.0) * 2.0) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::TimingAdvance)
            .byte(value)
            .build()
    }

    /// MAF air flow rate response (PID 0x10).
    fn maf_response(&self, engine_state: &ObdEngineState) -> Option<CanFrame> {
        // Encoding: value = maf_g_per_s * 100, as u16 big-endian
        let maf_raw = (engine_state.maf_g_per_s.clamp(0.0, 655.35) * 100.0) as u16;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Maf)
            .byte((maf_raw >> 8) as u8)
            .byte(maf_raw as u8)
            .build()
    }

    /// O2 sensors present response (PID 0x13).
    fn o2_sensors_present_response(&self) -> Option<CanFrame> {
        // Bitmask: bit 0 = B1S1, bit 1 = B1S2, etc.
        // Report B1S1 present (bit 0 set)
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::O2SensorsPresent)
            .byte(0x01)
            .build()
    }

    /// O2 sensor bank 1 sensor 1 response (PID 0x14).
    fn o2_sensor_b1s1_response(&self, sensors: &SensorData) -> Option<CanFrame> {
        // Byte A: voltage (0–1.275V, scale = 0.005V/bit)
        // Byte B: STFT (0x00 = not in closed loop, 0xFF = not available)
        let voltage = sensors.lambda1_voltage.unwrap_or(0.0).clamp(0.0, 1.275);
        let volt_raw = (voltage / 0.005) as u8;
        ObdResponseBuilder::new(self.response_can_id)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::O2SensorB1S1)
            .byte(volt_raw)
            .byte(0xFF) // STFT not used (wideband)
            .build()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn obd_response_builder() {
        let frame = ObdResponseBuilder::new(0x7E8)
            .service(ObdService::ShowCurrentData)
            .pid(ObdPid::Rpm)
            .byte(0x12)
            .byte(0x34)
            .build();
        assert!(frame.is_some());
        let frame = frame.unwrap();
        assert_eq!(frame.id, 0x7E8);
        assert_eq!(frame.dlc, 4);
    }

    #[test]
    fn rpm_response() {
        let handler = ObdHandler::new(0x7E8);
        let sensors = SensorData {
            rpm: Some(1000.0),
            ..Default::default()
        };
        let response = handler.rpm_response(&sensors);
        assert!(response.is_some());
        let frame = response.unwrap();
        // 1000 RPM * 4 = 4000 = 0x0FA0
        assert_eq!(frame.data[2], 0x0F);
        assert_eq!(frame.data[3], 0xA0);
    }

    #[test]
    fn coolant_temp_response() {
        let handler = ObdHandler::new(0x7E8);
        let sensors = SensorData {
            clt_celsius: Some(80.0),
            ..Default::default()
        };
        let response = handler.coolant_temp_response(&sensors);
        assert!(response.is_some());
        let frame = response.unwrap();
        // 80°C + 40 = 120
        assert_eq!(frame.data[2], 120);
    }
}
