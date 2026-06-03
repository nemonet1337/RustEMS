//! Device-side RDP (RustEMS Device Protocol) message handler.
//!
//! Dispatches parsed RDP opcodes to the parameter catalog and telemetry
//! layer.  Transport framing (COBS + CRC16) is handled separately by the
//! `rusefi-device-api` crate; this module works with decoded message payloads.
//!
//! # Message format
//!
//! Each payload starts with a 3-byte header:
//! ```text
//! [KIND: u8][OP: u16-LE][body...]
//! ```
//!
//! Responses use `Kind::Response` (0x01) with the same opcode mirrored and
//! either a 1-byte status code or inline data.

use crate::config::EngineConfig;
use crate::comms::output::OutputChannels;
use crate::params::{get_param, set_param, get_table_cell, set_table_cell, set_array_element, ParamId};

// ── Response status bytes ────────────────────────────────────────────────────

/// Request was processed successfully.
pub const RDP_OK: u8 = 0x00;
/// Request failed (bad parameter ID, out-of-range index, …).
pub const RDP_ERR_PARAM: u8 = 0x01;
/// Request rejected (bad auth, protected state, …).
pub const RDP_ERR_REJECTED: u8 = 0x02;
/// Opcode not recognised.
pub const RDP_ERR_UNKNOWN_OP: u8 = 0xFF;

// ── Opcodes (mirrors device-api op catalog) ──────────────────────────────────

const OP_HELLO: u16            = 0x0101;
const OP_PING: u16             = 0x0102;
const OP_GET_SCHEMA_INFO: u16  = 0x0201;
const OP_PARAM_GET: u16        = 0x0301;
const OP_PARAM_SET: u16        = 0x0302;
const OP_TABLE_GET: u16        = 0x0303;
const OP_TABLE_SET_CELL: u16   = 0x0304;
const OP_TABLE_SET_AXIS: u16   = 0x0305;
const OP_CONFIG_SAVE: u16      = 0x0306;
const OP_CONFIG_DISCARD: u16   = 0x0307;
const OP_TELEMETRY_GET: u16    = 0x0401;

/// Schema version — bump whenever the param catalog changes.
const SCHEMA_VERSION: u32 = 1;

/// Result from [`handle_rdp`].
pub struct RdpResponse {
    /// Number of bytes written into the response buffer.
    pub len: usize,
    /// `true` when the handler requests a config save (flash write).
    pub save_requested: bool,
}

/// Handle one decoded RDP request payload, writing the response into `out`.
///
/// # Arguments
/// * `req`     — decoded request payload (header + body, no framing)
/// * `cfg`     — live engine config (read-only for most commands; mutated by SET)
/// * `cfg_mut` — mutable reference used by PARAM_SET / TABLE_SET_* / CONFIG_SAVE
/// * `outputs` — live telemetry snapshot for TELEMETRY_GET
/// * `out`     — response buffer (should be ≥ 256 bytes)
///
/// # Returns
/// [`RdpResponse`] with the written byte count and save flag.
pub fn handle_rdp(
    req: &[u8],
    cfg: &EngineConfig,
    cfg_mut: &mut EngineConfig,
    outputs: &OutputChannels,
    out: &mut [u8],
) -> RdpResponse {
    let mut save_requested = false;

    // Parse header: KIND(1) OP(2LE)
    if req.len() < 3 || out.len() < 4 {
        return RdpResponse { len: 0, save_requested: false };
    }
    let _kind = req[0];
    let op = u16::from_le_bytes([req[1], req[2]]);
    let body = &req[3..];

    // Write response header: KIND=Response(1) OP(2LE) STATUS(1)
    out[0] = 0x01; // Kind::Response
    out[1] = req[1];
    out[2] = req[2];

    let payload_start = 4; // after header + status byte
    let len = match op {
        OP_HELLO => {
            let sig = b"RustEMS 0.1.0";
            let n = sig.len().min(out.len().saturating_sub(payload_start));
            out[3] = RDP_OK;
            out[payload_start..payload_start + n].copy_from_slice(&sig[..n]);
            payload_start + n
        }

        OP_PING => {
            out[3] = RDP_OK;
            payload_start
        }

        OP_GET_SCHEMA_INFO => {
            if out.len() < payload_start + 4 {
                out[3] = RDP_ERR_REJECTED;
                payload_start
            } else {
                out[3] = RDP_OK;
                out[payload_start..payload_start + 4]
                    .copy_from_slice(&SCHEMA_VERSION.to_le_bytes());
                payload_start + 4
            }
        }

        OP_PARAM_GET => {
            // Body: [param_id: u16-LE] (repeating)
            if body.len() < 2 {
                out[3] = RDP_ERR_PARAM;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            let id_raw = u16::from_le_bytes([body[0], body[1]]);
            match ParamId::from_u16(id_raw).and_then(|id| get_param(cfg, id)) {
                Some(v) => {
                    if out.len() < payload_start + 4 {
                        out[3] = RDP_ERR_REJECTED;
                        payload_start
                    } else {
                        out[3] = RDP_OK;
                        out[payload_start..payload_start + 4]
                            .copy_from_slice(&v.to_le_bytes());
                        payload_start + 4
                    }
                }
                None => { out[3] = RDP_ERR_PARAM; payload_start }
            }
        }

        OP_PARAM_SET => {
            // Body: [param_id: u16-LE][value: f32-LE]
            if body.len() < 6 {
                out[3] = RDP_ERR_PARAM;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            let id_raw = u16::from_le_bytes([body[0], body[1]]);
            let value  = f32::from_le_bytes([body[2], body[3], body[4], body[5]]);
            match ParamId::from_u16(id_raw) {
                Some(id) if set_param(cfg_mut, id, value) => {
                    out[3] = RDP_OK;
                }
                _ => { out[3] = RDP_ERR_PARAM; }
            }
            payload_start
        }

        OP_TABLE_GET => {
            // Body: [table_base: u16-LE][row: u8][col: u8]
            if body.len() < 4 {
                out[3] = RDP_ERR_PARAM;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            let base_raw = u16::from_le_bytes([body[0], body[1]]);
            let row = body[2] as usize;
            let col = body[3] as usize;
            match ParamId::from_u16(base_raw).and_then(|b| get_table_cell(cfg, b, row, col)) {
                Some(v) if out.len() >= payload_start + 4 => {
                    out[3] = RDP_OK;
                    out[payload_start..payload_start + 4].copy_from_slice(&v.to_le_bytes());
                    payload_start + 4
                }
                Some(_) => { out[3] = RDP_ERR_REJECTED; payload_start }
                None    => { out[3] = RDP_ERR_PARAM;    payload_start }
            }
        }

        OP_TABLE_SET_CELL => {
            // Body: [table_base: u16-LE][row: u8][col: u8][value: f32-LE]
            if body.len() < 8 {
                out[3] = RDP_ERR_PARAM;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            let base_raw = u16::from_le_bytes([body[0], body[1]]);
            let row   = body[2] as usize;
            let col   = body[3] as usize;
            let value = f32::from_le_bytes([body[4], body[5], body[6], body[7]]);
            match ParamId::from_u16(base_raw) {
                Some(b) if set_table_cell(cfg_mut, b, row, col, value) => {
                    out[3] = RDP_OK;
                }
                _ => { out[3] = RDP_ERR_PARAM; }
            }
            payload_start
        }

        OP_TABLE_SET_AXIS => {
            // Body: [array_base: u16-LE][index: u8][value: f32-LE]
            if body.len() < 7 {
                out[3] = RDP_ERR_PARAM;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            let base_raw = u16::from_le_bytes([body[0], body[1]]);
            let idx   = body[2] as usize;
            let value = f32::from_le_bytes([body[3], body[4], body[5], body[6]]);
            match ParamId::from_u16(base_raw) {
                Some(b) if set_array_element(cfg_mut, b, idx, value) => {
                    out[3] = RDP_OK;
                }
                _ => { out[3] = RDP_ERR_PARAM; }
            }
            payload_start
        }

        OP_CONFIG_SAVE => {
            save_requested = true;
            out[3] = RDP_OK;
            payload_start
        }

        OP_CONFIG_DISCARD => {
            // Caller should restore cfg_mut from flash snapshot; here we just ACK.
            out[3] = RDP_OK;
            payload_start
        }

        OP_TELEMETRY_GET => {
            // Returns a compact telemetry snapshot: 28 bytes of f32 LE values.
            // rpm, clt, iat, map, tps, vbatt, lambda, inj_ms, adv, oil_kpa, fuel_pct,
            // knock_retard, cl_corr, ltft_corr, (flags u32)
            let needed = payload_start + 14 * 4 + 4;
            if out.len() < needed {
                out[3] = RDP_ERR_REJECTED;
                return RdpResponse { len: payload_start, save_requested: false };
            }
            out[3] = RDP_OK;
            let mut pos = payload_start;
            let put_f32 = |buf: &mut [u8], p: &mut usize, v: f32| {
                buf[*p..*p + 4].copy_from_slice(&v.to_le_bytes());
                *p += 4;
            };
            put_f32(out, &mut pos, outputs.rpm);
            put_f32(out, &mut pos, outputs.clt_c);
            put_f32(out, &mut pos, outputs.iat_c);
            put_f32(out, &mut pos, outputs.map_kpa);
            put_f32(out, &mut pos, outputs.tps_pct);
            put_f32(out, &mut pos, outputs.battery_v);
            put_f32(out, &mut pos, outputs.lambda);
            put_f32(out, &mut pos, outputs.inj_pulse_ms);
            put_f32(out, &mut pos, outputs.advance_deg);
            put_f32(out, &mut pos, outputs.oil_pressure_kpa);
            put_f32(out, &mut pos, outputs.fuel_level_pct);
            put_f32(out, &mut pos, outputs.knock_retard_deg);
            put_f32(out, &mut pos, outputs.cl_correction);
            put_f32(out, &mut pos, outputs.ltft_correction);
            // Flags word
            let mut flags: u32 = 0;
            if outputs.spark_cut      { flags |= 1 << 0; }
            if outputs.sequential     { flags |= 1 << 1; }
            if outputs.dfco_active    { flags |= 1 << 2; }
            if outputs.fuel_pump_on   { flags |= 1 << 3; }
            if outputs.fan_on         { flags |= 1 << 4; }
            if outputs.limp_active    { flags |= 1 << 5; }
            out[pos..pos + 4].copy_from_slice(&flags.to_le_bytes());
            pos + 4
        }

        _ => {
            out[3] = RDP_ERR_UNKNOWN_OP;
            payload_start
        }
    };

    RdpResponse { len, save_requested }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EngineConfig;
    use crate::comms::output::OutputChannels;

    fn make_req(kind: u8, op: u16, body: &[u8]) -> heapless::Vec<u8, 64> {
        let mut v = heapless::Vec::new();
        let _ = v.push(kind);
        let _ = v.extend_from_slice(&op.to_le_bytes());
        let _ = v.extend_from_slice(body);
        v
    }

    #[test]
    fn ping_returns_ok() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let req = make_req(0, OP_PING, &[]);
        let mut out = [0u8; 64];
        let r = handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_OK);
        assert_eq!(r.len, 4);
    }

    #[test]
    fn param_get_cranking_rpm() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let id = ParamId::CrankingRpm.as_u16();
        let req = make_req(0, OP_PARAM_GET, &id.to_le_bytes());
        let mut out = [0u8; 64];
        let r = handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_OK);
        let v = f32::from_le_bytes(out[4..8].try_into().unwrap());
        assert_eq!(v, cfg.cranking_rpm);
        assert_eq!(r.len, 8);
    }

    #[test]
    fn param_set_cranking_rpm() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let id = ParamId::CrankingRpm.as_u16();
        let mut body = [0u8; 6];
        body[0..2].copy_from_slice(&id.to_le_bytes());
        body[2..6].copy_from_slice(&350.0f32.to_le_bytes());
        let req = make_req(0, OP_PARAM_SET, &body);
        let mut out = [0u8; 64];
        handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_OK);
        assert_eq!(cfg_mut.cranking_rpm, 350.0);
    }

    #[test]
    fn table_set_and_get() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let base = ParamId::IgnitionTableBase.as_u16();

        // Set cell [1][2] = 28.0
        let mut body = [0u8; 8];
        body[0..2].copy_from_slice(&base.to_le_bytes());
        body[2] = 1; // row
        body[3] = 2; // col
        body[4..8].copy_from_slice(&28.0f32.to_le_bytes());
        let req = make_req(0, OP_TABLE_SET_CELL, &body);
        let mut out = [0u8; 64];
        handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_OK);

        // Get same cell
        let mut body2 = [0u8; 4];
        body2[0..2].copy_from_slice(&base.to_le_bytes());
        body2[2] = 1;
        body2[3] = 2;
        let req2 = make_req(0, OP_TABLE_GET, &body2);
        let mut out2 = [0u8; 64];
        handle_rdp(&req2, &cfg_mut, &mut cfg_mut.clone(), &outputs, &mut out2);
        assert_eq!(out2[3], RDP_OK);
        let v = f32::from_le_bytes(out2[4..8].try_into().unwrap());
        assert!((v - 28.0).abs() < 0.001);
    }

    #[test]
    fn config_save_sets_flag() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let req = make_req(0, OP_CONFIG_SAVE, &[]);
        let mut out = [0u8; 64];
        let r = handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert!(r.save_requested);
        assert_eq!(out[3], RDP_OK);
    }

    #[test]
    fn telemetry_get_encodes_rpm() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let mut outputs = OutputChannels::zeroed();
        outputs.rpm = 3000.0;
        let req = make_req(0, OP_TELEMETRY_GET, &[]);
        let mut out = [0u8; 128];
        let r = handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_OK);
        let rpm = f32::from_le_bytes(out[4..8].try_into().unwrap());
        assert!((rpm - 3000.0).abs() < 0.1);
        assert_eq!(r.len, 4 + 14 * 4 + 4);
    }

    #[test]
    fn unknown_op_returns_error() {
        let cfg = EngineConfig::default_4cyl();
        let mut cfg_mut = cfg.clone();
        let outputs = OutputChannels::zeroed();
        let req = make_req(0, 0xDEAD, &[]);
        let mut out = [0u8; 64];
        handle_rdp(&req, &cfg, &mut cfg_mut, &outputs, &mut out);
        assert_eq!(out[3], RDP_ERR_UNKNOWN_OP);
    }
}
