//! Device-side RDP (RustEMS Device Protocol) message handler.
//!
//! Implements every opcode of the message catalog in
//! `docs/api/03-message-protocol.md` on top of the shared
//! `rusefi-device-api` wire types:
//!
//! - **System** — Hello / Ping / Reboot / EnterBootloader
//! - **Descriptor** — schema info and paged parameter / table / telemetry
//!   catalogs (self-describing, no INI needed on the host)
//! - **Config** — ParamGet/Set, TableGet/SetCell/SetAxis and the RAM-staging →
//!   flash-commit transaction (Save / Discard / ResetDefaults / Status)
//! - **Telemetry** — subscription streams, one-shot reads, packed push frames
//! - **Control** — bench tests, fail-safe overrides, calibration routines
//! - **Diagnostics** — structured faults and asynchronous events
//!
//! Transport framing (COBS + CRC16 + fragmentation) is handled by the
//! `rusefi-device-api` crate; this module consumes decoded message payloads
//! (`KIND(u8) OP(u16 LE) BODY`) and produces response payloads.

use crate::comms::control::{
    BenchTarget, BenchTestSpec, CalibrateRoutine, Calibrations, OverrideTarget, Overrides,
};
use crate::comms::faults::{EventKind, EventQueue, FaultStore, Severity};
use crate::comms::output::OutputChannels;
use crate::comms::telemetry::{
    telemetry_hash, SubscribeError, TelemetryStreams, TELEMETRY_CATALOG,
};
use crate::config::EngineConfig;
use crate::params::{
    self, catalog_hash, config_crc, fnv1a, param_meta, set_param, ParamId, TableAxis, TableId,
    TableWriteError, CATEGORIES, FNV_OFFSET, PARAM_CATALOG, PFLAG_ENGINE_STOPPED_ONLY,
    PFLAG_READ_ONLY, TABLE_CATALOG,
};

use rusefi_device_api::cbor as bodies;
use rusefi_device_api::cbor::{decode_from_slice, encode_to_slice};
use rusefi_device_api::message::{
    op, read_message_header, write_message_header, ErrorCode, Kind, ValueType, MESSAGE_HEADER_LEN,
};
use rusefi_device_api::MAX_PAYLOAD_LEN;

// ─── Protocol constants ──────────────────────────────────────────────────────

/// Protocol major version (mismatch → disconnect).
pub const PROTO_MAJOR: u8 = 1;
/// Protocol minor version (backward-compatible additions).
pub const PROTO_MINOR: u8 = 0;

/// Confirmation magic for `EnterBootloader`.
pub const BOOTLOADER_CONFIRM: u32 = 0xB007;
/// Confirmation magic for `ConfigResetDefaults`.
pub const RESET_DEFAULTS_CONFIRM: u32 = 0xDEFA;

/// Descriptor catalog paging sizes.
const PARAMS_PER_PAGE: usize = 4;
const TABLES_PER_PAGE: usize = 4;
const CHANNELS_PER_PAGE: usize = 8;

/// Board identifiers for `HelloInfo.board`.
pub mod board {
    /// PC simulator.
    pub const SIM: u8 = 0;
    /// Nano (STM32F4, 1–2 cylinders).
    pub const NANO: u8 = 1;
    /// microRusEFI (STM32F407).
    pub const MICRO_RUSEFI: u8 = 2;
    /// uaEFI (STM32F4, 6 cylinders).
    pub const UAEFI: u8 = 3;
    /// Proteus (STM32F767).
    pub const PROTEUS: u8 = 4;
    /// Huge (STM32F4, 12 cylinders).
    pub const HUGE: u8 = 5;
}

/// Capability bit flags for `HelloInfo.capabilities`.
pub mod capability {
    /// Fuel injection control.
    pub const FUEL: u32 = 1 << 0;
    /// Ignition control.
    pub const IGNITION: u32 = 1 << 1;
    /// Boost control.
    pub const BOOST: u32 = 1 << 2;
    /// VVT control.
    pub const VVT: u32 = 1 << 3;
    /// Knock detection.
    pub const KNOCK: u32 = 1 << 4;
    /// CAN bus.
    pub const CAN: u32 = 1 << 5;
    /// Sequential injection.
    pub const SEQUENTIAL: u32 = 1 << 6;
}

/// Static device identity advertised by `Hello`.
#[derive(Clone, Copy, Debug)]
pub struct DeviceIdentity {
    /// Firmware version string.
    pub fw_version: &'static str,
    /// Board identifier (see [`board`]).
    pub board: u8,
    /// MCU name, e.g. `"STM32F407"`.
    pub mcu: &'static str,
    /// Cylinder count of this build.
    pub cylinders: u8,
    /// Capability bit flags (see [`capability`]).
    pub capabilities: u32,
    /// Unique device ID (MCU UID).
    pub device_id: [u8; 12],
}

impl DeviceIdentity {
    /// Identity used by the PC simulator.
    pub const fn sim(cylinders: u8) -> Self {
        Self {
            fw_version: "RustEMS 0.1.0",
            board: board::SIM,
            mcu: "x86-sim",
            cylinders,
            capabilities: capability::FUEL
                | capability::IGNITION
                | capability::BOOST
                | capability::SEQUENTIAL,
            device_id: [0x51, 0x4D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    }
}

/// Combined schema hash over parameter, table, and telemetry catalogs.
pub fn schema_hash() -> u32 {
    let mut h = FNV_OFFSET;
    h = fnv1a(h, &catalog_hash().to_le_bytes());
    h = fnv1a(h, &telemetry_hash().to_le_bytes());
    h
}

// ─── Handler context and side-effect actions ────────────────────────────────

/// Per-request environment supplied by the control loop / comms task.
pub struct RdpContext<'a> {
    /// Live (RAM-staged) configuration — writes take effect immediately.
    pub ram: &'a mut EngineConfig,
    /// Last persisted configuration (flash snapshot) for `ConfigDiscard`.
    pub flash: &'a EngineConfig,
    /// Build defaults for `ConfigResetDefaults`.
    pub defaults: &'a EngineConfig,
    /// Live telemetry snapshot.
    pub outputs: &'a OutputChannels,
    /// Milliseconds since boot.
    pub now_ms: u32,
    /// True while the engine is rotating (gates stopped-only operations).
    pub engine_running: bool,
}

/// Side effects the caller must perform after [`RdpServer::handle`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RdpActions {
    /// Persist the RAM config to flash.
    pub save: bool,
    /// Reboot the device.
    pub reboot: bool,
    /// Jump to the bootloader / DFU.
    pub enter_bootloader: bool,
}

/// Device-side RDP server state.
pub struct RdpServer {
    identity: DeviceIdentity,
    /// Telemetry subscriptions.
    pub streams: TelemetryStreams,
    /// Structured fault store.
    pub faults: FaultStore,
    /// Pending push events.
    pub events: EventQueue,
    /// Active control overrides.
    pub overrides: Overrides,
    /// Learned calibrations.
    pub calibrations: Calibrations,
    /// Bench test queued for the control loop (taken via [`RdpServer::take_bench_test`]).
    pending_bench: Option<BenchTestSpec>,
    dirty: bool,
    flash_crc: u32,
}

impl RdpServer {
    /// Create a server for a device identity. `flash_cfg` is the currently
    /// persisted configuration (used to seed the flash CRC).
    pub fn new(identity: DeviceIdentity, flash_cfg: &EngineConfig) -> Self {
        Self {
            identity,
            streams: TelemetryStreams::new(),
            faults: FaultStore::new(),
            events: EventQueue::new(),
            overrides: Overrides::new(),
            calibrations: Calibrations::default(),
            pending_bench: None,
            dirty: false,
            flash_crc: config_crc(flash_cfg),
        }
    }

    /// True when RAM config has unsaved edits.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Take a queued bench-test request (called by the control loop).
    pub fn take_bench_test(&mut self) -> Option<BenchTestSpec> {
        self.pending_bench.take()
    }

    /// Fail-safe on transport disconnect: drop overrides and subscriptions.
    pub fn on_disconnect(&mut self) {
        self.overrides.clear_all();
        self.streams.clear();
        self.pending_bench = None;
    }

    // ── Event/fault notification helpers (called by the control loop) ──────

    /// Raise a fault and queue a `FaultSet` event when newly active.
    pub fn note_fault(&mut self, code: u16, severity: Severity, detail: u16, now_ms: u32) {
        if self.faults.raise(code, severity, detail, now_ms) {
            self.events
                .push(EventKind::FaultSet, now_ms, code as i32, detail as i32);
        }
    }

    /// Resolve a fault and queue a `FaultCleared` event when it was active.
    pub fn note_fault_cleared(&mut self, code: u16, now_ms: u32) {
        if self.faults.resolve(code) {
            self.events
                .push(EventKind::FaultCleared, now_ms, code as i32, 0);
        }
    }

    /// Queue a knock event (`retard` in milli-degrees).
    pub fn note_knock(&mut self, cylinder: u8, retard_milli_deg: i32, now_ms: u32) {
        self.events
            .push(EventKind::Knock, now_ms, cylinder as i32, retard_milli_deg);
    }

    /// Queue a trigger sync transition event.
    pub fn note_sync(&mut self, gained: bool, teeth: u32, now_ms: u32) {
        self.events.push(
            EventKind::SyncState,
            now_ms,
            if gained { 1 } else { 0 },
            teeth as i32,
        );
    }

    /// Queue a protection-cut event.
    pub fn note_protection_cut(&mut self, reason: u16, now_ms: u32) {
        self.events
            .push(EventKind::ProtectionCut, now_ms, reason as i32, 0);
    }

    /// Queue a limp-mode transition event.
    pub fn note_limp(&mut self, reason: u16, now_ms: u32) {
        self.events
            .push(EventKind::LimpMode, now_ms, reason as i32, 0);
    }

    // ── Push frame emitters (called periodically by the comms task) ────────

    /// Encode the next due telemetry frame (`KIND=Telemetry OP=0x04F0`).
    /// Returns the payload length, or `None` when nothing is due.
    pub fn poll_telemetry(
        &mut self,
        outputs: &OutputChannels,
        now_ms: u32,
        out: &mut [u8],
    ) -> Option<usize> {
        if out.len() < MESSAGE_HEADER_LEN {
            return None;
        }
        let n = write_message_header(Kind::Telemetry, op::TELEM_FRAME, out).ok()?;
        let body_len = self
            .streams
            .encode_due_frame(outputs, now_ms, &mut out[n..])?;
        Some(n + body_len)
    }

    /// Encode the next pending event (`KIND=Event OP=0x06F0`).
    /// Returns the payload length, or `None` when the queue is empty.
    pub fn poll_event(&mut self, out: &mut [u8]) -> Option<usize> {
        if self.events.is_empty() || out.len() < MESSAGE_HEADER_LEN {
            return None;
        }
        let rec = self.events.pop()?;
        let n = write_message_header(Kind::Event, op::EVENT, out).ok()?;
        let body = bodies::EventBody {
            kind: rec.kind as u8,
            ts_ms: rec.ts_ms,
            a: rec.a,
            b: rec.b,
        };
        let body_len = encode_to_slice(&body, &mut out[n..])?;
        Some(n + body_len)
    }

    // ── Request dispatch ────────────────────────────────────────────────────

    /// Handle one decoded request payload, writing the response payload into
    /// `out` (recommended ≥ 2 KiB to fit a full 16×16 `TableGet`). Returns the
    /// response length and any side-effect actions.
    pub fn handle(
        &mut self,
        req: &[u8],
        ctx: &mut RdpContext<'_>,
        out: &mut [u8],
    ) -> (usize, RdpActions) {
        let mut actions = RdpActions::default();
        let Ok((header, body)) = read_message_header(req) else {
            return (0, actions);
        };
        if header.kind != Kind::Request {
            return (0, actions);
        }
        let opcode = header.op;
        let len = match opcode {
            op::HELLO => self.op_hello(opcode, out),
            op::PING => self.op_ping(opcode, body, ctx.now_ms, out),
            op::REBOOT => {
                actions.reboot = true;
                respond_status(opcode, ErrorCode::Ok, 0, out)
            }
            op::ENTER_BOOTLOADER => {
                match decode_from_slice::<bodies::EnterBootloaderRequest>(body) {
                    Some(r) if r.confirm == BOOTLOADER_CONFIRM => {
                        actions.enter_bootloader = true;
                        respond_status(opcode, ErrorCode::Ok, 0, out)
                    }
                    _ => respond_status(opcode, ErrorCode::BadRequest, 0, out),
                }
            }

            op::GET_SCHEMA_INFO => self.op_schema_info(opcode, out),
            op::GET_PARAM_CATALOG => self.op_param_catalog(opcode, body, out),
            op::GET_TABLE_CATALOG => self.op_table_catalog(opcode, body, out),
            op::GET_TELEMETRY_CATALOG => self.op_telemetry_catalog(opcode, body, out),

            op::PARAM_GET => self.op_param_get(opcode, body, ctx, out),
            op::PARAM_SET => self.op_param_set(opcode, body, ctx, out),
            op::TABLE_GET => self.op_table_get(opcode, body, ctx, out),
            op::TABLE_SET_CELL => self.op_table_set_cell(opcode, body, ctx, out),
            op::TABLE_SET_AXIS => self.op_table_set_axis(opcode, body, ctx, out),
            op::CONFIG_SAVE => {
                let crc = config_crc(ctx.ram);
                self.dirty = false;
                self.flash_crc = crc;
                actions.save = true;
                let resp = bodies::ConfigSaveResponse {
                    saved_bytes: core::mem::size_of::<EngineConfig>() as u32,
                    crc,
                };
                respond_body(opcode, &resp, out)
            }
            op::CONFIG_DISCARD => {
                *ctx.ram = ctx.flash.clone();
                self.dirty = false;
                respond_status(opcode, ErrorCode::Ok, 0, out)
            }
            op::CONFIG_RESET_DEFAULTS => {
                match decode_from_slice::<bodies::ConfigResetDefaultsRequest>(body) {
                    Some(r) if r.confirm == RESET_DEFAULTS_CONFIRM => {
                        if ctx.engine_running {
                            respond_status(opcode, ErrorCode::Busy, 0, out)
                        } else {
                            *ctx.ram = ctx.defaults.clone();
                            self.dirty = true;
                            respond_status(opcode, ErrorCode::Ok, 0, out)
                        }
                    }
                    _ => respond_status(opcode, ErrorCode::BadRequest, 0, out),
                }
            }
            op::CONFIG_STATUS => {
                let resp = bodies::ConfigStatusResponse {
                    dirty: self.dirty,
                    ram_crc: config_crc(ctx.ram),
                    flash_crc: self.flash_crc,
                };
                respond_body(opcode, &resp, out)
            }

            op::TELEM_SUBSCRIBE => self.op_subscribe(opcode, body, ctx.now_ms, out),
            op::TELEM_UNSUBSCRIBE => match decode_from_slice::<bodies::UnsubscribeRequest>(body) {
                Some(r) if self.streams.unsubscribe(r.stream_id) => {
                    respond_status(opcode, ErrorCode::Ok, 0, out)
                }
                Some(r) => respond_status(opcode, ErrorCode::NotFound, r.stream_id as u16, out),
                None => respond_status(opcode, ErrorCode::BadRequest, 0, out),
            },
            op::TELEM_READ_ONCE => self.op_read_once(opcode, body, ctx, out),

            op::BENCH_TEST => self.op_bench_test(opcode, body, ctx, out),
            op::SET_OVERRIDE => match decode_from_slice::<bodies::SetOverrideRequest>(body) {
                Some(r) => match OverrideTarget::from_u8(r.target) {
                    Some(target) => {
                        self.overrides
                            .set(target, r.value, r.timeout_ms, ctx.now_ms);
                        respond_status(opcode, ErrorCode::Ok, 0, out)
                    }
                    None => respond_status(opcode, ErrorCode::NotFound, r.target as u16, out),
                },
                None => respond_status(opcode, ErrorCode::BadRequest, 0, out),
            },
            op::CLEAR_OVERRIDE => match decode_from_slice::<bodies::ClearOverrideRequest>(body) {
                Some(r) => match OverrideTarget::from_u8(r.target) {
                    Some(target) => {
                        self.overrides.clear(target);
                        respond_status(opcode, ErrorCode::Ok, 0, out)
                    }
                    None => respond_status(opcode, ErrorCode::NotFound, r.target as u16, out),
                },
                None => respond_status(opcode, ErrorCode::BadRequest, 0, out),
            },
            op::CALIBRATE => self.op_calibrate(opcode, body, ctx, out),

            op::GET_FAULTS => self.op_get_faults(opcode, out),
            op::CLEAR_FAULTS => match decode_from_slice::<bodies::ClearFaultsRequest>(body) {
                Some(r) => {
                    let cleared = self.faults.clear_mask(r.mask);
                    respond_body(opcode, &bodies::ClearFaultsResponse { cleared }, out)
                }
                None => respond_status(opcode, ErrorCode::BadRequest, 0, out),
            },

            _ => respond_status(opcode, ErrorCode::UnknownOp, 0, out),
        };
        (len, actions)
    }

    // ── System ──────────────────────────────────────────────────────────────

    fn op_hello(&self, opcode: u16, out: &mut [u8]) -> usize {
        let info = bodies::HelloInfo {
            proto_major: PROTO_MAJOR,
            proto_minor: PROTO_MINOR,
            fw_version: self.identity.fw_version,
            board: self.identity.board,
            mcu: self.identity.mcu,
            cylinders: self.identity.cylinders,
            capabilities: self.identity.capabilities,
            schema_hash: schema_hash(),
            max_payload: MAX_PAYLOAD_LEN as u16,
            device_id: &self.identity.device_id,
        };
        respond_body(opcode, &info, out)
    }

    fn op_ping(&self, opcode: u16, body: &[u8], now_ms: u32, out: &mut [u8]) -> usize {
        let nonce = decode_from_slice::<bodies::PingRequest>(body)
            .map(|r| r.nonce)
            .unwrap_or(0);
        respond_body(
            opcode,
            &bodies::PingResponse {
                nonce,
                uptime_ms: now_ms,
            },
            out,
        )
    }

    // ── Descriptor ──────────────────────────────────────────────────────────

    fn op_schema_info(&self, opcode: u16, out: &mut [u8]) -> usize {
        let mut categories: heapless::Vec<bodies::CategoryDesc<'static>, 16> = heapless::Vec::new();
        for (i, name) in CATEGORIES.iter().enumerate() {
            let _ = categories.push(bodies::CategoryDesc { id: i as u8, name });
        }
        let resp = bodies::GetSchemaInfoResponse {
            schema_hash: schema_hash(),
            param_count: PARAM_CATALOG.len() as u16,
            table_count: TABLE_CATALOG.len() as u16,
            categories,
        };
        respond_body(opcode, &resp, out)
    }

    fn op_param_catalog(&self, opcode: u16, body: &[u8], out: &mut [u8]) -> usize {
        let page = decode_from_slice::<bodies::GetCatalogRequest>(body)
            .map(|r| r.page)
            .unwrap_or(0) as usize;
        let total_pages = PARAM_CATALOG.len().div_ceil(PARAMS_PER_PAGE).max(1);
        if page >= total_pages {
            return respond_status(opcode, ErrorCode::NotFound, page as u16, out);
        }
        let mut items: heapless::Vec<bodies::ParamDesc<'static>, 8> = heapless::Vec::new();
        for m in PARAM_CATALOG
            .iter()
            .skip(page * PARAMS_PER_PAGE)
            .take(PARAMS_PER_PAGE)
        {
            let _ = items.push(bodies::ParamDesc {
                id: m.id.as_u16(),
                key: m.key,
                label: m.label,
                category: m.category,
                vtype: ValueType::F32,
                unit: m.unit,
                scale: 1.0,
                offset: 0.0,
                min: m.min,
                max: m.max,
                default: m.default,
                digits: m.digits,
                flags: m.flags,
                enum_labels: None,
            });
        }
        let resp = bodies::GetParamCatalogResponse {
            page: page as u16,
            total_pages: total_pages as u16,
            items,
        };
        respond_body(opcode, &resp, out)
    }

    fn op_table_catalog(&self, opcode: u16, body: &[u8], out: &mut [u8]) -> usize {
        let page = decode_from_slice::<bodies::GetCatalogRequest>(body)
            .map(|r| r.page)
            .unwrap_or(0) as usize;
        let total_pages = TABLE_CATALOG.len().div_ceil(TABLES_PER_PAGE).max(1);
        if page >= total_pages {
            return respond_status(opcode, ErrorCode::NotFound, page as u16, out);
        }
        let mut items: heapless::Vec<bodies::TableDesc<'static>, 8> = heapless::Vec::new();
        for m in TABLE_CATALOG
            .iter()
            .skip(page * TABLES_PER_PAGE)
            .take(TABLES_PER_PAGE)
        {
            let _ = items.push(bodies::TableDesc {
                id: m.id.as_u16(),
                key: m.key,
                label: m.label,
                category: m.category,
                dims: m.dims,
                x_size: m.x_size,
                y_size: m.y_size,
                x_axis_key: m.x_axis_key,
                y_axis_key: m.y_axis_key,
                x_unit: m.x_unit,
                y_unit: m.y_unit,
                cell_unit: m.cell_unit,
                cell_min: m.cell_min,
                cell_max: m.cell_max,
                cell_digits: m.cell_digits,
            });
        }
        let resp = bodies::GetTableCatalogResponse {
            page: page as u16,
            total_pages: total_pages as u16,
            items,
        };
        respond_body(opcode, &resp, out)
    }

    fn op_telemetry_catalog(&self, opcode: u16, body: &[u8], out: &mut [u8]) -> usize {
        let page = decode_from_slice::<bodies::GetCatalogRequest>(body)
            .map(|r| r.page)
            .unwrap_or(0) as usize;
        let total_pages = TELEMETRY_CATALOG.len().div_ceil(CHANNELS_PER_PAGE).max(1);
        if page >= total_pages {
            return respond_status(opcode, ErrorCode::NotFound, page as u16, out);
        }
        let mut items: heapless::Vec<bodies::ChannelDesc<'static>, 8> = heapless::Vec::new();
        for c in TELEMETRY_CATALOG
            .iter()
            .skip(page * CHANNELS_PER_PAGE)
            .take(CHANNELS_PER_PAGE)
        {
            let vtype = match c.wire {
                crate::comms::telemetry::WireType::U16 => ValueType::U16,
                crate::comms::telemetry::WireType::I16 => ValueType::I16,
                crate::comms::telemetry::WireType::Bit => ValueType::Bool,
            };
            let _ = items.push(bodies::ChannelDesc {
                id: c.id,
                key: c.key,
                label: c.label,
                category: c.group,
                vtype,
                unit: c.unit,
                scale: c.scale,
                offset: 0.0,
                digits: 0,
            });
        }
        let resp = bodies::GetTelemetryCatalogResponse {
            page: page as u16,
            total_pages: total_pages as u16,
            items,
        };
        respond_body(opcode, &resp, out)
    }

    // ── Config ──────────────────────────────────────────────────────────────

    fn op_param_get(
        &self,
        opcode: u16,
        body: &[u8],
        ctx: &RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::ParamGetRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        // Raw f32 LE storage backing the borrowed ParamValue entries.
        let mut raw = [[0u8; 4]; 32];
        for (i, &id_raw) in req.ids.iter().enumerate() {
            let value = ParamId::from_u16(id_raw).and_then(|id| params::get_param(ctx.ram, id));
            let Some(v) = value else {
                return respond_status(opcode, ErrorCode::NotFound, id_raw, out);
            };
            raw[i] = v.to_le_bytes();
        }
        let mut values: heapless::Vec<bodies::ParamValue<'_>, 32> = heapless::Vec::new();
        for (i, _) in req.ids.iter().enumerate() {
            let _ = values.push(bodies::ParamValue {
                vtype: ValueType::F32,
                raw: &raw[i],
            });
        }
        let resp = bodies::ParamGetResponse { values };
        let len = respond_body(opcode, &resp, out);
        drop(resp);
        len
    }

    fn op_param_set(
        &mut self,
        opcode: u16,
        body: &[u8],
        ctx: &mut RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::ParamSetRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let mut results: heapless::Vec<bodies::ParamSetResult, 32> = heapless::Vec::new();
        for entry in req.entries.iter() {
            let code = self.apply_param_set(entry, ctx);
            if code == ErrorCode::Ok {
                self.dirty = true;
            }
            let _ = results.push(bodies::ParamSetResult { id: entry.id, code });
        }
        respond_body(opcode, &bodies::ParamSetResponse { results }, out)
    }

    fn apply_param_set(
        &self,
        entry: &bodies::ParamSetEntry<'_>,
        ctx: &mut RdpContext<'_>,
    ) -> ErrorCode {
        let Some(id) = ParamId::from_u16(entry.id) else {
            return ErrorCode::NotFound;
        };
        let Some(meta) = param_meta(id) else {
            return ErrorCode::NotFound;
        };
        if meta.flags & PFLAG_READ_ONLY != 0 {
            return ErrorCode::ReadOnly;
        }
        if meta.flags & PFLAG_ENGINE_STOPPED_ONLY != 0 && ctx.engine_running {
            return ErrorCode::Busy;
        }
        if entry.value.vtype != ValueType::F32 || entry.value.raw.len() != 4 {
            return ErrorCode::BadRequest;
        }
        let value = f32::from_le_bytes([
            entry.value.raw[0],
            entry.value.raw[1],
            entry.value.raw[2],
            entry.value.raw[3],
        ]);
        if !(meta.min..=meta.max).contains(&value) {
            return ErrorCode::OutOfRange;
        }
        if set_param(ctx.ram, id, value) {
            ErrorCode::Ok
        } else {
            ErrorCode::NotFound
        }
    }

    fn op_table_get(
        &self,
        opcode: u16,
        body: &[u8],
        ctx: &RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::TableGetRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let Some(id) = TableId::from_u16(req.table_id) else {
            return respond_status(opcode, ErrorCode::NotFound, req.table_id, out);
        };
        let data = params::table_get(ctx.ram, id);
        let resp = bodies::TableGetResponse {
            x_axis: data.x_axis,
            y_axis: data.y_axis,
            cells: data.cells,
        };
        respond_body(opcode, &resp, out)
    }

    fn op_table_set_cell(
        &mut self,
        opcode: u16,
        body: &[u8],
        ctx: &mut RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::TableSetCellRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let Some(id) = TableId::from_u16(req.table_id) else {
            return respond_status(opcode, ErrorCode::NotFound, req.table_id, out);
        };
        match params::table_write_cell(ctx.ram, id, req.ix as usize, req.iy as usize, req.value) {
            Ok(()) => {
                self.dirty = true;
                respond_status(opcode, ErrorCode::Ok, 0, out)
            }
            Err(TableWriteError::OutOfRange) => {
                respond_status(opcode, ErrorCode::OutOfRange, req.table_id, out)
            }
            Err(TableWriteError::BadIndex) => {
                respond_status(opcode, ErrorCode::BadRequest, req.ix, out)
            }
        }
    }

    fn op_table_set_axis(
        &mut self,
        opcode: u16,
        body: &[u8],
        ctx: &mut RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::TableSetAxisRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let Some(id) = TableId::from_u16(req.table_id) else {
            return respond_status(opcode, ErrorCode::NotFound, req.table_id, out);
        };
        let axis = if req.axis == 0 {
            TableAxis::X
        } else {
            TableAxis::Y
        };
        match params::table_write_axis(ctx.ram, id, axis, &req.values) {
            Ok(()) => {
                self.dirty = true;
                respond_status(opcode, ErrorCode::Ok, 0, out)
            }
            Err(_) => respond_status(opcode, ErrorCode::BadRequest, req.axis as u16, out),
        }
    }

    // ── Telemetry ───────────────────────────────────────────────────────────

    fn op_subscribe(&mut self, opcode: u16, body: &[u8], now_ms: u32, out: &mut [u8]) -> usize {
        let Some(req) = decode_from_slice::<bodies::SubscribeRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        match self.streams.subscribe(&req.channels, req.rate_hz, now_ms) {
            Ok((stream_id, rate_hz)) => {
                let resp = bodies::SubscribeResponse {
                    stream_id,
                    layout: req.channels.clone(),
                    rate_hz,
                };
                respond_body(opcode, &resp, out)
            }
            Err(SubscribeError::BadChannel(ch)) => {
                respond_status(opcode, ErrorCode::NotFound, ch, out)
            }
            Err(SubscribeError::NoFreeStream) => respond_status(opcode, ErrorCode::Busy, 0, out),
            Err(SubscribeError::TooManyChannels) => {
                respond_status(opcode, ErrorCode::BadRequest, 0, out)
            }
        }
    }

    fn op_read_once(
        &self,
        opcode: u16,
        body: &[u8],
        ctx: &RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::ReadOnceRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let mut values: heapless::Vec<f32, 32> = heapless::Vec::new();
        for &ch in req.channels.iter() {
            let Some(v) = crate::comms::telemetry::channel_value(ctx.outputs, ch) else {
                return respond_status(opcode, ErrorCode::NotFound, ch, out);
            };
            let _ = values.push(v);
        }
        respond_body(opcode, &bodies::ReadOnceResponse { values }, out)
    }

    // ── Control ─────────────────────────────────────────────────────────────

    fn op_bench_test(
        &mut self,
        opcode: u16,
        body: &[u8],
        ctx: &RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::BenchTestRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        if ctx.engine_running {
            return respond_status(opcode, ErrorCode::Busy, 0, out);
        }
        let Some(target) = BenchTarget::from_u8(req.target) else {
            return respond_status(opcode, ErrorCode::NotFound, req.target as u16, out);
        };
        self.pending_bench = Some(BenchTestSpec {
            target,
            index: req.index,
            on_ms: req.on_ms,
            off_ms: req.off_ms,
            count: req.count,
        });
        respond_status(opcode, ErrorCode::Ok, 0, out)
    }

    fn op_calibrate(
        &mut self,
        opcode: u16,
        body: &[u8],
        ctx: &RdpContext<'_>,
        out: &mut [u8],
    ) -> usize {
        let Some(req) = decode_from_slice::<bodies::CalibrateRequest>(body) else {
            return respond_status(opcode, ErrorCode::BadRequest, 0, out);
        };
        let Some(routine) = CalibrateRoutine::from_u8(req.routine) else {
            return respond_status(opcode, ErrorCode::NotFound, req.routine as u16, out);
        };
        let mut result: heapless::Vec<f32, 8> = heapless::Vec::new();
        match routine {
            CalibrateRoutine::TpsClosed => {
                self.calibrations.tps_closed_pct = ctx.outputs.tps_pct;
                let _ = result.push(ctx.outputs.tps_pct);
            }
            CalibrateRoutine::TpsOpen => {
                self.calibrations.tps_open_pct = ctx.outputs.tps_pct;
                let _ = result.push(ctx.outputs.tps_pct);
            }
            CalibrateRoutine::MapBaro => {
                if ctx.engine_running {
                    return respond_status(opcode, ErrorCode::Busy, 0, out);
                }
                self.calibrations.baro_kpa = ctx.outputs.map_kpa;
                let _ = result.push(ctx.outputs.map_kpa);
            }
            CalibrateRoutine::ClearAdaptive => {
                self.calibrations.clear_adaptive_pending = true;
            }
        }
        respond_body(opcode, &bodies::CalibrateResponse { result }, out)
    }

    // ── Diagnostics ─────────────────────────────────────────────────────────

    fn op_get_faults(&self, opcode: u16, out: &mut [u8]) -> usize {
        let mut faults: heapless::Vec<bodies::FaultEntry, 16> = heapless::Vec::new();
        for rec in self.faults.iter() {
            let _ = faults.push(bodies::FaultEntry {
                code: rec.code,
                severity: rec.severity as u8,
                active: rec.active,
                count: rec.count,
                first_ts_ms: rec.first_ts_ms,
                last_ts_ms: rec.last_ts_ms,
                detail: rec.detail,
            });
        }
        respond_body(opcode, &bodies::GetFaultsResponse { faults }, out)
    }
}

// ─── Response encoding helpers ───────────────────────────────────────────────

/// Write `KIND=Response OP body` where `body` is a CBOR-encoded value.
fn respond_body<T: rusefi_device_api::minicbor::Encode<()>>(
    opcode: u16,
    body: &T,
    out: &mut [u8],
) -> usize {
    let Ok(n) = write_message_header(Kind::Response, opcode, out) else {
        return 0;
    };
    match encode_to_slice(body, &mut out[n..]) {
        Some(body_len) => n + body_len,
        // Body did not fit: degrade to a Fragmentation error status.
        None => respond_status(opcode, ErrorCode::Fragmentation, 0, out),
    }
}

/// Write a plain status response (`ErrorResponseBody` with no message).
fn respond_status(opcode: u16, code: ErrorCode, detail: u16, out: &mut [u8]) -> usize {
    let Ok(n) = write_message_header(Kind::Response, opcode, out) else {
        return 0;
    };
    let body = bodies::ErrorResponseBody {
        code,
        detail,
        message: None,
    };
    encode_to_slice(&body, &mut out[n..])
        .map(|l| n + l)
        .unwrap_or(0)
}

// ─── Request building helpers (shared with host-side tests/tools) ───────────

/// Write `KIND=Request OP` followed by a CBOR body. Returns the payload length.
pub fn build_request<T: rusefi_device_api::minicbor::Encode<()>>(
    opcode: u16,
    body: &T,
    out: &mut [u8],
) -> Option<usize> {
    let n = write_message_header(Kind::Request, opcode, out).ok()?;
    let body_len = encode_to_slice(body, &mut out[n..])?;
    Some(n + body_len)
}

/// Write a body-less `KIND=Request OP`. Returns the payload length.
pub fn build_request_empty(opcode: u16, out: &mut [u8]) -> Option<usize> {
    write_message_header(Kind::Request, opcode, out).ok()
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comms::output::OutputChannels;

    fn sim_identity() -> DeviceIdentity {
        DeviceIdentity {
            fw_version: "RustEMS 0.1.0-test",
            board: board::SIM,
            mcu: "x86-sim",
            cylinders: 4,
            capabilities: capability::FUEL | capability::IGNITION,
            device_id: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
        }
    }

    struct Fixture {
        server: RdpServer,
        ram: EngineConfig,
        flash: EngineConfig,
        defaults: EngineConfig,
        outputs: OutputChannels,
        now_ms: u32,
        engine_running: bool,
    }

    impl Fixture {
        fn new() -> Self {
            let cfg = EngineConfig::default_4cyl();
            Self {
                server: RdpServer::new(sim_identity(), &cfg),
                ram: cfg.clone(),
                flash: cfg.clone(),
                defaults: cfg,
                outputs: OutputChannels::zeroed(),
                now_ms: 1000,
                engine_running: false,
            }
        }

        fn handle(&mut self, req: &[u8], out: &mut [u8]) -> (usize, RdpActions) {
            let mut ctx = RdpContext {
                ram: &mut self.ram,
                flash: &self.flash,
                defaults: &self.defaults,
                outputs: &self.outputs,
                now_ms: self.now_ms,
                engine_running: self.engine_running,
            };
            self.server.handle(req, &mut ctx, out)
        }
    }

    fn parse_response(buf: &[u8]) -> (u16, &[u8]) {
        let (h, body) = read_message_header(buf).unwrap();
        assert_eq!(h.kind, Kind::Response);
        (h.op, body)
    }

    fn status_of(body: &[u8]) -> ErrorCode {
        decode_from_slice::<bodies::ErrorResponseBody>(body)
            .unwrap()
            .code
    }

    #[test]
    fn hello_returns_identity_and_schema_hash() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 16];
        let n = build_request_empty(op::HELLO, &mut req).unwrap();
        let mut out = [0u8; 256];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (opcode, body) = parse_response(&out[..len]);
        assert_eq!(opcode, op::HELLO);
        let info: bodies::HelloInfo = decode_from_slice(body).unwrap();
        assert_eq!(info.proto_major, PROTO_MAJOR);
        assert_eq!(info.cylinders, 4);
        assert_eq!(info.schema_hash, schema_hash());
        assert_eq!(info.board, board::SIM);
    }

    #[test]
    fn ping_echoes_nonce_and_uptime() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 32];
        let n = build_request(op::PING, &bodies::PingRequest { nonce: 0xCAFE }, &mut req).unwrap();
        let mut out = [0u8; 64];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::PingResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.nonce, 0xCAFE);
        assert_eq!(resp.uptime_ms, 1000);
    }

    #[test]
    fn catalogs_cover_all_items_via_paging() {
        let mut fx = Fixture::new();
        // Params
        let mut total_params = 0usize;
        let mut page = 0u16;
        loop {
            let mut req = [0u8; 32];
            let n = build_request(
                op::GET_PARAM_CATALOG,
                &bodies::GetCatalogRequest { page },
                &mut req,
            )
            .unwrap();
            let mut out = [0u8; 1024];
            let (len, _) = fx.handle(&req[..n], &mut out);
            let (_, body) = parse_response(&out[..len]);
            let resp: bodies::GetParamCatalogResponse = decode_from_slice(body).unwrap();
            total_params += resp.items.len();
            page += 1;
            if page >= resp.total_pages {
                break;
            }
        }
        assert_eq!(total_params, PARAM_CATALOG.len());

        // Telemetry channels
        let mut total_ch = 0usize;
        let mut page = 0u16;
        loop {
            let mut req = [0u8; 32];
            let n = build_request(
                op::GET_TELEMETRY_CATALOG,
                &bodies::GetCatalogRequest { page },
                &mut req,
            )
            .unwrap();
            let mut out = [0u8; 1024];
            let (len, _) = fx.handle(&req[..n], &mut out);
            let (_, body) = parse_response(&out[..len]);
            let resp: bodies::GetTelemetryCatalogResponse = decode_from_slice(body).unwrap();
            total_ch += resp.items.len();
            page += 1;
            if page >= resp.total_pages {
                break;
            }
        }
        assert_eq!(total_ch, TELEMETRY_CATALOG.len());
    }

    #[test]
    fn param_get_set_round_trip() {
        let mut fx = Fixture::new();
        // Set cranking RPM to 350
        let raw = 350.0f32.to_le_bytes();
        let mut entries = heapless::Vec::new();
        let _ = entries.push(bodies::ParamSetEntry {
            id: ParamId::CrankingRpm.as_u16(),
            value: bodies::ParamValue {
                vtype: ValueType::F32,
                raw: &raw,
            },
        });
        let mut req = [0u8; 64];
        let n = build_request(
            op::PARAM_SET,
            &bodies::ParamSetRequest { entries },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::ParamSetResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.results[0].code, ErrorCode::Ok);
        assert_eq!(fx.ram.cranking_rpm, 350.0);
        assert!(fx.server.is_dirty());

        // Read it back
        let mut ids = heapless::Vec::new();
        let _ = ids.push(ParamId::CrankingRpm.as_u16());
        let n = build_request(op::PARAM_GET, &bodies::ParamGetRequest { ids }, &mut req).unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::ParamGetResponse = decode_from_slice(body).unwrap();
        let v = f32::from_le_bytes(resp.values[0].raw.try_into().unwrap());
        assert_eq!(v, 350.0);
    }

    #[test]
    fn param_set_validates_range_and_running_state() {
        let mut fx = Fixture::new();
        // Out of range
        let raw = 9999.0f32.to_le_bytes();
        let mut entries = heapless::Vec::new();
        let _ = entries.push(bodies::ParamSetEntry {
            id: ParamId::CrankingRpm.as_u16(),
            value: bodies::ParamValue {
                vtype: ValueType::F32,
                raw: &raw,
            },
        });
        let mut req = [0u8; 64];
        let n = build_request(
            op::PARAM_SET,
            &bodies::ParamSetRequest { entries },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::ParamSetResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.results[0].code, ErrorCode::OutOfRange);

        // Engine-stopped-only param while running → Busy
        fx.engine_running = true;
        let raw = 60.0f32.to_le_bytes();
        let mut entries = heapless::Vec::new();
        let _ = entries.push(bodies::ParamSetEntry {
            id: ParamId::TriggerTotalTeeth.as_u16(),
            value: bodies::ParamValue {
                vtype: ValueType::F32,
                raw: &raw,
            },
        });
        let n = build_request(
            op::PARAM_SET,
            &bodies::ParamSetRequest { entries },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::ParamSetResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.results[0].code, ErrorCode::Busy);
    }

    #[test]
    fn table_get_and_set_cell() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 64];
        let n = build_request(
            op::TABLE_SET_CELL,
            &bodies::TableSetCellRequest {
                table_id: TableId::Ignition.as_u16(),
                ix: 4,
                iy: 2,
                value: 28.5,
            },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 4096];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        assert_eq!(fx.ram.ignition_table[2][4], 28.5);

        let n = build_request(
            op::TABLE_GET,
            &bodies::TableGetRequest {
                table_id: TableId::Ignition.as_u16(),
            },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::TableGetResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.x_axis.len(), 16);
        assert_eq!(resp.y_axis.len(), 16);
        assert_eq!(resp.cells.len(), 256);
        assert_eq!(resp.cells[2 * 16 + 4], 28.5);
    }

    #[test]
    fn config_transaction_save_discard_status() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 64];
        let mut out = [0u8; 256];

        // Initially clean
        let n = build_request_empty(op::CONFIG_STATUS, &mut req).unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let st: bodies::ConfigStatusResponse = decode_from_slice(body).unwrap();
        assert!(!st.dirty);
        assert_eq!(st.ram_crc, st.flash_crc);

        // Edit a cell → dirty
        let n = build_request(
            op::TABLE_SET_CELL,
            &bodies::TableSetCellRequest {
                table_id: TableId::Ve.as_u16(),
                ix: 0,
                iy: 0,
                value: 0.95,
            },
            &mut req,
        )
        .unwrap();
        let _ = fx.handle(&req[..n], &mut out);

        let n = build_request_empty(op::CONFIG_STATUS, &mut req).unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let st: bodies::ConfigStatusResponse = decode_from_slice(body).unwrap();
        assert!(st.dirty);
        assert_ne!(st.ram_crc, st.flash_crc);

        // Save → action flag + clean
        let n = build_request_empty(op::CONFIG_SAVE, &mut req).unwrap();
        let (len, actions) = fx.handle(&req[..n], &mut out);
        assert!(actions.save);
        let (_, body) = parse_response(&out[..len]);
        let saved: bodies::ConfigSaveResponse = decode_from_slice(body).unwrap();
        assert_eq!(saved.crc, config_crc(&fx.ram));
        assert!(!fx.server.is_dirty());

        // Edit again then discard → RAM reverts to flash snapshot
        let n = build_request(
            op::TABLE_SET_CELL,
            &bodies::TableSetCellRequest {
                table_id: TableId::Ve.as_u16(),
                ix: 1,
                iy: 1,
                value: 1.2,
            },
            &mut req,
        )
        .unwrap();
        let _ = fx.handle(&req[..n], &mut out);
        assert_eq!(fx.ram.ve_table[1][1], 1.2);

        let n = build_request_empty(op::CONFIG_DISCARD, &mut req).unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        assert_eq!(fx.ram.ve_table[1][1], fx.flash.ve_table[1][1]);
        assert!(!fx.server.is_dirty());
    }

    #[test]
    fn reset_defaults_requires_confirm_and_stopped_engine() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];

        // Wrong magic
        let n = build_request(
            op::CONFIG_RESET_DEFAULTS,
            &bodies::ConfigResetDefaultsRequest { confirm: 0x1234 },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::BadRequest);

        // Running engine
        fx.engine_running = true;
        let n = build_request(
            op::CONFIG_RESET_DEFAULTS,
            &bodies::ConfigResetDefaultsRequest {
                confirm: RESET_DEFAULTS_CONFIRM,
            },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Busy);

        // OK when stopped
        fx.engine_running = false;
        fx.ram.cranking_rpm = 700.0;
        let n = build_request(
            op::CONFIG_RESET_DEFAULTS,
            &bodies::ConfigResetDefaultsRequest {
                confirm: RESET_DEFAULTS_CONFIRM,
            },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        assert_eq!(fx.ram.cranking_rpm, fx.defaults.cranking_rpm);
    }

    #[test]
    fn telemetry_subscribe_and_push() {
        let mut fx = Fixture::new();
        fx.outputs.rpm = 4200.0;
        let mut channels = heapless::Vec::new();
        let _ = channels.push(1u16); // rpm
        let mut req = [0u8; 64];
        let n = build_request(
            op::TELEM_SUBSCRIBE,
            &bodies::SubscribeRequest {
                channels,
                rate_hz: 50,
            },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let sub: bodies::SubscribeResponse = decode_from_slice(body).unwrap();
        assert_eq!(sub.stream_id, 1);
        assert_eq!(sub.rate_hz, 50);
        assert_eq!(&sub.layout[..], &[1u16]);

        // Push a frame
        let mut frame = [0u8; 64];
        let outputs = fx.outputs;
        let flen = fx
            .server
            .poll_telemetry(&outputs, 2000, &mut frame)
            .unwrap();
        let (h, fbody) = read_message_header(&frame[..flen]).unwrap();
        assert_eq!(h.kind, Kind::Telemetry);
        assert_eq!(h.op, op::TELEM_FRAME);
        assert_eq!(fbody[0], 1); // stream id
        let rpm = u16::from_le_bytes([fbody[7], fbody[8]]);
        assert_eq!(rpm, 4200);
    }

    #[test]
    fn read_once_returns_values() {
        let mut fx = Fixture::new();
        fx.outputs.clt_c = 88.0;
        let mut channels = heapless::Vec::new();
        let _ = channels.push(2u16);
        let mut req = [0u8; 64];
        let n = build_request(
            op::TELEM_READ_ONCE,
            &bodies::ReadOnceRequest { channels },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::ReadOnceResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.values[0], 88.0);
    }

    #[test]
    fn bench_test_rejected_while_running() {
        let mut fx = Fixture::new();
        fx.engine_running = true;
        let mut req = [0u8; 64];
        let n = build_request(
            op::BENCH_TEST,
            &bodies::BenchTestRequest {
                target: 0,
                index: 1,
                on_ms: 3,
                off_ms: 100,
                count: 5,
            },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Busy);

        fx.engine_running = false;
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        let bench = fx.server.take_bench_test().unwrap();
        assert_eq!(bench.target, BenchTarget::Injector);
        assert_eq!(bench.count, 5);
    }

    #[test]
    fn overrides_set_and_clear() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 64];
        let n = build_request(
            op::SET_OVERRIDE,
            &bodies::SetOverrideRequest {
                target: 2,
                value: 10.0,
                timeout_ms: 5000,
            },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 128];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        assert_eq!(
            fx.server.overrides.get(OverrideTarget::TimingFix, 1000),
            Some(10.0)
        );

        let n = build_request(
            op::CLEAR_OVERRIDE,
            &bodies::ClearOverrideRequest { target: 2 },
            &mut req,
        )
        .unwrap();
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::Ok);
        assert_eq!(
            fx.server.overrides.get(OverrideTarget::TimingFix, 1000),
            None
        );
    }

    #[test]
    fn faults_and_events_flow() {
        let mut fx = Fixture::new();
        fx.server.note_fault(
            crate::comms::faults::fault_code::OVER_TEMP,
            Severity::Critical,
            0,
            500,
        );
        fx.server.note_knock(3, 4000, 600);

        // GetFaults lists the fault
        let mut req = [0u8; 32];
        let n = build_request_empty(op::GET_FAULTS, &mut req).unwrap();
        let mut out = [0u8; 512];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (_, body) = parse_response(&out[..len]);
        let resp: bodies::GetFaultsResponse = decode_from_slice(body).unwrap();
        assert_eq!(resp.faults.len(), 1);
        assert!(resp.faults[0].active);

        // Events pop in order: FaultSet, then Knock
        let mut ev = [0u8; 64];
        let elen = fx.server.poll_event(&mut ev).unwrap();
        let (h, ebody) = read_message_header(&ev[..elen]).unwrap();
        assert_eq!(h.kind, Kind::Event);
        let e: bodies::EventBody = decode_from_slice(ebody).unwrap();
        assert_eq!(e.kind, EventKind::FaultSet as u8);

        let elen = fx.server.poll_event(&mut ev).unwrap();
        let (_, ebody) = read_message_header(&ev[..elen]).unwrap();
        let e: bodies::EventBody = decode_from_slice(ebody).unwrap();
        assert_eq!(e.kind, EventKind::Knock as u8);
        assert_eq!(e.a, 3);
        assert_eq!(e.b, 4000);

        assert!(fx.server.poll_event(&mut ev).is_none());
    }

    #[test]
    fn unknown_op_reports_error() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 16];
        let n = build_request_empty(0x7F7F, &mut req).unwrap();
        let mut out = [0u8; 64];
        let (len, _) = fx.handle(&req[..n], &mut out);
        let (opcode, body) = parse_response(&out[..len]);
        assert_eq!(opcode, 0x7F7F);
        assert_eq!(status_of(body), ErrorCode::UnknownOp);
    }

    #[test]
    fn bootloader_requires_magic() {
        let mut fx = Fixture::new();
        let mut req = [0u8; 32];
        let n = build_request(
            op::ENTER_BOOTLOADER,
            &bodies::EnterBootloaderRequest { confirm: 0 },
            &mut req,
        )
        .unwrap();
        let mut out = [0u8; 64];
        let (len, actions) = fx.handle(&req[..n], &mut out);
        assert!(!actions.enter_bootloader);
        let (_, body) = parse_response(&out[..len]);
        assert_eq!(status_of(body), ErrorCode::BadRequest);

        let n = build_request(
            op::ENTER_BOOTLOADER,
            &bodies::EnterBootloaderRequest {
                confirm: BOOTLOADER_CONFIRM,
            },
            &mut req,
        )
        .unwrap();
        let (_, actions) = fx.handle(&req[..n], &mut out);
        assert!(actions.enter_bootloader);
    }

    #[test]
    fn disconnect_failsafe_clears_state() {
        let mut fx = Fixture::new();
        fx.server
            .overrides
            .set(OverrideTarget::SparkCut, 1.0, 10_000, 0);
        let _ = fx.server.streams.subscribe(&[1], 10, 0);
        fx.server.on_disconnect();
        assert!(!fx.server.overrides.any_active());
        assert_eq!(fx.server.streams.active(), 0);
    }
}
