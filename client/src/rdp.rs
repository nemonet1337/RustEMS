//! RDP (RustEMS Device Protocol) host-side client.
//!
//! Speaks the new self-describing device protocol (`docs/api/02..05`) over any
//! async byte stream (TCP, serial, in-process duplex). The wire layer (COBS
//! framing, CRC16, fragmentation, CBOR bodies) is shared with the firmware via
//! the `rusefi-device-api` crate.
//!
//! # Usage
//!
//! ```no_run
//! # async fn demo() -> Result<(), rusefi_client::rdp::RdpError> {
//! use rusefi_client::rdp::RdpClient;
//!
//! let mut ecu = RdpClient::<tokio::net::TcpStream>::connect("127.0.0.1", 29002).await?;
//! let hello = ecu.hello().await?;
//! println!("connected to {} ({} cylinders)", hello.fw_version, hello.cylinders);
//!
//! let (stream_id, layout, rate) = ecu.subscribe(&[1, 2, 7], 25).await?;
//! # let _ = (stream_id, layout, rate);
//! # Ok(())
//! # }
//! ```

use std::collections::{HashMap, VecDeque};

use rusefi_device_api::cbor as bodies;
use rusefi_device_api::cbor::{decode_from_slice, encode_to_slice};
use rusefi_device_api::frame::{decode_frame, encode_message, Flags, MAX_RAW_FRAME_LEN};
use rusefi_device_api::message::{
    op, read_message_header, write_message_header, Kind, MESSAGE_HEADER_LEN,
};
use rusefi_device_api::{Defragmenter, FrameError};
use thiserror::Error;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub use rusefi_device_api::message::{ErrorCode, ValueType};

/// Confirmation magic for `ConfigResetDefaults` (see `docs/api/03`).
pub const RESET_DEFAULTS_CONFIRM: u32 = 0xDEFA;

/// Maximum reassembled message size accepted from the device.
const MAX_MESSAGE_LEN: usize = 4096;

// ─── Errors ──────────────────────────────────────────────────────────────────

/// Errors from RDP client operations.
#[derive(Debug, Error)]
pub enum RdpError {
    /// Transport I/O failure.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// The device closed the connection.
    #[error("connection closed by device")]
    ConnectionClosed,

    /// Frame encode failure (request too large, etc.).
    #[error("framing error: {0:?}")]
    Frame(FrameError),

    /// Request body could not be encoded.
    #[error("request encode failed (op 0x{op:04X})")]
    Encode {
        /// Opcode of the failing request.
        op: u16,
    },

    /// Response body could not be decoded.
    #[error("response decode failed (op 0x{op:04X})")]
    Decode {
        /// Opcode of the failing response.
        op: u16,
    },

    /// The device returned a non-Ok error response.
    #[error("device error {code:?} (detail {detail})")]
    Device {
        /// Standard RDP error code.
        code: ErrorCode,
        /// Context-dependent detail (offending id, etc.).
        detail: u16,
    },

    /// A response arrived with an unexpected opcode.
    #[error("unexpected response op: expected 0x{expected:04X}, got 0x{got:04X}")]
    UnexpectedOp {
        /// Opcode of the in-flight request.
        expected: u16,
        /// Opcode found in the response.
        got: u16,
    },

    /// More items were supplied than the protocol allows in one request.
    #[error("too many items in one request: {0}")]
    TooManyItems(usize),
}

// ─── Owned result types ──────────────────────────────────────────────────────

/// Owned copy of the device `HelloInfo`.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedHelloInfo {
    /// Protocol major version (mismatch → disconnect).
    pub proto_major: u8,
    /// Protocol minor version (backward-compatible additions).
    pub proto_minor: u8,
    /// Firmware version string.
    pub fw_version: String,
    /// Board identifier.
    pub board: u8,
    /// MCU name.
    pub mcu: String,
    /// Cylinder count.
    pub cylinders: u8,
    /// Capability bit flags.
    pub capabilities: u32,
    /// Schema hash over the parameter/table/telemetry catalogs.
    pub schema_hash: u32,
    /// Maximum frame payload supported by the device.
    pub max_payload: u16,
    /// Unique device id (MCU UID).
    pub device_id: Vec<u8>,
}

/// One schema category (UI tab/group).
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedCategory {
    /// Category index referenced by descriptors.
    pub id: u8,
    /// Category display name.
    pub name: String,
}

/// Owned copy of `GetSchemaInfoResponse`.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedSchemaInfo {
    /// Schema hash.
    pub schema_hash: u32,
    /// Number of parameters in the catalog.
    pub param_count: u16,
    /// Number of tables in the catalog.
    pub table_count: u16,
    /// UI categories.
    pub categories: Vec<OwnedCategory>,
}

/// Owned copy of a `ParamDesc` catalog entry.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedParamDesc {
    /// Stable parameter id.
    pub id: u16,
    /// Machine-readable key, e.g. `"fuel.injector_flow_cc_min"`.
    pub key: String,
    /// Display label.
    pub label: String,
    /// Category index.
    pub category: u8,
    /// Value type.
    pub vtype: ValueType,
    /// Physical unit.
    pub unit: String,
    /// `physical = raw * scale + offset`.
    pub scale: f32,
    /// `physical = raw * scale + offset`.
    pub offset: f32,
    /// Minimum allowed physical value.
    pub min: f32,
    /// Maximum allowed physical value.
    pub max: f32,
    /// Default physical value.
    pub default: f32,
    /// Display decimal digits.
    pub digits: u8,
    /// bit0=ReadOnly, bit1=needs reboot, bit2=engine-stopped only.
    pub flags: u8,
    /// Labels for enum-typed parameters.
    pub enum_labels: Option<Vec<String>>,
}

/// Owned copy of a `TableDesc` catalog entry.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedTableDesc {
    /// Stable table id.
    pub id: u16,
    /// Machine-readable key, e.g. `"fuel.ve_table"`.
    pub key: String,
    /// Display label.
    pub label: String,
    /// Category index.
    pub category: u8,
    /// 1 or 2 dimensions.
    pub dims: u8,
    /// Columns (x bins).
    pub x_size: u16,
    /// Rows (y bins, 1 for 1D).
    pub y_size: u16,
    /// X axis key.
    pub x_axis_key: String,
    /// Y axis key.
    pub y_axis_key: String,
    /// X axis unit.
    pub x_unit: String,
    /// Y axis unit.
    pub y_unit: String,
    /// Cell unit.
    pub cell_unit: String,
    /// Minimum cell value.
    pub cell_min: f32,
    /// Maximum cell value.
    pub cell_max: f32,
    /// Cell display digits.
    pub cell_digits: u8,
}

/// Owned copy of a telemetry `ChannelDesc`.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedChannelDesc {
    /// Stable channel id.
    pub id: u16,
    /// Machine-readable key, e.g. `"rpm"`.
    pub key: String,
    /// Display label.
    pub label: String,
    /// UI group.
    pub category: u8,
    /// Wire value type (U16/I16/Bool).
    pub vtype: ValueType,
    /// Physical unit.
    pub unit: String,
    /// `physical = wire_raw * scale`.
    pub scale: f32,
    /// Offset (normally 0).
    pub offset: f32,
    /// Display digits.
    pub digits: u8,
}

/// Owned table contents from `TableGet`.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedTableData {
    /// X axis breakpoints.
    pub x_axis: Vec<f32>,
    /// Y axis breakpoints (empty for 1D tables).
    pub y_axis: Vec<f32>,
    /// Cells in row-major (`[y][x]`) order.
    pub cells: Vec<f32>,
}

/// Owned fault (DTC) entry.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedFault {
    /// Fault code.
    pub code: u16,
    /// Severity (0=Info, 1=Warn, 2=Critical).
    pub severity: u8,
    /// True while the underlying condition persists.
    pub active: bool,
    /// Occurrence count.
    pub count: u16,
    /// First occurrence (device ms).
    pub first_ts_ms: u32,
    /// Most recent occurrence (device ms).
    pub last_ts_ms: u32,
    /// Context-dependent detail.
    pub detail: u16,
}

/// Owned asynchronous device event.
#[derive(Clone, Debug, PartialEq)]
pub struct OwnedEvent {
    /// Event kind discriminator.
    pub kind: u8,
    /// Device timestamp (ms since boot).
    pub ts_ms: u32,
    /// Kind-dependent value A.
    pub a: i32,
    /// Kind-dependent value B.
    pub b: i32,
}

/// One decoded telemetry push frame (physical values in layout order).
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetryFrame {
    /// Stream this frame belongs to.
    pub stream_id: u8,
    /// Per-stream sequence number (drop detection).
    pub seq: u16,
    /// Device timestamp (ms since boot).
    pub ts_ms: u32,
    /// Physical channel values, in the subscribed layout order.
    pub values: Vec<f32>,
}

/// A pushed message from the device (telemetry frame or async event).
#[derive(Clone, Debug, PartialEq)]
pub enum Push {
    /// Packed telemetry frame.
    Telemetry(TelemetryFrame),
    /// Asynchronous event.
    Event(OwnedEvent),
}

// ─── Client ──────────────────────────────────────────────────────────────────

/// Host-side RDP client over an async byte stream.
pub struct RdpClient<S: AsyncRead + AsyncWrite + Unpin> {
    stream: S,
    next_seq: u16,
    rx: Vec<u8>,
    defrag: Defragmenter<MAX_MESSAGE_LEN>,
    telemetry: VecDeque<TelemetryFrame>,
    events: VecDeque<OwnedEvent>,
    /// Telemetry channel catalog cache (id → descriptor).
    channels: HashMap<u16, OwnedChannelDesc>,
    /// Active stream layouts (stream_id → channel ids in pack order).
    layouts: HashMap<u8, Vec<u16>>,
}

impl RdpClient<tokio::net::TcpStream> {
    /// Connect to a device over TCP.
    pub async fn connect(host: &str, port: u16) -> Result<Self, RdpError> {
        let stream = tokio::net::TcpStream::connect((host, port)).await?;
        let _ = stream.set_nodelay(true);
        Ok(Self::new(stream))
    }
}

impl<S: AsyncRead + AsyncWrite + Unpin> RdpClient<S> {
    /// Wrap an existing async byte stream.
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            next_seq: 1,
            rx: Vec::new(),
            defrag: Defragmenter::new(),
            telemetry: VecDeque::new(),
            events: VecDeque::new(),
            channels: HashMap::new(),
            layouts: HashMap::new(),
        }
    }

    // ── Low-level send/receive ───────────────────────────────────────────────

    /// Send a complete message payload, returning the frame `seq` used.
    async fn send_payload(&mut self, payload: &[u8]) -> Result<u16, RdpError> {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if self.next_seq == 0 {
            self.next_seq = 1;
        }
        // Worst case: one frame per 512-byte chunk, each with header/CRC and
        // COBS expansion plus the delimiter.
        let frames = payload.len().div_ceil(512).max(1);
        let mut out = vec![0u8; frames * 600 + 16];
        let n = encode_message(Flags::none(), seq, payload, &mut out).map_err(RdpError::Frame)?;
        self.stream.write_all(&out[..n]).await?;
        Ok(seq)
    }

    /// Pop the next complete (defragmented) message from the receive buffer,
    /// if one is available. Returns `(frame_seq, message_payload)`.
    fn pop_message(&mut self) -> Option<(u16, Vec<u8>)> {
        while let Some(pos) = self.rx.iter().position(|&b| b == 0) {
            let chunk: Vec<u8> = self.rx.drain(..=pos).collect();
            let frame = &chunk[..chunk.len().saturating_sub(1)];
            if frame.is_empty() {
                continue; // stray delimiter — resynchronise
            }
            let mut scratch = [0u8; MAX_RAW_FRAME_LEN];
            let Ok((header, payload)) = decode_frame(frame, &mut scratch) else {
                continue; // corrupt frame — COBS is self-synchronising, skip it
            };
            match self.defrag.feed(&header, payload) {
                Ok(Some(complete)) => {
                    let msg = complete.to_vec();
                    return Some((header.seq, msg));
                }
                Ok(None) | Err(_) => continue,
            }
        }
        None
    }

    /// Receive the next complete message, reading from the stream as needed.
    async fn recv_message(&mut self) -> Result<(u16, Vec<u8>), RdpError> {
        loop {
            if let Some(msg) = self.pop_message() {
                return Ok(msg);
            }
            let mut buf = [0u8; 4096];
            let n = self.stream.read(&mut buf).await?;
            if n == 0 {
                return Err(RdpError::ConnectionClosed);
            }
            self.rx.extend_from_slice(&buf[..n]);
        }
    }

    /// Send a request payload and await the correlated response body.
    async fn round_trip(&mut self, opcode: u16, payload: &[u8]) -> Result<Vec<u8>, RdpError> {
        let seq = self.send_payload(payload).await?;
        loop {
            let (frame_seq, msg) = self.recv_message().await?;
            let Ok((header, body)) = read_message_header(&msg) else {
                continue;
            };
            match header.kind {
                Kind::Response => {
                    if frame_seq != seq {
                        // Stale/mismatched response — tolerate by skipping.
                        continue;
                    }
                    if header.op != opcode {
                        return Err(RdpError::UnexpectedOp {
                            expected: opcode,
                            got: header.op,
                        });
                    }
                    return Ok(body.to_vec());
                }
                Kind::Telemetry => self.queue_telemetry(body),
                Kind::Event => self.queue_event(body),
                Kind::Request => {}
            }
        }
    }

    /// Issue a request with a CBOR body and return the response body.
    async fn request<T: rusefi_device_api::minicbor::Encode<()>>(
        &mut self,
        opcode: u16,
        body: &T,
    ) -> Result<Vec<u8>, RdpError> {
        let mut buf = [0u8; MAX_MESSAGE_LEN];
        let n = write_message_header(Kind::Request, opcode, &mut buf)
            .map_err(|_| RdpError::Encode { op: opcode })?;
        let body_len =
            encode_to_slice(body, &mut buf[n..]).ok_or(RdpError::Encode { op: opcode })?;
        let payload = buf[..n + body_len].to_vec();
        self.round_trip(opcode, &payload).await
    }

    /// Issue a body-less request and return the response body.
    async fn request_empty(&mut self, opcode: u16) -> Result<Vec<u8>, RdpError> {
        let mut buf = [0u8; MESSAGE_HEADER_LEN];
        let n = write_message_header(Kind::Request, opcode, &mut buf)
            .map_err(|_| RdpError::Encode { op: opcode })?;
        let payload = buf[..n].to_vec();
        self.round_trip(opcode, &payload).await
    }

    // ── Push handling ────────────────────────────────────────────────────────

    fn queue_telemetry(&mut self, body: &[u8]) {
        if body.len() < 7 {
            return;
        }
        let stream_id = body[0];
        let seq = u16::from_le_bytes([body[1], body[2]]);
        let ts_ms = u32::from_le_bytes([body[3], body[4], body[5], body[6]]);
        let data = &body[7..];
        let Some(layout) = self.layouts.get(&stream_id) else {
            return; // unknown stream (e.g. before subscribe completed) — drop
        };
        let mut values = vec![0.0f32; layout.len()];
        let mut pos = 0usize;
        let mut bit_slots: Vec<usize> = Vec::new();
        for (i, ch) in layout.iter().enumerate() {
            let Some(desc) = self.channels.get(ch) else {
                return;
            };
            match desc.vtype {
                ValueType::U16 => {
                    let Some(raw) = data.get(pos..pos + 2) else { return };
                    values[i] = f32::from(u16::from_le_bytes([raw[0], raw[1]])) * desc.scale;
                    pos += 2;
                }
                ValueType::I16 => {
                    let Some(raw) = data.get(pos..pos + 2) else { return };
                    values[i] = f32::from(i16::from_le_bytes([raw[0], raw[1]])) * desc.scale;
                    pos += 2;
                }
                ValueType::Bool => bit_slots.push(i),
                _ => return, // unsupported wire type
            }
        }
        for (bit_no, &slot) in bit_slots.iter().enumerate() {
            let Some(&byte) = data.get(pos + bit_no / 8) else {
                return;
            };
            values[slot] = if (byte >> (bit_no % 8)) & 1 == 1 { 1.0 } else { 0.0 };
        }
        self.telemetry.push_back(TelemetryFrame {
            stream_id,
            seq,
            ts_ms,
            values,
        });
    }

    fn queue_event(&mut self, body: &[u8]) {
        if let Some(e) = decode_from_slice::<bodies::EventBody>(body) {
            self.events.push_back(OwnedEvent {
                kind: e.kind,
                ts_ms: e.ts_ms,
                a: e.a,
                b: e.b,
            });
        }
    }

    /// Pop a buffered telemetry frame, if any (non-blocking).
    pub fn try_next_telemetry(&mut self) -> Option<TelemetryFrame> {
        self.telemetry.pop_front()
    }

    /// Pop a buffered event, if any (non-blocking).
    pub fn try_next_event(&mut self) -> Option<OwnedEvent> {
        self.events.pop_front()
    }

    /// Await the next pushed message (telemetry frame or event).
    pub async fn next_push(&mut self) -> Result<Push, RdpError> {
        loop {
            if let Some(f) = self.telemetry.pop_front() {
                return Ok(Push::Telemetry(f));
            }
            if let Some(e) = self.events.pop_front() {
                return Ok(Push::Event(e));
            }
            let (_seq, msg) = self.recv_message().await?;
            if let Ok((header, body)) = read_message_header(&msg) {
                match header.kind {
                    Kind::Telemetry => self.queue_telemetry(body),
                    Kind::Event => self.queue_event(body),
                    // Stale response with no in-flight request — drop it.
                    Kind::Response | Kind::Request => {}
                }
            }
        }
    }

    // ── System ───────────────────────────────────────────────────────────────

    /// `Hello` — identify the device.
    pub async fn hello(&mut self) -> Result<OwnedHelloInfo, RdpError> {
        let body = self.request_empty(op::HELLO).await?;
        let info: bodies::HelloInfo<'_> = decode_data(&body, op::HELLO)?;
        Ok(OwnedHelloInfo {
            proto_major: info.proto_major,
            proto_minor: info.proto_minor,
            fw_version: info.fw_version.to_owned(),
            board: info.board,
            mcu: info.mcu.to_owned(),
            cylinders: info.cylinders,
            capabilities: info.capabilities,
            schema_hash: info.schema_hash,
            max_payload: info.max_payload,
            device_id: info.device_id.to_vec(),
        })
    }

    /// `Ping` — liveness probe. Returns `(nonce, uptime_ms)`.
    pub async fn ping(&mut self, nonce: u32) -> Result<(u32, u32), RdpError> {
        let body = self
            .request(op::PING, &bodies::PingRequest { nonce })
            .await?;
        let resp: bodies::PingResponse = decode_data(&body, op::PING)?;
        Ok((resp.nonce, resp.uptime_ms))
    }

    // ── Descriptor ───────────────────────────────────────────────────────────

    /// `GetSchemaInfo` — schema hash, catalog sizes and UI categories.
    pub async fn schema_info(&mut self) -> Result<OwnedSchemaInfo, RdpError> {
        let body = self.request_empty(op::GET_SCHEMA_INFO).await?;
        let resp: bodies::GetSchemaInfoResponse<'_> = decode_data(&body, op::GET_SCHEMA_INFO)?;
        Ok(OwnedSchemaInfo {
            schema_hash: resp.schema_hash,
            param_count: resp.param_count,
            table_count: resp.table_count,
            categories: resp
                .categories
                .iter()
                .map(|c| OwnedCategory {
                    id: c.id,
                    name: c.name.to_owned(),
                })
                .collect(),
        })
    }

    /// `GetParamCatalog` — fetch all parameter descriptors (loops pages).
    pub async fn param_catalog(&mut self) -> Result<Vec<OwnedParamDesc>, RdpError> {
        let mut items = Vec::new();
        let mut page = 0u16;
        loop {
            let body = self
                .request(op::GET_PARAM_CATALOG, &bodies::GetCatalogRequest { page })
                .await?;
            let resp: bodies::GetParamCatalogResponse<'_> =
                decode_data(&body, op::GET_PARAM_CATALOG)?;
            for d in resp.items.iter() {
                items.push(OwnedParamDesc {
                    id: d.id,
                    key: d.key.to_owned(),
                    label: d.label.to_owned(),
                    category: d.category,
                    vtype: d.vtype,
                    unit: d.unit.to_owned(),
                    scale: d.scale,
                    offset: d.offset,
                    min: d.min,
                    max: d.max,
                    default: d.default,
                    digits: d.digits,
                    flags: d.flags,
                    enum_labels: d
                        .enum_labels
                        .as_ref()
                        .map(|ls| ls.iter().map(|s| (*s).to_owned()).collect()),
                });
            }
            page += 1;
            if page >= resp.total_pages {
                break;
            }
        }
        Ok(items)
    }

    /// `GetTableCatalog` — fetch all table descriptors (loops pages).
    pub async fn table_catalog(&mut self) -> Result<Vec<OwnedTableDesc>, RdpError> {
        let mut items = Vec::new();
        let mut page = 0u16;
        loop {
            let body = self
                .request(op::GET_TABLE_CATALOG, &bodies::GetCatalogRequest { page })
                .await?;
            let resp: bodies::GetTableCatalogResponse<'_> =
                decode_data(&body, op::GET_TABLE_CATALOG)?;
            for d in resp.items.iter() {
                items.push(OwnedTableDesc {
                    id: d.id,
                    key: d.key.to_owned(),
                    label: d.label.to_owned(),
                    category: d.category,
                    dims: d.dims,
                    x_size: d.x_size,
                    y_size: d.y_size,
                    x_axis_key: d.x_axis_key.to_owned(),
                    y_axis_key: d.y_axis_key.to_owned(),
                    x_unit: d.x_unit.to_owned(),
                    y_unit: d.y_unit.to_owned(),
                    cell_unit: d.cell_unit.to_owned(),
                    cell_min: d.cell_min,
                    cell_max: d.cell_max,
                    cell_digits: d.cell_digits,
                });
            }
            page += 1;
            if page >= resp.total_pages {
                break;
            }
        }
        Ok(items)
    }

    /// `GetTelemetryCatalog` — fetch all channel descriptors (loops pages).
    ///
    /// The catalog is also cached internally so pushed telemetry frames can be
    /// decoded into physical values.
    pub async fn telemetry_catalog(&mut self) -> Result<Vec<OwnedChannelDesc>, RdpError> {
        let mut items = Vec::new();
        let mut page = 0u16;
        loop {
            let body = self
                .request(
                    op::GET_TELEMETRY_CATALOG,
                    &bodies::GetCatalogRequest { page },
                )
                .await?;
            let resp: bodies::GetTelemetryCatalogResponse<'_> =
                decode_data(&body, op::GET_TELEMETRY_CATALOG)?;
            for d in resp.items.iter() {
                items.push(OwnedChannelDesc {
                    id: d.id,
                    key: d.key.to_owned(),
                    label: d.label.to_owned(),
                    category: d.category,
                    vtype: d.vtype,
                    unit: d.unit.to_owned(),
                    scale: d.scale,
                    offset: d.offset,
                    digits: d.digits,
                });
            }
            page += 1;
            if page >= resp.total_pages {
                break;
            }
        }
        self.channels = items.iter().map(|c| (c.id, c.clone())).collect();
        Ok(items)
    }

    // ── Config ───────────────────────────────────────────────────────────────

    /// `ParamGet` — read parameter values (physical f32) by id.
    pub async fn param_get(&mut self, ids: &[u16]) -> Result<Vec<f32>, RdpError> {
        let ids_vec: heapless::Vec<u16, 32> =
            heapless::Vec::from_slice(ids).map_err(|_| RdpError::TooManyItems(ids.len()))?;
        let body = self
            .request(op::PARAM_GET, &bodies::ParamGetRequest { ids: ids_vec })
            .await?;
        let resp: bodies::ParamGetResponse<'_> = decode_data(&body, op::PARAM_GET)?;
        let mut values = Vec::with_capacity(resp.values.len());
        for v in resp.values.iter() {
            let raw: [u8; 4] = v
                .raw
                .try_into()
                .map_err(|_| RdpError::Decode { op: op::PARAM_GET })?;
            if v.vtype != ValueType::F32 {
                return Err(RdpError::Decode { op: op::PARAM_GET });
            }
            values.push(f32::from_le_bytes(raw));
        }
        Ok(values)
    }

    /// `ParamSet` — write one parameter (physical f32). Returns the per-entry
    /// result code (`Ok`, `OutOfRange`, `ReadOnly`, `Busy`, `NotFound`, …).
    pub async fn param_set(&mut self, id: u16, value: f32) -> Result<ErrorCode, RdpError> {
        let raw = value.to_le_bytes();
        let mut entries: heapless::Vec<bodies::ParamSetEntry<'_>, 32> = heapless::Vec::new();
        entries
            .push(bodies::ParamSetEntry {
                id,
                value: bodies::ParamValue {
                    vtype: ValueType::F32,
                    raw: &raw,
                },
            })
            .map_err(|_| RdpError::TooManyItems(1))?;
        let body = self
            .request(op::PARAM_SET, &bodies::ParamSetRequest { entries })
            .await?;
        let resp: bodies::ParamSetResponse = decode_data(&body, op::PARAM_SET)?;
        match resp.results.first() {
            Some(r) => Ok(r.code),
            None => Err(RdpError::Decode { op: op::PARAM_SET }),
        }
    }

    /// `TableGet` — read a full table (axes + row-major cells).
    pub async fn table_get(&mut self, table_id: u16) -> Result<OwnedTableData, RdpError> {
        let body = self
            .request(op::TABLE_GET, &bodies::TableGetRequest { table_id })
            .await?;
        let resp: bodies::TableGetResponse = decode_data(&body, op::TABLE_GET)?;
        Ok(OwnedTableData {
            x_axis: resp.x_axis.to_vec(),
            y_axis: resp.y_axis.to_vec(),
            cells: resp.cells.to_vec(),
        })
    }

    /// `TableSetCell` — write one table cell (effective immediately in RAM).
    pub async fn table_set_cell(
        &mut self,
        table_id: u16,
        ix: u16,
        iy: u16,
        value: f32,
    ) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(
                op::TABLE_SET_CELL,
                &bodies::TableSetCellRequest {
                    table_id,
                    ix,
                    iy,
                    value,
                },
            )
            .await?;
        decode_status(&body, op::TABLE_SET_CELL)
    }

    /// `TableSetAxis` — replace a table axis (`axis`: 0=X, 1=Y).
    pub async fn table_set_axis(
        &mut self,
        table_id: u16,
        axis: u8,
        values: &[f32],
    ) -> Result<ErrorCode, RdpError> {
        let vals: heapless::Vec<f32, 16> =
            heapless::Vec::from_slice(values).map_err(|_| RdpError::TooManyItems(values.len()))?;
        let body = self
            .request(
                op::TABLE_SET_AXIS,
                &bodies::TableSetAxisRequest {
                    table_id,
                    axis,
                    values: vals,
                },
            )
            .await?;
        decode_status(&body, op::TABLE_SET_AXIS)
    }

    /// `ConfigSave` — persist staged RAM config to flash.
    /// Returns `(saved_bytes, crc)`.
    pub async fn config_save(&mut self) -> Result<(u32, u32), RdpError> {
        let body = self.request_empty(op::CONFIG_SAVE).await?;
        let resp: bodies::ConfigSaveResponse = decode_data(&body, op::CONFIG_SAVE)?;
        Ok((resp.saved_bytes, resp.crc))
    }

    /// `ConfigDiscard` — drop staged RAM edits (revert to flash).
    pub async fn config_discard(&mut self) -> Result<ErrorCode, RdpError> {
        let body = self.request_empty(op::CONFIG_DISCARD).await?;
        decode_status(&body, op::CONFIG_DISCARD)
    }

    /// `ConfigResetDefaults` — fill RAM config with build defaults
    /// (sends the required confirmation magic).
    pub async fn config_reset_defaults(&mut self) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(
                op::CONFIG_RESET_DEFAULTS,
                &bodies::ConfigResetDefaultsRequest {
                    confirm: RESET_DEFAULTS_CONFIRM,
                },
            )
            .await?;
        decode_status(&body, op::CONFIG_RESET_DEFAULTS)
    }

    /// `ConfigStatus` — `(dirty, ram_crc, flash_crc)`.
    pub async fn config_status(&mut self) -> Result<(bool, u32, u32), RdpError> {
        let body = self.request_empty(op::CONFIG_STATUS).await?;
        let resp: bodies::ConfigStatusResponse = decode_data(&body, op::CONFIG_STATUS)?;
        Ok((resp.dirty, resp.ram_crc, resp.flash_crc))
    }

    // ── Telemetry ────────────────────────────────────────────────────────────

    /// `Subscribe` — start a telemetry stream.
    /// Returns `(stream_id, layout, actual_rate_hz)`.
    ///
    /// The channel catalog is fetched (and cached) automatically when needed so
    /// that pushed frames can be decoded into physical values.
    pub async fn subscribe(
        &mut self,
        channels: &[u16],
        rate_hz: u16,
    ) -> Result<(u8, Vec<u16>, u16), RdpError> {
        if self.channels.is_empty() {
            let _ = self.telemetry_catalog().await?;
        }
        let chans: heapless::Vec<u16, 32> = heapless::Vec::from_slice(channels)
            .map_err(|_| RdpError::TooManyItems(channels.len()))?;
        let body = self
            .request(
                op::TELEM_SUBSCRIBE,
                &bodies::SubscribeRequest {
                    channels: chans,
                    rate_hz,
                },
            )
            .await?;
        let resp: bodies::SubscribeResponse = decode_data(&body, op::TELEM_SUBSCRIBE)?;
        let layout: Vec<u16> = resp.layout.to_vec();
        self.layouts.insert(resp.stream_id, layout.clone());
        Ok((resp.stream_id, layout, resp.rate_hz))
    }

    /// `Unsubscribe` — stop a telemetry stream.
    pub async fn unsubscribe(&mut self, stream_id: u8) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(op::TELEM_UNSUBSCRIBE, &bodies::UnsubscribeRequest { stream_id })
            .await?;
        self.layouts.remove(&stream_id);
        decode_status(&body, op::TELEM_UNSUBSCRIBE)
    }

    /// `ReadOnce` — one-shot read of channel values (physical units).
    pub async fn read_once(&mut self, channels: &[u16]) -> Result<Vec<f32>, RdpError> {
        let chans: heapless::Vec<u16, 32> = heapless::Vec::from_slice(channels)
            .map_err(|_| RdpError::TooManyItems(channels.len()))?;
        let body = self
            .request(op::TELEM_READ_ONCE, &bodies::ReadOnceRequest { channels: chans })
            .await?;
        let resp: bodies::ReadOnceResponse = decode_data(&body, op::TELEM_READ_ONCE)?;
        Ok(resp.values.to_vec())
    }

    // ── Control ──────────────────────────────────────────────────────────────

    /// `BenchTest` — pulse an actuator (engine must be stopped).
    pub async fn bench_test(
        &mut self,
        target: u8,
        index: u8,
        on_ms: u16,
        off_ms: u16,
        count: u16,
    ) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(
                op::BENCH_TEST,
                &bodies::BenchTestRequest {
                    target,
                    index,
                    on_ms,
                    off_ms,
                    count,
                },
            )
            .await?;
        decode_status(&body, op::BENCH_TEST)
    }

    /// `SetOverride` — temporarily override a control output (fail-safe).
    pub async fn set_override(
        &mut self,
        target: u8,
        value: f32,
        timeout_ms: u16,
    ) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(
                op::SET_OVERRIDE,
                &bodies::SetOverrideRequest {
                    target,
                    value,
                    timeout_ms,
                },
            )
            .await?;
        decode_status(&body, op::SET_OVERRIDE)
    }

    /// `ClearOverride` — remove a control override.
    pub async fn clear_override(&mut self, target: u8) -> Result<ErrorCode, RdpError> {
        let body = self
            .request(op::CLEAR_OVERRIDE, &bodies::ClearOverrideRequest { target })
            .await?;
        decode_status(&body, op::CLEAR_OVERRIDE)
    }

    /// `Calibrate` — run a calibration routine, returning its results.
    pub async fn calibrate(&mut self, routine: u8, args: &[f32]) -> Result<Vec<f32>, RdpError> {
        let a: heapless::Vec<f32, 8> =
            heapless::Vec::from_slice(args).map_err(|_| RdpError::TooManyItems(args.len()))?;
        let body = self
            .request(op::CALIBRATE, &bodies::CalibrateRequest { routine, args: a })
            .await?;
        let resp: bodies::CalibrateResponse = decode_data(&body, op::CALIBRATE)?;
        Ok(resp.result.to_vec())
    }

    // ── Diagnostics ──────────────────────────────────────────────────────────

    /// `GetFaults` — list stored faults (DTCs).
    pub async fn get_faults(&mut self) -> Result<Vec<OwnedFault>, RdpError> {
        let body = self.request_empty(op::GET_FAULTS).await?;
        let resp: bodies::GetFaultsResponse = decode_data(&body, op::GET_FAULTS)?;
        Ok(resp
            .faults
            .iter()
            .map(|f| OwnedFault {
                code: f.code,
                severity: f.severity,
                active: f.active,
                count: f.count,
                first_ts_ms: f.first_ts_ms,
                last_ts_ms: f.last_ts_ms,
                detail: f.detail,
            })
            .collect())
    }

    /// `ClearFaults` — clear faults by bitmask. Returns the cleared count.
    pub async fn clear_faults(&mut self, mask: u32) -> Result<u16, RdpError> {
        let body = self
            .request(op::CLEAR_FAULTS, &bodies::ClearFaultsRequest { mask })
            .await?;
        let resp: bodies::ClearFaultsResponse = decode_data(&body, op::CLEAR_FAULTS)?;
        Ok(resp.cleared)
    }
}

// ─── Response body decoding helpers ──────────────────────────────────────────

/// Decode a data-carrying response body. A failed response carries a CBOR
/// `ErrorResponseBody` instead, which is surfaced as [`RdpError::Device`].
fn decode_data<'b, T>(body: &'b [u8], opcode: u16) -> Result<T, RdpError>
where
    T: rusefi_device_api::minicbor::Decode<'b, ()>,
{
    if let Some(v) = decode_from_slice::<T>(body) {
        return Ok(v);
    }
    match decode_from_slice::<bodies::ErrorResponseBody<'_>>(body) {
        Some(e) => Err(RdpError::Device {
            code: e.code,
            detail: e.detail,
        }),
        None => Err(RdpError::Decode { op: opcode }),
    }
}

/// Decode a status-only response (`ErrorResponseBody`), returning its code.
fn decode_status(body: &[u8], opcode: u16) -> Result<ErrorCode, RdpError> {
    decode_from_slice::<bodies::ErrorResponseBody<'_>>(body)
        .map(|e| e.code)
        .ok_or(RdpError::Decode { op: opcode })
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use rusefi_core::comms::{DeviceIdentity, OutputChannels, RdpContext, RdpServer};
    use rusefi_core::config::EngineConfig;
    use std::time::{Duration, Instant};
    use tokio::io::DuplexStream;

    /// CrankingRpm parameter id (0x0120).
    const CRANKING_RPM_ID: u16 = 288;
    /// Ignition table id.
    const IGNITION_TABLE_ID: u16 = 1;

    /// Run an in-process RDP device (mirrors the sim serve loop) until the
    /// peer disconnects.
    async fn device_loop(mut io: DuplexStream, rpm: f32) {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        let defaults = EngineConfig::default_4cyl();
        let mut ram = defaults.clone();
        let mut flash = defaults.clone();
        let mut server = RdpServer::new(DeviceIdentity::sim(4), &flash);
        let mut defrag = Defragmenter::<4096>::new();
        let mut rx: Vec<u8> = Vec::new();
        let mut outputs = OutputChannels::zeroed();
        outputs.rpm = rpm;
        outputs.clt_c = 80.0;
        outputs.battery_v = 14.0;
        outputs.lambda = 1.0;
        let start = Instant::now();
        let mut push_seq = 0u16;

        loop {
            let mut buf = [0u8; 4096];
            let n = match tokio::time::timeout(Duration::from_millis(5), io.read(&mut buf)).await
            {
                Ok(Ok(0)) => break,
                Ok(Ok(n)) => n,
                Ok(Err(_)) => break,
                Err(_) => 0,
            };
            rx.extend_from_slice(&buf[..n]);

            while let Some(pos) = rx.iter().position(|&b| b == 0) {
                let chunk: Vec<u8> = rx.drain(..=pos).collect();
                let frame = &chunk[..chunk.len() - 1];
                if frame.is_empty() {
                    continue;
                }
                let mut scratch = [0u8; MAX_RAW_FRAME_LEN];
                let Ok((header, payload)) = decode_frame(frame, &mut scratch) else {
                    continue;
                };
                let complete = match defrag.feed(&header, payload) {
                    Ok(Some(p)) => p.to_vec(),
                    _ => continue,
                };
                let now_ms = start.elapsed().as_millis() as u32;
                let mut resp = vec![0u8; 4096];
                let (len, actions) = {
                    let mut ctx = RdpContext {
                        ram: &mut ram,
                        flash: &flash,
                        defaults: &defaults,
                        outputs: &outputs,
                        now_ms,
                        engine_running: outputs.rpm > 100.0,
                    };
                    server.handle(&complete, &mut ctx, &mut resp)
                };
                if actions.save {
                    flash = ram.clone();
                }
                if len > 0 {
                    let mut enc = vec![0u8; len.div_ceil(512).max(1) * 600 + 16];
                    if let Ok(en) =
                        encode_message(Flags::none(), header.seq, &resp[..len], &mut enc)
                    {
                        if io.write_all(&enc[..en]).await.is_err() {
                            return;
                        }
                    }
                }
            }

            // Telemetry / event pushes.
            let now_ms = start.elapsed().as_millis() as u32;
            let mut push = [0u8; 256];
            while let Some(plen) = server.poll_telemetry(&outputs, now_ms, &mut push) {
                let mut enc = [0u8; 1024];
                if let Ok(en) = encode_message(Flags::none(), push_seq, &push[..plen], &mut enc) {
                    push_seq = push_seq.wrapping_add(1);
                    if io.write_all(&enc[..en]).await.is_err() {
                        return;
                    }
                }
            }
            while let Some(plen) = server.poll_event(&mut push) {
                let mut enc = [0u8; 1024];
                if let Ok(en) = encode_message(Flags::none(), push_seq, &push[..plen], &mut enc) {
                    push_seq = push_seq.wrapping_add(1);
                    if io.write_all(&enc[..en]).await.is_err() {
                        return;
                    }
                }
            }
        }
        server.on_disconnect();
    }

    fn connect_pair(rpm: f32) -> RdpClient<DuplexStream> {
        let (a, b) = tokio::io::duplex(65536);
        tokio::spawn(device_loop(b, rpm));
        RdpClient::new(a)
    }

    #[tokio::test]
    async fn hello_reports_protocol() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let hello = client.hello().await?;
        assert_eq!(hello.proto_major, 1);
        assert_eq!(hello.cylinders, 4);
        assert_eq!(hello.mcu, "x86-sim");
        assert_eq!(hello.device_id.len(), 12);
        Ok(())
    }

    #[tokio::test]
    async fn ping_round_trip() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let (nonce, _uptime) = client.ping(0xDEAD_BEEF).await?;
        assert_eq!(nonce, 0xDEAD_BEEF);
        Ok(())
    }

    #[tokio::test]
    async fn param_set_get_round_trip() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let code = client.param_set(CRANKING_RPM_ID, 350.0).await?;
        assert_eq!(code, ErrorCode::Ok);
        let values = client.param_get(&[CRANKING_RPM_ID]).await?;
        assert_eq!(values, vec![350.0]);
        Ok(())
    }

    #[tokio::test]
    async fn param_get_unknown_id_is_device_error() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let result = client.param_get(&[0xFFFE]).await;
        assert!(matches!(
            result,
            Err(RdpError::Device {
                code: ErrorCode::NotFound,
                ..
            })
        ));
        Ok(())
    }

    #[tokio::test]
    async fn table_get_dimensions_via_fragmentation() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        // 16x16 table response exceeds the 512-byte frame payload limit, so
        // this exercises fragment reassembly.
        let table = client.table_get(IGNITION_TABLE_ID).await?;
        assert_eq!(table.x_axis.len(), 16);
        assert_eq!(table.y_axis.len(), 16);
        assert_eq!(table.cells.len(), 256);
        Ok(())
    }

    #[tokio::test]
    async fn catalogs_are_complete() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let schema = client.schema_info().await?;
        let params = client.param_catalog().await?;
        let tables = client.table_catalog().await?;
        let channels = client.telemetry_catalog().await?;
        assert_eq!(params.len(), schema.param_count as usize);
        assert_eq!(tables.len(), schema.table_count as usize);
        assert!(!schema.categories.is_empty());
        assert!(channels.iter().any(|c| c.key == "rpm"));
        Ok(())
    }

    #[tokio::test]
    async fn config_status_dirty_flow() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);

        let (dirty, ram_crc, flash_crc) = client.config_status().await?;
        assert!(!dirty);
        assert_eq!(ram_crc, flash_crc);

        let code = client.param_set(CRANKING_RPM_ID, 350.0).await?;
        assert_eq!(code, ErrorCode::Ok);
        let (dirty, ram_crc, flash_crc) = client.config_status().await?;
        assert!(dirty);
        assert_ne!(ram_crc, flash_crc);

        let (saved_bytes, crc) = client.config_save().await?;
        assert!(saved_bytes > 0);
        let (dirty, ram_crc, flash_crc) = client.config_status().await?;
        assert!(!dirty);
        assert_eq!(ram_crc, flash_crc);
        assert_eq!(crc, flash_crc);
        Ok(())
    }

    #[tokio::test]
    async fn subscribe_and_receive_telemetry() -> Result<(), RdpError> {
        let mut client = connect_pair(3000.0);
        let (stream_id, layout, rate) = client.subscribe(&[1, 2, 7], 50).await?;
        assert_eq!(layout, vec![1, 2, 7]);
        assert_eq!(rate, 50);

        // Wait for the first frame of our stream.
        let frame = loop {
            match client.next_push().await? {
                Push::Telemetry(f) if f.stream_id == stream_id => break f,
                _ => continue,
            }
        };
        assert_eq!(frame.values.len(), 3);
        // rpm is plausible (the device snapshot is exactly 3000)
        assert!((frame.values[0] - 3000.0).abs() < 1.0, "rpm = {}", frame.values[0]);
        // clt ~80, lambda ~1.0
        assert!((frame.values[1] - 80.0).abs() < 0.5);
        assert!((frame.values[2] - 1.0).abs() < 0.01);

        let code = client.unsubscribe(stream_id).await?;
        assert_eq!(code, ErrorCode::Ok);
        Ok(())
    }

    #[tokio::test]
    async fn get_faults_empty() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let faults = client.get_faults().await?;
        assert!(faults.is_empty());
        Ok(())
    }

    #[tokio::test]
    async fn bench_test_allowed_when_stopped_busy_when_running() -> Result<(), RdpError> {
        let mut stopped = connect_pair(0.0);
        let code = stopped.bench_test(0, 1, 3, 100, 5).await?;
        assert_eq!(code, ErrorCode::Ok);

        let mut running = connect_pair(3000.0);
        let code = running.bench_test(0, 1, 3, 100, 5).await?;
        assert_eq!(code, ErrorCode::Busy);
        Ok(())
    }

    #[tokio::test]
    async fn override_set_and_clear() -> Result<(), RdpError> {
        let mut client = connect_pair(0.0);
        let code = client.set_override(2, 15.0, 5000).await?;
        assert_eq!(code, ErrorCode::Ok);
        let code = client.clear_override(2).await?;
        assert_eq!(code, ErrorCode::Ok);
        Ok(())
    }

    #[tokio::test]
    async fn read_once_returns_values() -> Result<(), RdpError> {
        let mut client = connect_pair(3000.0);
        let values = client.read_once(&[1, 6]).await?;
        assert_eq!(values.len(), 2);
        assert!((values[0] - 3000.0).abs() < 0.1);
        assert!((values[1] - 14.0).abs() < 0.1);
        Ok(())
    }
}
