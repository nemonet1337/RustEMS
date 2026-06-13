//! CBOR message body codec definitions.

use crate::message::{ErrorCode, ValueType};

// --- Custom codec helper for heapless::Vec ---

pub mod heapless_vec {
    use minicbor::encode::{Encoder, Error as EncodeError, Write};
    use minicbor::decode::{Decoder, Error as DecodeError};

    pub fn encode<T: minicbor::Encode<C>, const N: usize, C, W: Write>(
        v: &heapless::Vec<T, N>,
        e: &mut Encoder<W>,
        ctx: &mut C,
    ) -> Result<(), EncodeError<W::Error>> {
        e.array(v.len() as u64)?;
        for item in v.iter() {
            item.encode(e, ctx)?;
        }
        Ok(())
    }

    pub fn decode<'b, T: minicbor::Decode<'b, C>, const N: usize, C>(
        d: &mut Decoder<'b>,
        ctx: &mut C,
    ) -> Result<heapless::Vec<T, N>, DecodeError> {
        let len = d.array()?.ok_or_else(|| DecodeError::message("expected array for heapless::Vec"))?;
        if len > N as u64 {
            return Err(DecodeError::message("array too large for heapless::Vec"));
        }
        let mut vec = heapless::Vec::new();
        for _ in 0..len {
            vec.push(T::decode(d, ctx)?).map_err(|_| DecodeError::message("failed to push to heapless::Vec"))?;
        }
        Ok(vec)
    }
}

// --- Custom codec helper for Option<heapless::Vec> ---

pub mod heapless_vec_opt {
    use minicbor::encode::{Encoder, Error as EncodeError, Write};
    use minicbor::decode::{Decoder, Error as DecodeError};

    pub fn encode<T: minicbor::Encode<C>, const N: usize, C, W: Write>(
        v: &Option<heapless::Vec<T, N>>,
        e: &mut Encoder<W>,
        ctx: &mut C,
    ) -> Result<(), EncodeError<W::Error>> {
        if let Some(ref vec) = v {
            super::heapless_vec::encode(vec, e, ctx)?;
        } else {
            e.null()?;
        }
        Ok(())
    }

    pub fn decode<'b, T: minicbor::Decode<'b, C>, const N: usize, C>(
        d: &mut Decoder<'b>,
        ctx: &mut C,
    ) -> Result<Option<heapless::Vec<T, N>>, DecodeError> {
        if d.datatype()? == minicbor::data::Type::Null {
            d.null()?;
            Ok(None)
        } else {
            let vec = super::heapless_vec::decode(d, ctx)?;
            Ok(Some(vec))
        }
    }
}

// --- Manual CBOR codecs for externally defined enums ---

impl<C> minicbor::Encode<C> for ErrorCode {
    fn encode<W: minicbor::encode::Write>(
        &self,
        e: &mut minicbor::Encoder<W>,
        _ctx: &mut C,
    ) -> Result<(), minicbor::encode::Error<W::Error>> {
        e.u16(*self as u16)?;
        Ok(())
    }
}

impl<'b, C> minicbor::Decode<'b, C> for ErrorCode {
    fn decode(
        d: &mut minicbor::Decoder<'b>,
        _ctx: &mut C,
    ) -> Result<Self, minicbor::decode::Error> {
        let val = d.u16()?;
        ErrorCode::from_u16(val).ok_or_else(|| minicbor::decode::Error::message("invalid ErrorCode"))
    }
}

// --- Manual CBOR codecs for externally defined enums ---

impl<C> minicbor::Encode<C> for ValueType {
    fn encode<W: minicbor::encode::Write>(
        &self,
        e: &mut minicbor::Encoder<W>,
        _ctx: &mut C,
    ) -> Result<(), minicbor::encode::Error<W::Error>> {
        e.u8(*self as u8)?;
        Ok(())
    }
}

impl<'b, C> minicbor::Decode<'b, C> for ValueType {
    fn decode(
        d: &mut minicbor::Decoder<'b>,
        _ctx: &mut C,
    ) -> Result<Self, minicbor::decode::Error> {
        let val = d.u8()?;
        ValueType::from_u8(val).ok_or_else(|| minicbor::decode::Error::message("invalid ValueType"))
    }
}

// --- A. System Messages ---

/// Device identity and properties.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct HelloInfo<'a> {
    #[n(0)] pub proto_major: u8,
    #[n(1)] pub proto_minor: u8,
    #[b(2)] pub fw_version: &'a str,
    #[n(3)] pub board: u8,
    #[b(4)] pub mcu: &'a str,
    #[n(5)] pub cylinders: u8,
    #[n(6)] pub capabilities: u32,
    #[n(7)] pub schema_hash: u32,
    #[n(8)] pub max_payload: u16,
    #[b(9)] #[cbor(with = "minicbor::bytes")] pub device_id: &'a [u8],
}

/// Liveness keepalive request.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct PingRequest {
    #[n(0)] pub nonce: u32,
}

/// Liveness keepalive response.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct PingResponse {
    #[n(0)] pub nonce: u32,
    #[n(1)] pub uptime_ms: u32,
}

/// Device reboot request.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct RebootRequest {
    #[n(0)] pub mode: u8,
}

/// Request to enter bootloader/DFU.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct EnterBootloaderRequest {
    #[n(0)] pub confirm: u32,
}

// --- B. Descriptor Messages ---

/// UI category group.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct CategoryDesc<'a> {
    #[n(0)] pub id: u8,
    #[b(1)] pub name: &'a str,
}

/// Response containing catalog size metrics.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetSchemaInfoResponse<'a> {
    #[n(0)] pub schema_hash: u32,
    #[n(1)] pub param_count: u16,
    #[n(2)] pub table_count: u16,
    #[b(3)] #[cbor(with = "crate::cbor::heapless_vec")] pub categories: heapless::Vec<CategoryDesc<'a>, 16>,
}

/// Request to get a catalog page.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetCatalogRequest {
    #[n(0)] pub page: u16,
}

/// Single parameter descriptor.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamDesc<'a> {
    #[n(0)] pub id: u16,
    #[b(1)] pub key: &'a str,
    #[b(2)] pub label: &'a str,
    #[n(3)] pub category: u8,
    #[n(4)] pub vtype: ValueType,
    #[b(5)] pub unit: &'a str,
    #[n(6)] pub scale: f32,
    #[n(7)] pub offset: f32,
    #[n(8)] pub min: f32,
    #[n(9)] pub max: f32,
    #[n(10)] pub default: f32,
    #[n(11)] pub digits: u8,
    #[n(12)] pub flags: u8,
    #[b(13)] #[cbor(with = "crate::cbor::heapless_vec_opt")] pub enum_labels: Option<heapless::Vec<&'a str, 16>>,
}

/// Response for a page of parameter descriptors.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetParamCatalogResponse<'a> {
    #[n(0)] pub page: u16,
    #[n(1)] pub total_pages: u16,
    #[b(2)] #[cbor(with = "crate::cbor::heapless_vec")] pub items: heapless::Vec<ParamDesc<'a>, 8>,
}

/// Single table (2D/1D map) descriptor.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct TableDesc<'a> {
    #[n(0)] pub id: u16,
    #[b(1)] pub key: &'a str,
    #[b(2)] pub label: &'a str,
    #[n(3)] pub category: u8,
    #[n(4)] pub dims: u8,
    #[n(5)] pub x_size: u16,
    #[n(6)] pub y_size: u16,
    #[b(7)] pub x_axis_key: &'a str,
    #[b(8)] pub y_axis_key: &'a str,
    #[b(9)] pub x_unit: &'a str,
    #[b(10)] pub y_unit: &'a str,
    #[b(11)] pub cell_unit: &'a str,
    #[n(12)] pub cell_min: f32,
    #[n(13)] pub cell_max: f32,
    #[n(14)] pub cell_digits: u8,
}

/// Response for a page of table descriptors.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetTableCatalogResponse<'a> {
    #[n(0)] pub page: u16,
    #[n(1)] pub total_pages: u16,
    #[b(2)] #[cbor(with = "crate::cbor::heapless_vec")] pub items: heapless::Vec<TableDesc<'a>, 8>,
}

/// Single telemetry channel descriptor.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ChannelDesc<'a> {
    #[n(0)] pub id: u16,
    #[b(1)] pub key: &'a str,
    #[b(2)] pub label: &'a str,
    #[n(3)] pub category: u8,
    #[n(4)] pub vtype: ValueType,
    #[b(5)] pub unit: &'a str,
    #[n(6)] pub scale: f32,
    #[n(7)] pub offset: f32,
    #[n(8)] pub digits: u8,
}

/// Response for a page of telemetry channel descriptors.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetTelemetryCatalogResponse<'a> {
    #[n(0)] pub page: u16,
    #[n(1)] pub total_pages: u16,
    #[b(2)] #[cbor(with = "crate::cbor::heapless_vec")] pub items: heapless::Vec<ChannelDesc<'a>, 8>,
}

// --- C. Config Messages ---

/// Request to read parameters by IDs.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamGetRequest {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub ids: heapless::Vec<u16, 32>,
}

/// Wrapped parameter value.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamValue<'a> {
    #[n(0)] pub vtype: ValueType,
    #[b(1)] #[cbor(with = "minicbor::bytes")] pub raw: &'a [u8],
}

/// Response with parameter values.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamGetResponse<'a> {
    #[b(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub values: heapless::Vec<ParamValue<'a>, 32>,
}

/// Entry to set a parameter value.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamSetEntry<'a> {
    #[n(0)] pub id: u16,
    #[b(1)] pub value: ParamValue<'a>,
}

/// Request to write multiple parameter values.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamSetRequest<'a> {
    #[b(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub entries: heapless::Vec<ParamSetEntry<'a>, 32>,
}

/// Result of a single parameter write.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamSetResult {
    #[n(0)] pub id: u16,
    #[n(1)] pub code: ErrorCode,
}

/// Response for parameter writes.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ParamSetResponse {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub results: heapless::Vec<ParamSetResult, 32>,
}

/// Request a 2D/1D table data.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct TableGetRequest {
    #[n(0)] pub table_id: u16,
}

/// Response containing full table data.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct TableGetResponse {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub x_axis: heapless::Vec<f32, 16>,
    #[n(1)] #[cbor(with = "crate::cbor::heapless_vec")] pub y_axis: heapless::Vec<f32, 16>,
    #[n(2)] #[cbor(with = "crate::cbor::heapless_vec")] pub cells: heapless::Vec<f32, 256>,
}

/// Request to set a single table cell.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct TableSetCellRequest {
    #[n(0)] pub table_id: u16,
    #[n(1)] pub ix: u16,
    #[n(2)] pub iy: u16,
    #[n(3)] pub value: f32,
}

/// Request to set a table axis.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct TableSetAxisRequest {
    #[n(0)] pub table_id: u16,
    #[n(1)] pub axis: u8,
    #[n(2)] #[cbor(with = "crate::cbor::heapless_vec")] pub values: heapless::Vec<f32, 16>,
}

/// Response for configuration save.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ConfigSaveResponse {
    #[n(0)] pub saved_bytes: u32,
    #[n(1)] pub crc: u32,
}

/// Request to reset configuration to default.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ConfigResetDefaultsRequest {
    #[n(0)] pub confirm: u32,
}

/// Response containing configuration status.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ConfigStatusResponse {
    #[n(0)] pub dirty: bool,
    #[n(1)] pub ram_crc: u32,
    #[n(2)] pub flash_crc: u32,
}

// --- D. Telemetry & Events ---

/// Request to subscribe to telemetry channels.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct SubscribeRequest {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub channels: heapless::Vec<u16, 32>,
    #[n(1)] pub rate_hz: u16,
}

/// Response for telemetry subscription request.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct SubscribeResponse {
    #[n(0)] pub stream_id: u8,
    #[n(1)] #[cbor(with = "crate::cbor::heapless_vec")] pub layout: heapless::Vec<u16, 32>,
    /// Actual push rate after clamping to device capability.
    #[n(2)] pub rate_hz: u16,
}

/// Request to unsubscribe from a stream.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct UnsubscribeRequest {
    #[n(0)] pub stream_id: u8,
}

/// Request for a one-shot telemetry read.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ReadOnceRequest {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub channels: heapless::Vec<u16, 32>,
}

/// Response with one-shot telemetry values (physical units, channel order).
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ReadOnceResponse {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub values: heapless::Vec<f32, 32>,
}

// --- E. Control Messages ---

/// Request to bench-test an actuator (engine must be stopped).
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct BenchTestRequest {
    #[n(0)] pub target: u8,
    #[n(1)] pub index: u8,
    #[n(2)] pub on_ms: u16,
    #[n(3)] pub off_ms: u16,
    #[n(4)] pub count: u16,
}

/// Request to set a temporary control override.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct SetOverrideRequest {
    #[n(0)] pub target: u8,
    #[n(1)] pub value: f32,
    #[n(2)] pub timeout_ms: u16,
}

/// Request to clear a control override.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ClearOverrideRequest {
    #[n(0)] pub target: u8,
}

/// Request to run a calibration routine.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct CalibrateRequest {
    #[n(0)] pub routine: u8,
    #[n(1)] #[cbor(with = "crate::cbor::heapless_vec")] pub args: heapless::Vec<f32, 8>,
}

/// Response with calibration routine results.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct CalibrateResponse {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub result: heapless::Vec<f32, 8>,
}

// --- F. Diagnostics Messages ---

/// One structured fault (DTC) entry.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct FaultEntry {
    #[n(0)] pub code: u16,
    #[n(1)] pub severity: u8,
    #[n(2)] pub active: bool,
    #[n(3)] pub count: u16,
    #[n(4)] pub first_ts_ms: u32,
    #[n(5)] pub last_ts_ms: u32,
    #[n(6)] pub detail: u16,
}

/// Response listing stored faults.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct GetFaultsResponse {
    #[n(0)] #[cbor(with = "crate::cbor::heapless_vec")] pub faults: heapless::Vec<FaultEntry, 16>,
}

/// Request to clear faults by bitmask.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ClearFaultsRequest {
    #[n(0)] pub mask: u32,
}

/// Response reporting how many faults were cleared.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ClearFaultsResponse {
    #[n(0)] pub cleared: u16,
}

/// Asynchronous event body (Kind::Event push).
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct EventBody {
    #[n(0)] pub kind: u8,
    #[n(1)] pub ts_ms: u32,
    #[n(2)] pub a: i32,
    #[n(3)] pub b: i32,
}

// --- Buffer-oriented encode/decode helpers ---

/// Encode a CBOR value into `buf`, returning the number of bytes written.
pub fn encode_to_slice<T: minicbor::Encode<()>>(value: &T, buf: &mut [u8]) -> Option<usize> {
    let mut cursor = minicbor::encode::write::Cursor::new(buf);
    minicbor::encode(value, &mut cursor).ok()?;
    Some(cursor.position())
}

/// Decode a CBOR value from `buf`.
pub fn decode_from_slice<'b, T: minicbor::Decode<'b, ()>>(buf: &'b [u8]) -> Option<T> {
    minicbor::decode(buf).ok()
}

/// Generic error response body.
#[derive(Debug, Clone, PartialEq, minicbor::Encode, minicbor::Decode)]
pub struct ErrorResponseBody<'a> {
    #[n(0)] pub code: ErrorCode,
    #[n(1)] pub detail: u16,
    #[b(2)] pub message: Option<&'a str>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hello_info_round_trip() {
        let info = HelloInfo {
            proto_major: 1,
            proto_minor: 0,
            fw_version: "0.1.0-test",
            board: 3,
            mcu: "STM32F407",
            cylinders: 4,
            capabilities: 0b1011,
            schema_hash: 0xDEADBEEF,
            max_payload: 512,
            device_id: &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
        };

        let mut buf = [0u8; 256];
        let mut cursor = minicbor::encode::write::Cursor::new(&mut buf[..]);
        minicbor::encode(&info, &mut cursor).unwrap();
        let size = cursor.position();

        let decoded: HelloInfo = minicbor::decode(&buf[..size]).unwrap();
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_param_desc_round_trip() {
        let mut enum_labels = heapless::Vec::new();
        enum_labels.push("Disabled").unwrap();
        enum_labels.push("Enabled").unwrap();

        let desc = ParamDesc {
            id: 42,
            key: "engine.cylinders",
            label: "Cylinders count",
            category: 0,
            vtype: ValueType::U8,
            unit: "qty",
            scale: 1.0,
            offset: 0.0,
            min: 1.0,
            max: 12.0,
            default: 4.0,
            digits: 0,
            flags: 0b001,
            enum_labels: Some(enum_labels),
        };

        let mut buf = [0u8; 256];
        let mut cursor = minicbor::encode::write::Cursor::new(&mut buf[..]);
        minicbor::encode(&desc, &mut cursor).unwrap();
        let size = cursor.position();

        let decoded: ParamDesc = minicbor::decode(&buf[..size]).unwrap();
        assert_eq!(decoded, desc);
    }

    #[test]
    fn test_error_response_round_trip() {
        let err = ErrorResponseBody {
            code: ErrorCode::OutOfRange,
            detail: 42,
            message: Some("Value is too high"),
        };

        let mut buf = [0u8; 128];
        let mut cursor = minicbor::encode::write::Cursor::new(&mut buf[..]);
        minicbor::encode(&err, &mut cursor).unwrap();
        let size = cursor.position();

        let decoded: ErrorResponseBody = minicbor::decode(&buf[..size]).unwrap();
        assert_eq!(decoded, err);
    }
}
