//! RDP serve mode: expose the device-side RDP server over a local TCP port.
//!
//! Drives a synthetic engine model and lets host tools (`rusefi rdp ...`,
//! `rusefi-client`) exercise the full RustEMS Device Protocol against a
//! persistent in-memory ECU: RAM/flash/defaults config staging, telemetry
//! streams, control overrides, faults and events.

use anyhow::{Context, Result};
use rusefi_core::comms::control::OverrideTarget;
use rusefi_core::comms::{DeviceIdentity, OutputChannels, RdpContext, RdpServer};
use rusefi_core::config::EngineConfig;
use rusefi_device_api::frame::{decode_frame, encode_message, Flags, MAX_RAW_FRAME_LEN};
use rusefi_device_api::Defragmenter;
use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

/// How often the synthetic engine model is advanced.
const MODEL_STEP_MS: u32 = 10;

/// Run the RDP TCP server on `127.0.0.1:port`, accepting one connection at a
/// time. State (RAM/flash/defaults config and the [`RdpServer`]) persists
/// across connections, like a real ECU staying powered between host sessions.
pub fn run(port: u16, serve_rpm: f32) -> Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))
        .with_context(|| format!("binding 127.0.0.1:{port}"))?;
    println!("RDP serve mode: listening on 127.0.0.1:{port} (engine rpm ~{serve_rpm:.0})");

    let start = Instant::now();
    let defaults = EngineConfig::default_4cyl();
    let mut ram = defaults.clone();
    let mut flash = defaults.clone();
    let mut server = RdpServer::new(DeviceIdentity::sim(4), &flash);

    loop {
        let (stream, peer) = listener.accept().context("accepting connection")?;
        println!("[rdp] client connected: {peer}");
        match serve_connection(
            stream,
            &mut server,
            &mut ram,
            &mut flash,
            &defaults,
            serve_rpm,
            start,
        ) {
            Ok(()) => println!("[rdp] client disconnected: {peer}"),
            Err(e) => println!("[rdp] connection ended with error: {e} ({peer})"),
        }
        // Fail-safe: drop overrides, subscriptions and pending bench tests.
        server.on_disconnect();
    }
}

/// Serve one connected client until EOF or I/O error.
fn serve_connection(
    mut stream: TcpStream,
    server: &mut RdpServer,
    ram: &mut EngineConfig,
    flash: &mut EngineConfig,
    defaults: &EngineConfig,
    serve_rpm: f32,
    start: Instant,
) -> Result<()> {
    stream.set_read_timeout(Some(Duration::from_millis(5)))?;
    let _ = stream.set_nodelay(true);

    let mut defrag = Defragmenter::<4096>::new();
    let mut rx: Vec<u8> = Vec::new();
    let mut outputs = OutputChannels::zeroed();
    let mut next_model_ms: u32 = 0;
    let mut push_seq: u16 = 0;
    let mut read_buf = [0u8; 4096];

    loop {
        // (a) advance the synthetic engine model every MODEL_STEP_MS.
        let now_ms = start.elapsed().as_millis() as u32;
        if now_ms >= next_model_ms {
            update_model(&mut outputs, server, serve_rpm, now_ms);
            next_model_ms = now_ms.wrapping_add(MODEL_STEP_MS);
        }

        // (b) read available bytes (5 ms timeout doubles as the loop tick).
        match stream.read(&mut read_buf) {
            Ok(0) => return Ok(()), // EOF
            Ok(n) => rx.extend_from_slice(&read_buf[..n]),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(e) => return Err(e.into()),
        }

        // Split the receive buffer at 0x00 delimiters and handle each frame.
        while let Some(pos) = rx.iter().position(|&b| b == 0) {
            let chunk: Vec<u8> = rx.drain(..=pos).collect();
            let frame = &chunk[..chunk.len() - 1];
            if frame.is_empty() {
                continue; // stray delimiter — resynchronise
            }
            let mut scratch = [0u8; MAX_RAW_FRAME_LEN];
            let Ok((header, payload)) = decode_frame(frame, &mut scratch) else {
                continue; // corrupt frame — COBS is self-synchronising
            };
            let request = match defrag.feed(&header, payload) {
                Ok(Some(complete)) => complete.to_vec(),
                Ok(None) => continue,          // more fragments expected
                Err(_) => continue,            // reassembly failure — drop
            };

            let now_ms = start.elapsed().as_millis() as u32;
            let engine_running = outputs.rpm > 100.0;
            let mut resp = vec![0u8; 4096];
            let (len, actions) = {
                let mut ctx = RdpContext {
                    ram: &mut *ram,
                    flash: &*flash,
                    defaults,
                    outputs: &outputs,
                    now_ms,
                    engine_running,
                };
                server.handle(&request, &mut ctx, &mut resp)
            };
            if actions.save {
                *flash = ram.clone();
                println!("[rdp] config saved to flash");
            }
            if actions.reboot {
                println!("[rdp] reboot requested (ignored in sim)");
            }
            if actions.enter_bootloader {
                println!("[rdp] bootloader entry requested (ignored in sim)");
            }
            if len > 0 {
                send_message(&mut stream, header.seq, &resp[..len])?;
            }
            // Bench tests are acknowledged; the sim just logs them.
            if let Some(bench) = server.take_bench_test() {
                println!(
                    "[rdp] bench test: {:?} #{} on={}ms off={}ms count={}",
                    bench.target, bench.index, bench.on_ms, bench.off_ms, bench.count
                );
            }
        }

        // (c) push due telemetry frames and pending events.
        let now_ms = start.elapsed().as_millis() as u32;
        let mut push = [0u8; 256];
        while let Some(n) = server.poll_telemetry(&outputs, now_ms, &mut push) {
            send_message(&mut stream, push_seq, &push[..n])?;
            push_seq = push_seq.wrapping_add(1);
        }
        while let Some(n) = server.poll_event(&mut push) {
            send_message(&mut stream, push_seq, &push[..n])?;
            push_seq = push_seq.wrapping_add(1);
        }
    }
}

/// COBS-encode and send one message payload (fragmenting when needed).
fn send_message(stream: &mut TcpStream, seq: u16, payload: &[u8]) -> Result<()> {
    let frames = payload.len().div_ceil(512).max(1);
    let mut enc = vec![0u8; frames * 600 + 16];
    let n = encode_message(Flags::none(), seq, payload, &mut enc)
        .map_err(|e| anyhow::anyhow!("frame encode failed: {e:?}"))?;
    stream.write_all(&enc[..n])?;
    Ok(())
}

/// Advance the synthetic engine model and refresh the telemetry snapshot.
///
/// - RPM sweeps sinusoidally ±20% around `base_rpm` (0 means engine stopped).
/// - CLT warms from 25 °C to 90 °C over ~60 s of process uptime.
/// - MAP/TPS track the RPM plausibly; lambda ~1.0; battery 14.0 V.
/// - Active control overrides (spark cut / fixed timing / boost duty) are
///   reflected in the snapshot.
fn update_model(
    outputs: &mut OutputChannels,
    server: &mut RdpServer,
    base_rpm: f32,
    now_ms: u32,
) {
    let t = now_ms as f32 / 1000.0;
    let rpm = if base_rpm > 0.0 {
        base_rpm * (1.0 + 0.2 * (t * 0.5).sin())
    } else {
        0.0
    };
    let load = (rpm / 8000.0).clamp(0.0, 1.0);

    outputs.rpm = rpm;
    outputs.clt_c = 25.0 + 65.0 * (t / 60.0).min(1.0);
    outputs.iat_c = 25.0 + 10.0 * load;
    outputs.map_kpa = 30.0 + 70.0 * load;
    outputs.tps_pct = 100.0 * load;
    outputs.lambda = 1.0;
    outputs.battery_v = 14.0;
    outputs.advance_deg = 25.0;
    outputs.inj_pulse_ms = if rpm > 0.0 { 1.5 + 8.0 * load } else { 0.0 };
    outputs.sequential = true;
    outputs.fuel_pump_on = rpm > 0.0;
    outputs.cl_correction = 1.0;
    outputs.ltft_correction = 1.0;
    outputs.iac_duty_pct = 30.0;
    outputs.boost_duty_pct = 20.0;

    // Honor active control overrides (fail-safe expiry handled by Overrides).
    outputs.spark_cut = server
        .overrides
        .get(OverrideTarget::SparkCut, now_ms)
        .is_some_and(|v| v != 0.0);
    if let Some(fix_deg) = server.overrides.get(OverrideTarget::TimingFix, now_ms) {
        outputs.advance_deg = fix_deg;
    }
    if let Some(duty) = server.overrides.get(OverrideTarget::BoostDuty, now_ms) {
        outputs.boost_duty_pct = duty;
    }
}
