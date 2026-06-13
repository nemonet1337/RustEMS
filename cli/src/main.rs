//! `rusefi` — command-line interface for interacting with a rusEFI ECU.
//!
//! ## Subcommands
//!
//! - `hello`      — connect, send Hello and print the firmware signature
//! - `read-image` — read the full configuration image and save it to a file
//! - `burn`       — persist ECU RAM configuration to flash
//! - `rdp ...`    — RustEMS Device Protocol operations (new self-describing API)

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use rusefi_client::rdp::{Push, RdpClient};
use rusefi_client::EcuClient;
use rusefi_device_api::message::ErrorCode;
use rusefi_protocol::transport::tcp;
use std::collections::HashMap;
use tracing::info;

/// Default TCP port for RDP serve mode (`rusefi-sim --serve`).
const RDP_DEFAULT_PORT: u16 = 29002;

#[derive(Parser)]
#[command(name = "rusefi", about = "rusEFI ECU command-line tool", version)]
struct Cli {
    /// ECU host (TCP gateway or simulator)
    #[arg(long, default_value = "127.0.0.1")]
    host: String,

    /// TCP port
    #[arg(long, default_value_t = tcp::DEFAULT_PORT)]
    port: u16,

    /// Chunk size for multi-part reads/writes (bytes)
    #[arg(long, default_value_t = 128)]
    blocking_factor: usize,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Send Hello and print the ECU firmware signature
    Hello,

    /// Read the full configuration image and save it to a file
    ReadImage {
        /// Total image size in bytes (check your INI file)
        #[arg(long)]
        size: usize,

        /// Output file path
        #[arg(long, default_value = "rusefi_config.bin")]
        output: String,
    },

    /// Persist ECU RAM configuration to flash
    Burn,

    /// Request live output channels and print the first N bytes as hex
    OutputChannels {
        /// Total output channel block size in bytes (check your INI file)
        #[arg(long)]
        size: usize,

        /// Number of bytes to display
        #[arg(long, default_value_t = 32)]
        display: usize,
    },

    /// RustEMS Device Protocol (RDP) operations
    Rdp {
        /// RDP TCP port (overrides the global --port for RDP)
        #[arg(long, default_value_t = RDP_DEFAULT_PORT)]
        rdp_port: u16,

        #[command(subcommand)]
        cmd: RdpCommands,
    },
}

#[derive(Subcommand)]
enum RdpCommands {
    /// Identify the device (protocol, board, capabilities, schema hash)
    Hello,

    /// Print schema info and UI categories
    Schema,

    /// List the parameter catalog
    Params,

    /// List the table catalog
    Tables,

    /// List the telemetry channel catalog
    Channels,

    /// Read parameter values by id (repeat --id for multiple)
    Get {
        /// Parameter id (decimal), may be given multiple times
        #[arg(long = "id", required = true)]
        id: Vec<u16>,
    },

    /// Write one parameter value (takes effect in RAM immediately)
    Set {
        /// Parameter id (decimal)
        #[arg(long)]
        id: u16,

        /// New physical value
        #[arg(long)]
        value: f32,
    },

    /// Print a full table: axes and the cell grid
    Table {
        /// Table id
        #[arg(long)]
        id: u16,
    },

    /// Show config staging status (dirty flag + CRCs)
    Status,

    /// Persist staged RAM config to flash
    Save,

    /// Discard staged RAM edits (revert to flash)
    Discard,

    /// Subscribe to telemetry channels and print incoming frames
    Watch {
        /// Channel ids, comma separated (e.g. 1,2,7)
        #[arg(long, value_delimiter = ',', required = true)]
        channels: Vec<u16>,

        /// Push rate in Hz
        #[arg(long, default_value_t = 10)]
        rate: u16,

        /// Number of frames to print before unsubscribing
        #[arg(long, default_value_t = 20)]
        count: u32,
    },

    /// List stored faults (DTCs)
    Faults,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let cli = Cli::parse();
    run(cli).await
}

async fn run(cli: Cli) -> Result<()> {
    if let Commands::Rdp { rdp_port, cmd } = cli.command {
        return run_rdp(&cli.host, rdp_port, cmd).await;
    }

    info!("Connecting to {}:{}", cli.host, cli.port);
    let stream = tcp::connect(&cli.host, cli.port)
        .await
        .with_context(|| format!("Failed to connect to {}:{}", cli.host, cli.port))?;

    let mut client = EcuClient::new(stream).with_blocking_factor(cli.blocking_factor);

    match cli.command {
        Commands::Hello => {
            let sig = client.hello().await?;
            println!("ECU signature: {sig}");
        }

        Commands::ReadImage { size, output } => {
            let sig = client.hello().await?;
            println!("Connected: {sig}");

            info!("Reading {size} bytes...");
            let image = client.read_image(size, &sig).await?;

            std::fs::write(&output, image.as_bytes())
                .with_context(|| format!("Failed to write {output}"))?;
            println!("Saved {size} bytes to {output}");
        }

        Commands::Burn => {
            client.hello().await?;
            client.burn().await?;
            println!("Burn complete.");
        }

        Commands::OutputChannels { size, display } => {
            client.hello().await?;
            let data = client.request_output_channels(size).await?;
            let n = display.min(data.len());
            let hex: Vec<String> = data[..n].iter().map(|b| format!("{b:02X}")).collect();
            println!("Output channels [{n} bytes]: {}", hex.join(" "));
        }

        Commands::Rdp { .. } => unreachable!("handled above"),
    }

    Ok(())
}

/// Render a non-Ok status code as a human-readable failure.
fn check_status(what: &str, code: ErrorCode) -> Result<()> {
    if code == ErrorCode::Ok {
        Ok(())
    } else {
        anyhow::bail!("{what} failed: {code:?}");
    }
}

async fn run_rdp(host: &str, port: u16, cmd: RdpCommands) -> Result<()> {
    info!("Connecting to {host}:{port} (RDP)");
    let mut rdp = RdpClient::connect(host, port)
        .await
        .with_context(|| format!("Failed to connect to {host}:{port}"))?;

    match cmd {
        RdpCommands::Hello => {
            let info = rdp.hello().await?;
            println!("Protocol      : {}.{}", info.proto_major, info.proto_minor);
            println!("Firmware      : {}", info.fw_version);
            println!(
                "Board         : {} (id {})",
                board_name(info.board),
                info.board
            );
            println!("MCU           : {}", info.mcu);
            println!("Cylinders     : {}", info.cylinders);
            println!("Capabilities  : 0x{:08X}", info.capabilities);
            println!("Schema hash   : 0x{:08X}", info.schema_hash);
            println!("Max payload   : {} bytes", info.max_payload);
            let id: Vec<String> = info.device_id.iter().map(|b| format!("{b:02X}")).collect();
            println!("Device ID     : {}", id.join(""));
        }

        RdpCommands::Schema => {
            let s = rdp.schema_info().await?;
            println!("Schema hash : 0x{:08X}", s.schema_hash);
            println!("Parameters  : {}", s.param_count);
            println!("Tables      : {}", s.table_count);
            println!("Categories  :");
            for c in &s.categories {
                println!("  [{}] {}", c.id, c.name);
            }
        }

        RdpCommands::Params => {
            let params = rdp.param_catalog().await?;
            println!(
                "{:>5}  {:<28} {:<10} {:>10} {:>10} {:>10}  flags",
                "id", "key", "unit", "min", "max", "default"
            );
            for p in &params {
                println!(
                    "{:>5}  {:<28} {:<10} {:>10.2} {:>10.2} {:>10.2}  0x{:02X}",
                    p.id, p.key, p.unit, p.min, p.max, p.default, p.flags
                );
            }
        }

        RdpCommands::Tables => {
            let tables = rdp.table_catalog().await?;
            println!(
                "{:>4}  {:<26} {:>4}  {:>5}x{:<5} {:<10}",
                "id", "key", "dims", "x", "y", "cell unit"
            );
            for t in &tables {
                println!(
                    "{:>4}  {:<26} {:>4}  {:>5}x{:<5} {:<10}",
                    t.id, t.key, t.dims, t.x_size, t.y_size, t.cell_unit
                );
            }
        }

        RdpCommands::Channels => {
            let channels = rdp.telemetry_catalog().await?;
            println!(
                "{:>4}  {:<16} {:<24} {:<8} {:>10}",
                "id", "key", "label", "unit", "scale"
            );
            for c in &channels {
                println!(
                    "{:>4}  {:<16} {:<24} {:<8} {:>10.4}",
                    c.id, c.key, c.label, c.unit, c.scale
                );
            }
        }

        RdpCommands::Get { id } => {
            let values = rdp.param_get(&id).await?;
            for (pid, v) in id.iter().zip(values.iter()) {
                println!("param {pid} = {v}");
            }
        }

        RdpCommands::Set { id, value } => {
            let code = rdp.param_set(id, value).await?;
            check_status("ParamSet", code)?;
            println!("param {id} set to {value} (RAM; use `rdp save` to persist)");
        }

        RdpCommands::Table { id } => {
            let t = rdp.table_get(id).await?;
            let x_len = t.x_axis.len().max(1);
            println!("x axis: {:?}", t.x_axis);
            if !t.y_axis.is_empty() {
                println!("y axis: {:?}", t.y_axis);
            }
            println!("cells:");
            for row in t.cells.chunks(x_len) {
                let line: Vec<String> = row.iter().map(|v| format!("{v:>7.2}")).collect();
                println!("  {}", line.join(" "));
            }
        }

        RdpCommands::Status => {
            let (dirty, ram_crc, flash_crc) = rdp.config_status().await?;
            println!("dirty     : {dirty}");
            println!("ram_crc   : 0x{ram_crc:08X}");
            println!("flash_crc : 0x{flash_crc:08X}");
        }

        RdpCommands::Save => {
            let (saved_bytes, crc) = rdp.config_save().await?;
            println!("saved {saved_bytes} bytes, crc 0x{crc:08X}");
        }

        RdpCommands::Discard => {
            let code = rdp.config_discard().await?;
            check_status("ConfigDiscard", code)?;
            println!("staged edits discarded (RAM reverted to flash)");
        }

        RdpCommands::Watch {
            channels,
            rate,
            count,
        } => {
            // Fetch the catalog once so frames decode and print with names.
            let catalog = rdp.telemetry_catalog().await?;
            let names: HashMap<u16, String> = catalog.into_iter().map(|c| (c.id, c.key)).collect();
            let (stream_id, layout, actual_rate) = rdp.subscribe(&channels, rate).await?;
            println!("stream {stream_id} @ {actual_rate} Hz, layout {layout:?}");

            let mut received = 0u32;
            while received < count {
                match rdp.next_push().await? {
                    Push::Telemetry(frame) => {
                        let mut parts: Vec<String> = Vec::with_capacity(layout.len());
                        for (ch, v) in layout.iter().zip(frame.values.iter()) {
                            let name = names.get(ch).map(String::as_str).unwrap_or("?");
                            parts.push(format!("{name}={v:.2}"));
                        }
                        println!(
                            "[{:>8} ms] seq={:<5} {}",
                            frame.ts_ms,
                            frame.seq,
                            parts.join(" ")
                        );
                        received += 1;
                    }
                    Push::Event(ev) => {
                        println!(
                            "event kind={} a={} b={} @ {} ms",
                            ev.kind, ev.a, ev.b, ev.ts_ms
                        );
                    }
                }
            }
            let _ = rdp.unsubscribe(stream_id).await;
        }

        RdpCommands::Faults => {
            let faults = rdp.get_faults().await?;
            if faults.is_empty() {
                println!("no stored faults");
            } else {
                println!(
                    "{:>6}  {:<8} {:>6} {:>6} {:>10} {:>10}  detail",
                    "code", "severity", "active", "count", "first_ms", "last_ms"
                );
                for f in &faults {
                    let sev = match f.severity {
                        0 => "info",
                        1 => "warn",
                        _ => "critical",
                    };
                    println!(
                        "0x{:04X}  {:<8} {:>6} {:>6} {:>10} {:>10}  {}",
                        f.code, sev, f.active, f.count, f.first_ts_ms, f.last_ts_ms, f.detail
                    );
                }
            }
        }
    }

    Ok(())
}

/// Board id → display name (see `engine-core` `comms::rdp::board`).
fn board_name(id: u8) -> &'static str {
    match id {
        0 => "Sim",
        1 => "Nano",
        2 => "microRusEFI",
        3 => "uaEFI",
        4 => "Proteus",
        5 => "Huge",
        _ => "unknown",
    }
}
