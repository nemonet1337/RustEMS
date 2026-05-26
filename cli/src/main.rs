//! `rusefi` — command-line interface for interacting with a rusEFI ECU.
//!
//! ## Subcommands
//!
//! - `hello`      — connect, send Hello and print the firmware signature
//! - `read-image` — read the full configuration image and save it to a file
//! - `burn`       — persist ECU RAM configuration to flash

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use rusefi_client::EcuClient;
use rusefi_protocol::transport::tcp;
use tracing::info;

#[derive(Parser)]
#[command(
    name = "rusefi",
    about = "rusEFI ECU command-line tool",
    version
)]
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
    }

    Ok(())
}
