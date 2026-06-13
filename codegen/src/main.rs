//! rusEFI Configuration Code Generator
//!
//! Rust port of the Java configuration_definition toolset.
//! Generates C headers and TunerStudio INI files from rusefi_config.txt.

// Modules are defined in lib.rs; the binary only links to them via the lib crate.
// Until the full pipeline is wired up, suppress dead_code for the bin target.
#![allow(dead_code)]

use clap::Parser;
use std::path::PathBuf;
use tracing::{error, info};

use anyhow::Result;

/// rusEFI Configuration Code Generator
#[derive(Parser, Debug)]
#[command(name = "rusefi-codegen")]
#[command(about = "Generate C headers and TunerStudio INI from rusefi_config.txt")]
#[command(version)]
struct Args {
    /// Main definition file (e.g., integration/rusefi_config.txt)
    #[arg(short, long)]
    definition: PathBuf,

    /// TunerStudio destination folder
    #[arg(long)]
    ts_destination: Option<PathBuf>,

    /// C header output file (can be specified multiple times)
    #[arg(long = "c_destination")]
    c_destinations: Vec<PathBuf>,

    /// C defines output file
    #[arg(long)]
    c_defines: Option<PathBuf>,

    /// Java destination folder
    #[arg(long)]
    java_destination: Option<PathBuf>,

    /// Prepend files to read before main processing
    #[arg(long)]
    prepend: Vec<PathBuf>,

    /// Soft prepend files (ignored if not exist)
    #[arg(long)]
    soft_prepend: Vec<PathBuf>,

    /// Read file content into registry key (format: key:path)
    #[arg(long = "readfile")]
    readfiles: Vec<String>,

    /// Enum input files
    #[arg(long = "enumInputFile")]
    enum_input_files: Vec<PathBuf>,

    /// Firing order enum file
    #[arg(long)]
    firing_order: Option<PathBuf>,

    /// Ignore gauges files
    #[arg(long = "ignore_gauges_file")]
    ignore_gauges_files: Vec<PathBuf>,

    /// Field lookup output files (format: cpp:md)
    #[arg(long)]
    field_lookup_file: Option<String>,

    /// Signature input file
    #[arg(long)]
    signature: Option<PathBuf>,

    /// Signature output file
    #[arg(long)]
    signature_destination: Option<PathBuf>,

    /// TS output INI filename
    #[arg(long)]
    ts_output_name: Option<String>,

    /// Board directory for config files
    #[arg(long)]
    board: Option<PathBuf>,

    /// Tool name for messages
    #[arg(long)]
    tool: Option<String>,

    /// Initialize to zero flag
    #[arg(long)]
    initialize_to_zero: Option<bool>,

    /// Include C defines in output
    #[arg(long)]
    with_c_defines: Option<bool>,

    /// Trigger input folder
    #[arg(long)]
    trigger_input_folder: Option<PathBuf>,

    /// Verbose output
    #[arg(short, long)]
    verbose: bool,
}

fn main() {
    let args = Args::parse();

    let subscriber = tracing_subscriber::fmt()
        .with_max_level(if args.verbose {
            tracing::Level::DEBUG
        } else {
            tracing::Level::INFO
        })
        .finish();

    let _ = tracing::subscriber::set_global_default(subscriber);

    if let Err(e) = run(args) {
        error!("Error: {}", e);
        std::process::exit(1);
    }
}

fn run(args: Args) -> Result<()> {
    info!("rusefi-codegen v{} starting", rusefi_codegen::VERSION);
    info!("Definition file: {:?}", args.definition);

    // Build ordered list of definition files: prepend -> soft_prepend -> main
    let mut definition_files: Vec<std::path::PathBuf> = Vec::new();
    for p in &args.prepend {
        definition_files.push(p.clone());
    }
    for p in &args.soft_prepend {
        if p.exists() {
            definition_files.push(p.clone());
        } else {
            tracing::debug!("soft_prepend not found, skipping: {:?}", p);
        }
    }
    definition_files.push(args.definition.clone());

    let definition_refs: Vec<&std::path::Path> =
        definition_files.iter().map(|p| p.as_path()).collect();
    let c_dest_refs: Vec<&std::path::Path> =
        args.c_destinations.iter().map(|p| p.as_path()).collect();

    let opts = rusefi_codegen::GenerateOptions {
        definition_files: definition_refs,
        c_destinations: c_dest_refs,
        ts_destination: args.ts_destination.as_deref(),
        with_c_defines: args.with_c_defines.unwrap_or(true),
        initialize_to_zero: args.initialize_to_zero.unwrap_or(false),
    };

    rusefi_codegen::generate(&opts)?;

    info!("Code generation complete");
    Ok(())
}
