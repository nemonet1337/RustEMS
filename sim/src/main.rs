//! rusEFI PC Simulator
//!
//! Runs the rusefi-core control logic against synthetic or CSV trigger data
//! and writes ignition/injection events to a CSV output file.
//!
//! # Usage
//!
//! ```text
//! rusefi-sim --rpm 3000 --cycles 10 --output output.csv
//! rusefi-sim --trigger trigger-log.csv --output output.csv
//! ```

use anyhow::{Context, Result};
use clap::Parser;
use rusefi_core::{
    config::EngineConfig,
    trigger::{MissingToothDecoder, SyncEdge, TriggerSignal},
};
use rusefi_core::trigger::missing_tooth::MissingToothConfig;
use rusefi_hal_sim::{
    SimAdcInput, SimIgnitionOutput, SimSystemTimer, SimTriggerInput,
};
#[cfg(feature = "fuel-fi")]
use rusefi_hal_sim::SimInjectorOutput;
use rusefi_core::hal::{AdcInput, IgnitionOutput, TriggerInput};
#[cfg(feature = "fuel-fi")]
use rusefi_core::hal::InjectorOutput;
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser, Debug)]
#[command(name = "rusefi-sim", about = "rusEFI Rust firmware PC simulator")]
struct Cli {
    /// Synthetic engine RPM (used when --trigger is not provided).
    #[arg(long, default_value = "3000")]
    rpm: f32,

    /// Number of engine cycles to simulate.
    #[arg(long, default_value = "5")]
    cycles: u32,

    /// Optional CSV trigger log file (timestamp_us,is_crank columns).
    #[arg(long)]
    trigger: Option<PathBuf>,

    /// Output CSV file for simulation results.
    #[arg(long, default_value = "output.csv")]
    output: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let cfg = EngineConfig::default_4cyl();

    // ── trigger source ───────────────────────────────────────────────────────
    let mut trigger_in = SimTriggerInput::new();

    if let Some(ref csv_path) = cli.trigger {
        load_trigger_csv(&mut trigger_in, csv_path)
            .with_context(|| format!("loading trigger CSV: {}", csv_path.display()))?;
        println!("Loaded trigger CSV: {}", csv_path.display());
    } else {
        println!(
            "Generating synthetic 36-1 trigger at {:.0} RPM for {} cycles",
            cli.rpm, cli.cycles
        );
        trigger_in.generate_missing_tooth(
            cfg.trigger_total_teeth,
            cfg.trigger_missing_teeth,
            cli.rpm,
            cli.cycles,
            0,
        );
    }

    // ── decoder ──────────────────────────────────────────────────────────────
    let mut decoder = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: cfg.trigger_total_teeth,
        missing_teeth: cfg.trigger_missing_teeth,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });

    // ── HAL devices ──────────────────────────────────────────────────────────
    let mut ignition_out = SimIgnitionOutput::new();
    #[cfg(feature = "fuel-fi")]
    let mut injector_out = SimInjectorOutput::new();
    let mut adc = SimAdcInput::new();
    let _timer = SimSystemTimer::new();
    
    // Pre-seed ADC with synthetic sensor values
    adc.set_map(2000);   // ~50 kPa
    adc.set_clt(1500);   // ~80°C
    adc.set_vbatt(2500); // ~12V

    // ── output CSV ───────────────────────────────────────────────────────────
    let out_file = File::create(&cli.output)
        .with_context(|| format!("creating output file: {}", cli.output.display()))?;
    let mut writer = BufWriter::new(out_file);
    writeln!(
        writer,
        "timestamp_us,tooth_index,sync,rpm_est,event_type,cylinder,pulse_ms"
    )?;

    // ── main simulation loop ─────────────────────────────────────────────────
    let mut event_count = 0u64;
    let mut fire_count = 0u64;
    let mut inj_count = 0u64;
    
    use rusefi_core::sensors::{adc_to_volts, AdcChannel, SensorData};
    use rusefi_core::ignition::{compute_ignition, tdc_angles_from_firing_order};
    #[cfg(feature = "fuel-fi")]
    use rusefi_core::fuel::{compute_injection, estimate_airmass_g};

    loop {
        // Drain crank pulses
        let Some(ts) = trigger_in.read_crank_timestamp() else {
            break;
        };

        let result = decoder.process(TriggerSignal::CrankRise, ts);

        match result {
            Ok(state) => {
                let rpm_str = state
                    .rpm
                    .map(|r| format!("{:.1}", r))
                    .unwrap_or_else(|| "N/A".to_string());

                writeln!(
                    writer,
                    "{},{},{:?},{},trigger,-,",
                    ts,
                    state.tooth_index,
                    state.sync,
                    rpm_str
                )?;

                // Process full cycle when synced at tooth 0
                if state.tooth_index == 0
                    && state.sync == rusefi_core::trigger::SyncState::CrankSynced
                {
                    if let Some(rpm) = state.rpm {
                        // Build sensor snapshot
                        let map_raw = adc.read_raw(AdcChannel::Map);
                        let map_v = adc_to_volts(map_raw);
                        let map_kpa = map_v * 50.0;
                        
                        let clt_raw = adc.read_raw(AdcChannel::Clt);
                        let clt_v = adc_to_volts(clt_raw);
                        
                        let vbatt_raw = adc.read_raw(AdcChannel::Vbatt);
                        let vbatt_v = adc_to_volts(vbatt_raw) * 8.232;

                        let sensors = SensorData {
                            rpm: Some(rpm),
                            load_pct: Some(map_kpa / 101.325 * 100.0),
                            clt_celsius: Some((clt_v - 0.5) / 0.01),
                            iat_celsius: None,
                            tps_pct: None,
                            map_kpa: Some(map_kpa),
                            battery_volts: Some(vbatt_v),
                            maf_voltage: None,
                            fuel_level_pct: None,
                            oil_pressure_kpa: None,
                            lambda1_voltage: None,
                            lambda2_voltage: None,
                        };

                        // Process all cylinders in firing order
                        let tdc_angles = tdc_angles_from_firing_order(&cfg.firing_order);
                        
                        for (i, &cyl) in cfg.firing_order.iter().enumerate() {
                            let tdc_deg = tdc_angles[i];
                            
                            // Ignition
                            if let Some(ign) = compute_ignition(&cfg, &sensors, tdc_deg) {
                                ignition_out.coil_charge(cyl);
                                ignition_out.coil_fire(cyl);
                                fire_count += 1;
                                writeln!(
                                    writer,
                                    "{},{},{:?},{:.1},ignition_fire,{},",
                                    ts, state.tooth_index, state.sync, rpm, cyl
                                )?;
                            }
                            
                            // Fuel injection
                            #[cfg(feature = "fuel-fi")]
                            {
                                let airmass = estimate_airmass_g(map_kpa, cfg.displacement_cc_per_cyl, 0.85);
                                if let Some(inj) = compute_injection(&cfg, &sensors, airmass) {
                                    injector_out.open(cyl);
                                    injector_out.close(cyl);
                                    inj_count += 1;
                                    writeln!(
                                        writer,
                                        "{},{},{:?},{:.1},injection_pulse,{},{:.2}",
                                        ts, state.tooth_index, state.sync, rpm, cyl, inj.pulse_ms
                                    )?;
                                }
                            }
                        }
                    }
                }

                event_count += 1;
            }
            Err(e) => {
                writeln!(writer, "{},-,-,-,error,{:?},", ts, e)?;
            }
        }
    }

    writer.flush()?;

    println!(
        "Simulation complete: {} trigger events, {} ignition fires, {} injections → {}",
        event_count,
        fire_count,
        inj_count,
        cli.output.display()
    );

    Ok(())
}

/// Load trigger events from a CSV file.
///
/// Expected format: `timestamp_us,is_crank` (header row optional).
fn load_trigger_csv(trigger: &mut SimTriggerInput, path: &PathBuf) -> Result<()> {
    let content = std::fs::read_to_string(path)?;
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("timestamp") {
            continue;
        }
        let mut parts = line.splitn(2, ',');
        let ts_str = parts.next().unwrap_or("").trim();
        let is_crank_str = parts.next().unwrap_or("1").trim();

        let ts: u64 = ts_str
            .parse()
            .with_context(|| format!("parsing timestamp: '{ts_str}'"))?;
        let is_crank: bool = is_crank_str != "0" && is_crank_str != "false";

        if is_crank {
            trigger.push_crank(ts);
        } else {
            trigger.push_cam(ts);
        }
    }
    Ok(())
}
