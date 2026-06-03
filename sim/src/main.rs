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
    trigger::{MissingToothDecoder, SyncEdge, SyncState, TriggerSignal},
};
use rusefi_core::trigger::missing_tooth::MissingToothConfig;
use rusefi_hal_sim::{
    SimAdcInput, SimIgnitionOutput, SimPwmOutput, SimRelayOutput, SimSystemTimer, SimTriggerInput,
};
#[cfg(feature = "fuel-fi")]
use rusefi_hal_sim::SimInjectorOutput;
use rusefi_core::hal::{AdcInput, IgnitionOutput, SystemTimer, TriggerInput};
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
    let timer = SimSystemTimer::new();

    // Stub PWM / relay outputs
    let mut iac_pwm      = SimPwmOutput::new();
    let mut boost_pwm    = SimPwmOutput::new();
    let mut fuel_pump_relay = SimRelayOutput::new();
    let mut fan_relay    = SimRelayOutput::new();

    // Pre-seed ADC with synthetic sensor values (stoichiometric lambda @ 2.5V wideband)
    adc.set_map(2000);   // ~50 kPa
    adc.set_clt(1500);   // ~80°C  (linearized)
    adc.set_vbatt(2500); // ~12V
    adc.set_lambda1(1250); // ~2.5V → lambda ~1.0 (stoichiometric for wideband)

    // ── Controller initialisation ─────────────────────────────────────────────
    use rusefi_core::sensors::{
        adc_to_volts, AdcChannel, IirFilter, LambdaSensor, LambdaSensorConfig, SensorData,
    };
    use rusefi_core::ignition::{
        compute_ignition, tdc_angles_from_firing_order,
        OverdwellConfig, OverdwellController, RpmLimiter, RpmLimiterConfig,
    };
    #[cfg(feature = "fuel-fi")]
    use rusefi_core::fuel::{
        compute_injection, estimate_airmass_g,
        AccelEnrichmentConfig, AccelEnrichmentController,
        ClosedLoopConfig, ClosedLoopController,
        DfcoConfig, DfcoController,
        ltft::LtftState,
        wall_wetting::{WallWettingConfig, MultiCylWallWetting},
    };
    use rusefi_core::outputs::{FanController, FanMode, FuelPumpConfig, FuelPumpController};
    use rusefi_core::actuators::{BoostConfig, BoostController, IdleConfig, IdleController};
    use rusefi_core::protection::ProtectionMonitor;
    use rusefi_core::hal::PwmOutput;
    use rusefi_core::hal::RelayOutput;
    use rusefi_core::config::MAX_CYLINDERS;

    let mut rpm_limiter = RpmLimiter::new(RpmLimiterConfig::default());
    let mut overdwell: [OverdwellController; MAX_CYLINDERS] =
        core::array::from_fn(|_| OverdwellController::new(OverdwellConfig::default()));

    #[cfg(feature = "fuel-fi")]
    let num_cylinders = cfg.firing_order.len() as u8;
    #[cfg(feature = "fuel-fi")]
    let mut dfco         = DfcoController::new(DfcoConfig::default());
    #[cfg(feature = "fuel-fi")]
    let mut accel        = AccelEnrichmentController::new(AccelEnrichmentConfig::default());
    #[cfg(feature = "fuel-fi")]
    let mut closed_loop  = ClosedLoopController::new(ClosedLoopConfig {
        enabled: true,
        ..ClosedLoopConfig::default()
    });
    #[cfg(feature = "fuel-fi")]
    let mut ltft         = LtftState::new();
    #[cfg(feature = "fuel-fi")]
    let mut wall_wetting = MultiCylWallWetting::new(WallWettingConfig::default(), num_cylinders);
    let lambda_sensor    = LambdaSensor::new(LambdaSensorConfig::default());

    let mut fuel_pump = FuelPumpController::new(FuelPumpConfig::default());
    let mut fan       = FanController::default_engine();
    let mut idle      = IdleController::new(IdleConfig::default_4cyl());
    let mut boost     = BoostController::new(BoostConfig::default_turbo());
    let mut protection = ProtectionMonitor::new();

    // Filters for sensor smoothing
    let mut clt_filter = IirFilter::new(0.1);
    let mut map_filter = IirFilter::new(0.2);
    let mut tps_filter = IirFilter::new(0.3);

    fuel_pump.on_key_on();
    fuel_pump_relay.on();

    #[cfg(feature = "fuel-fi")]
    let mut dfco_active     = false;
    #[cfg(feature = "fuel-fi")]
    let mut cl_correction   = 1.0f32;
    #[cfg(feature = "fuel-fi")]
    let mut ltft_correction = 1.0f32;
    #[cfg(feature = "fuel-fi")]
    let mut accel_mult      = 1.0f32;

    // ── output CSV ───────────────────────────────────────────────────────────
    let out_file = File::create(&cli.output)
        .with_context(|| format!("creating output file: {}", cli.output.display()))?;
    let mut writer = BufWriter::new(out_file);
    writeln!(
        writer,
        "timestamp_us,tooth_index,sync,rpm_est,event_type,cylinder,pulse_ms,advance_deg,lambda,dfco,cl_corr,ltft,accel"
    )?;

    // ── main simulation loop ─────────────────────────────────────────────────
    let mut event_count = 0u64;
    let mut fire_count  = 0u64;
    let mut inj_count   = 0u64;
    let mut last_us: u64 = 0;

    loop {
        let Some(ts) = trigger_in.read_crank_timestamp() else { break };

        let now_us = timer.now_us().max(ts);
        let dt_s = if last_us > 0 {
            (now_us.saturating_sub(last_us) as f32 / 1_000_000.0).clamp(0.00005, 0.5)
        } else {
            0.001
        };
        let dt_ms = (dt_s * 1000.0) as u32;
        last_us = now_us;

        // ── Sensor snapshot ──────────────────────────────────────────────────
        let map_kpa  = map_filter.update(adc_to_volts(adc.read_raw(AdcChannel::Map)) * 50.0);
        let clt_c    = clt_filter.update((adc_to_volts(adc.read_raw(AdcChannel::Clt)) - 0.5) / 0.01);
        let tps_pct  = tps_filter.update(0.0f32); // wide-open throttle = 0 in idle sim
        let vbatt_v  = adc_to_volts(adc.read_raw(AdcChannel::Vbatt)) * 8.232;
        let lambda1_v = adc_to_volts(adc.read_raw(AdcChannel::Lambda1));
        let lambda_meas = lambda_sensor.voltage_to_lambda(lambda1_v).unwrap_or(1.0);

        // ── Actuator updates ─────────────────────────────────────────────────
        let sens_snap = SensorData {
            clt_celsius: Some(clt_c),
            map_kpa:     Some(map_kpa),
            ..Default::default()
        };
        let _prot = protection.update(&sens_snap, dt_ms);

        if fuel_pump.update(0.0, dt_ms) { fuel_pump_relay.on(); } else { fuel_pump_relay.off(); }
        match fan.update(clt_c) {
            FanMode::On | FanMode::Pwm(_) => fan_relay.on(),
            FanMode::Off => fan_relay.off(),
        }
        let iac = idle.update(0.0, clt_c, tps_pct, false, dt_s * 1000.0, false);
        iac_pwm.set_duty(iac);
        let boost_d = boost.update(map_kpa, 0.0, tps_pct, false, dt_s * 1000.0);
        boost_pwm.set_duty(boost_d);

        // ── Trigger decoding ─────────────────────────────────────────────────
        let result = decoder.process(TriggerSignal::CrankRise, ts);

        match result {
            Ok(state) => {
                let rpm_str = state.rpm.map(|r| format!("{r:.1}")).unwrap_or_else(|| "N/A".to_string());
                writeln!(
                    writer,
                    "{},{},{:?},{},trigger,-,,,,,,",
                    ts, state.tooth_index, state.sync, rpm_str
                )?;

                if state.tooth_index == 0
                    && matches!(state.sync, SyncState::CrankSynced | SyncState::FullSync)
                {
                    if let Some(rpm) = state.rpm {
                        if fuel_pump.update(rpm, 0) { fuel_pump_relay.on(); }

                        let sensors = SensorData {
                            rpm:           Some(rpm),
                            load_pct:      Some(map_kpa / 101.325 * 100.0),
                            clt_celsius:   Some(clt_c),
                            tps_pct:       Some(tps_pct),
                            map_kpa:       Some(map_kpa),
                            battery_volts: Some(vbatt_v),
                            lambda1_voltage: Some(lambda1_v),
                            ..Default::default()
                        };

                        // ── Per-cycle fuel corrections ─────────────────────
                        #[cfg(feature = "fuel-fi")]
                        {
                            dfco_active = dfco.update(rpm, map_kpa, tps_pct, ts);
                            if dfco_active { closed_loop.trigger_pause(); }

                            cl_correction  = closed_loop.update(rpm, clt_c, lambda_meas, dt_s);
                            ltft.update(&sensors, 1.0);
                            ltft_correction = ltft.get_trim(rpm, map_kpa / 101.325 * 100.0);
                            accel_mult = accel.update(tps_pct, ts);
                        }

                        // ── Idle / boost per-cycle update ──────────────────
                        let iac2 = idle.update(rpm, clt_c, tps_pct, rpm < cfg.cranking_rpm, dt_s * 1000.0, false);
                        iac_pwm.set_duty(iac2);
                        let bd2 = boost.update(map_kpa, rpm, tps_pct, false, dt_s * 1000.0);
                        boost_pwm.set_duty(bd2);

                        let tdc_angles = tdc_angles_from_firing_order(&cfg.firing_order);
                        let spark_cut  = rpm_limiter.update(rpm);

                        for (i, &cyl) in cfg.firing_order.iter().enumerate() {
                            let tdc_deg = tdc_angles[i];

                            // Ignition
                            if !spark_cut {
                                if let Some(ign) = compute_ignition(&cfg, &sensors, tdc_deg) {
                                    let od = &mut overdwell[cyl as usize % MAX_CYLINDERS];
                                    od.start_charge(ts);
                                    ignition_out.coil_charge(cyl);
                                    ignition_out.coil_fire(cyl);
                                    od.end_charge();
                                    fire_count += 1;
                                    writeln!(
                                        writer,
                                        "{},{},{:?},{:.1},ignition_fire,{},0,{:.1},{:.3},,,",
                                        ts, state.tooth_index, state.sync, rpm, cyl, ign.advance_deg, lambda_meas
                                    )?;
                                }
                            }

                            // Injection
                            #[cfg(feature = "fuel-fi")]
                            if !dfco_active {
                                let airmass = estimate_airmass_g(map_kpa, cfg.displacement_cc_per_cyl, 0.85);
                                if let Some(inj) = compute_injection(&cfg, &sensors, airmass) {
                                    let corrected_mass = inj.fuel_mass_g
                                        * cl_correction
                                        * ltft_correction
                                        * accel_mult;
                                    let ww_mass = wall_wetting.compensate(cyl, corrected_mass, clt_c, dt_s);
                                    let flow_g_s = cfg.injector_flow_cc_per_min * 0.755 / 60.0;
                                    let deadtime  = inj.pulse_ms - inj.open_ms;
                                    let pulse_ms  = (ww_mass / flow_g_s * 1000.0 + deadtime).max(0.0);

                                    injector_out.open(cyl);
                                    injector_out.close(cyl);
                                    inj_count += 1;
                                    writeln!(
                                        writer,
                                        "{},{},{:?},{:.1},injection_pulse,{},{:.3},0.0,{:.3},{},{:.3},{:.3},{:.3}",
                                        ts, state.tooth_index, state.sync, rpm, cyl, pulse_ms,
                                        lambda_meas, dfco_active,
                                        cl_correction, ltft_correction, accel_mult
                                    )?;
                                }
                            }
                        }
                    }
                }

                event_count += 1;
            }
            Err(e) => {
                writeln!(writer, "{},-,-,-,error,{e:?},,,,,,", ts)?;
            }
        }
    }

    writer.flush()?;

    println!(
        "Simulation complete: {} trigger events, {} ignition fires, {} injections → {}",
        event_count, fire_count, inj_count, cli.output.display()
    );
    println!(
        "  Actuators: iac_duty={:.1}% boost_duty={:.1}% fuel_pump={} fan={}",
        iac_pwm.duty(), boost_pwm.duty(),
        if fuel_pump_relay.is_on() { "ON" } else { "OFF" },
        if fan_relay.is_on() { "ON" } else { "OFF" },
    );

    Ok(())
}

/// Load trigger events from a CSV file.
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

        if is_crank { trigger.push_crank(ts); } else { trigger.push_cam(ts); }
    }
    Ok(())
}

