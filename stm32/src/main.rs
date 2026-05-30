//! rusEFI STM32 firmware entry point.
//!
//! Selects the correct HAL crate at compile time via feature flags:
//! - `stm32f4` → `rusefi-hal-microrusefi` (microRusEFI, STM32F407ZGT6)
//! - `stm32f7` → `rusefi-hal-proteus` (Proteus F7, STM32F767ZIT6)
//!
//! Embassy executor spawns three tasks:
//! 1. `crank_task`   — EXTI interrupt producer (crank pulses)
//! 2. `cam_task`     — EXTI interrupt producer (cam pulses)
//! 3. `control_task` — main control loop (trigger decode → ignition → fuel)

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

#[cfg(feature = "stm32f4")]
use rusefi_hal_microrusefi as hal;

#[cfg(feature = "stm32f7")]
use rusefi_hal_proteus as hal;

#[cfg(feature = "uaefi")]
use rusefi_hal_uaefi as hal;

#[cfg(feature = "stm32f4-huge")]
use rusefi_hal_huge as hal;

#[cfg(feature = "stm32f4-nano")]
use rusefi_hal_nano as hal;

use rusefi_core::{
    config::EngineConfig,
    trigger::{
        missing_tooth::{MissingToothConfig, MissingToothDecoder},
        SyncEdge, SyncState, TriggerSignal,
    },
};
use rusefi_core::hal::{AdcInput, IgnitionOutput, TriggerInput};
#[cfg(feature = "fuel-fi")]
use rusefi_core::hal::InjectorOutput;

// ─── Embassy entry point ─────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("rusEFI STM32 firmware starting");

    // ── Trigger input ─────────────────────────────────────────────────────
    let (trigger_in, producers) = hal::trigger::Stm32TriggerInput::init();

    // ── Ignition output ───────────────────────────────────────────────────
    // microRusEFI: 4 cylinders.
    #[cfg(feature = "stm32f4")]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(
        p.PE14, p.PE13, p.PE12, p.PE11,
    );
    // Proteus / Huge: up to 12 cylinders.
    #[cfg(any(feature = "stm32f7", feature = "stm32f4-huge"))]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(
        p.PE4, p.PE5, p.PE6, p.PE7, p.PE8, p.PE9, p.PE10, p.PE11, p.PE12, p.PE13,
        p.PE14, p.PE15,
    );
    #[cfg(feature = "stm32f4-nano")]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(p.PE14, p.PE13);
    #[cfg(feature = "uaefi")]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(
        p.PE14, p.PE13, p.PE12, p.PE11, p.PE10, p.PE9,
    );

    // ── ADC input ─────────────────────────────────────────────────────────
    let adc_in = hal::adc::Stm32AdcInput::new(
        p.ADC1, p.PA0, p.PA1, p.PC3, p.PC0, p.PC1,
    );

    // ── Timer ─────────────────────────────────────────────────────────────
    let timer = hal::timer::Stm32SystemTimer::new();

    // ── Injector output (fuel-fi only) ────────────────────────────────────
    // microRusEFI: 4 cylinders.
    #[cfg(all(feature = "fuel-fi", feature = "stm32f4"))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(
        p.PB9, p.PB8, p.PD15, p.PD14,
    );
    // Proteus / Huge: up to 12 cylinders.
    #[cfg(all(feature = "fuel-fi", any(feature = "stm32f7", feature = "stm32f4-huge")))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(
        p.PF0, p.PF1, p.PF2, p.PF3, p.PF4, p.PF5, p.PF6, p.PF7, p.PF8, p.PF9,
        p.PF10, p.PF11,
    );
    #[cfg(all(feature = "fuel-fi", feature = "stm32f4-nano"))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(
        p.PB9, p.PB8, p.PD15, p.PD14, p.PD13, p.PD12, p.PD11, p.PD10,
    );
    #[cfg(all(feature = "fuel-fi", feature = "uaefi"))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(
        p.PB9, p.PB8, p.PD15, p.PD14, p.PD13, p.PD12,
    );

    // ── Engine config ─────────────────────────────────────────────────────
    // Select the calibration matching the compiled cylinder-count feature.
    let cfg = engine_config();

    // Spawn EXTI tasks
    spawner.spawn(crank_task(p.PA8, p.EXTI8, producers.crank).expect("spawn crank_task"));
    spawner.spawn(cam_task(p.PA5, p.EXTI5, producers.cam).expect("spawn cam_task"));

    // Run the control loop directly (highest priority, no yield needed)
    #[cfg(feature = "fuel-fi")]
    control_loop(cfg, trigger_in, ignition_out, adc_in, timer, injector_out).await;
    #[cfg(not(feature = "fuel-fi"))]
    control_loop_carb(cfg, trigger_in, ignition_out, adc_in, timer).await;
}

/// Select the engine calibration that matches the compiled `cyl-N` feature.
///
/// Exactly one cylinder-count feature is enabled per firmware build, so only
/// one branch is compiled in.
fn engine_config() -> EngineConfig {
    #[cfg(feature = "cyl-1")]
    return EngineConfig::default_1cyl();
    #[cfg(feature = "cyl-2")]
    return EngineConfig::default_2cyl();
    #[cfg(feature = "cyl-3")]
    return EngineConfig::default_3cyl();
    #[cfg(feature = "cyl-4")]
    return EngineConfig::default_4cyl();
    #[cfg(feature = "cyl-5")]
    return EngineConfig::default_5cyl();
    #[cfg(feature = "cyl-6")]
    return EngineConfig::default_6cyl();
    #[cfg(feature = "cyl-8")]
    return EngineConfig::default_8cyl();
    #[cfg(feature = "cyl-10")]
    return EngineConfig::default_10cyl();
    #[cfg(feature = "cyl-12")]
    return EngineConfig::default_12cyl();
    #[cfg(not(any(
        feature = "cyl-1",
        feature = "cyl-2",
        feature = "cyl-3",
        feature = "cyl-4",
        feature = "cyl-5",
        feature = "cyl-6",
        feature = "cyl-8",
        feature = "cyl-10",
        feature = "cyl-12",
    )))]
    return EngineConfig::default_4cyl();
}

// ─── EXTI tasks ──────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn crank_task(
    pa8: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA8>,
    exti8: embassy_stm32::Peri<'static, embassy_stm32::peripherals::EXTI8>,
    tx: heapless::spsc::Producer<'static, u64>,
) {
    hal::trigger::crank_exti_task(pa8, exti8, tx).await;
}

#[embassy_executor::task]
async fn cam_task(
    pa5: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA5>,
    exti5: embassy_stm32::Peri<'static, embassy_stm32::peripherals::EXTI5>,
    tx: heapless::spsc::Producer<'static, u64>,
) {
    hal::trigger::cam_exti_task(pa5, exti5, tx).await;
}

// ─── Control loop ─────────────────────────────────────────────────────────────

/// Main control loop with fuel injection and sequential cam-sync support.
///
/// Injection mode automatically upgrades from batch to sequential once a cam
/// pulse establishes FullSync (720° phase known).
#[cfg(feature = "fuel-fi")]
async fn control_loop(
    cfg: EngineConfig,
    mut trigger: hal::trigger::Stm32TriggerInput,
    mut ignition: hal::ignition::Stm32IgnitionOutput,
    mut adc: hal::adc::Stm32AdcInput,
    _timer: hal::timer::Stm32SystemTimer,
    mut injector: hal::injector::Stm32InjectorOutput,
) {
    use rusefi_core::sensors::{adc_to_volts, AdcChannel, IirFilter, SensorData};
    use rusefi_core::ignition::{
        compute_ignition, tdc_angles_from_firing_order, RpmLimiter, RpmLimiterConfig,
    };
    use rusefi_core::fuel::{compute_injection, estimate_airmass_g};
    use rusefi_core::engine_cycle::SequentialInjection;

    let mut decoder = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: cfg.trigger_total_teeth,
        missing_teeth: cfg.trigger_missing_teeth,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });

    let mut clt_filter = IirFilter::new(0.1);
    let mut iat_filter = IirFilter::new(0.1);
    let mut map_filter = IirFilter::new(0.2);
    let mut tps_filter = IirFilter::new(0.3);

    // Hard RPM limiter (spark cut) for over-rev protection.
    let mut rpm_limiter = RpmLimiter::new(RpmLimiterConfig::default());

    // Sequential injection — fires each cylinder individually at the right intake angle.
    // Inactive until a cam pulse establishes FullSync.
    let mut seq_inj = SequentialInjection::new(&cfg.firing_order, 90.0);

    // Which cylinders have already been injected in the current 720° cycle.
    let mut cycle_injected = [false; rusefi_core::config::MAX_CYLINDERS];

    defmt::info!("Control loop started (fuel-injection, sequential-capable)");

    loop {
        // ── Sensor snapshot ───────────────────────────────────────────────
        let map_raw = adc.read_raw(AdcChannel::Map);
        let map_kpa = map_filter.update(adc_to_volts(map_raw) * 50.0);
        let clt_raw = adc.read_raw(AdcChannel::Clt);
        let clt_c = clt_filter.update((adc_to_volts(clt_raw) - 0.5) / 0.01);
        let iat_raw = adc.read_raw(AdcChannel::Iat);
        let iat_c = iat_filter.update((adc_to_volts(iat_raw) - 0.5) / 0.01);
        let tps_raw = adc.read_raw(AdcChannel::Tps);
        let tps_pct = tps_filter.update((adc_to_volts(tps_raw) / 5.0 * 100.0).clamp(0.0, 100.0));
        let vbatt_raw = adc.read_raw(AdcChannel::Vbatt);
        let vbatt_v = adc_to_volts(vbatt_raw) * 8.232;

        // ── Crank pulse processing ────────────────────────────────────────
        while let Some(ts) = trigger.read_crank_timestamp() {
            match decoder.process(TriggerSignal::CrankRise, ts) {
                Ok(state) => {
                    // Reset per-cycle injection tracking at the cycle boundary (gap tooth)
                    if state.tooth_index == 0 {
                        cycle_injected = [false; rusefi_core::config::MAX_CYLINDERS];
                    }

                    let rpm = match state.rpm {
                        Some(r) if r > 50.0 => r,
                        _ => continue,
                    };

                    // Hard RPM cut for over-rev protection (hysteresis built in).
                    let spark_cut = rpm_limiter.update(rpm);

                    let sensors = SensorData {
                        rpm: Some(rpm),
                        load_pct: Some(map_kpa / 101.325 * 100.0),
                        clt_celsius: Some(clt_c),
                        iat_celsius: Some(iat_c),
                        tps_pct: Some(tps_pct),
                        map_kpa: Some(map_kpa),
                        battery_volts: Some(vbatt_v),
                        ..Default::default()
                    };

                    let airmass = estimate_airmass_g(
                        map_kpa, cfg.displacement_cc_per_cyl, 0.85,
                    );

                    // ── Sequential injection (FullSync only) ──────────────────
                    // Fires one cylinder per intake stroke window, per tooth.
                    if let Some(cyl) = seq_inj.update(&state) {
                        let ci = cyl as usize;
                        if ci < cycle_injected.len() && !cycle_injected[ci] {
                            cycle_injected[ci] = true;
                            if let Some(inj) = compute_injection(&cfg, &sensors, airmass) {
                                injector.open(cyl);
                                hal::timer::Stm32SystemTimer::sleep_us(
                                    (inj.pulse_ms * 1000.0) as u64,
                                )
                                .await;
                                injector.close(cyl);
                                defmt::debug!("SEQ INJ cyl{} {}ms", cyl, inj.pulse_ms);
                            }
                        }
                    }

                    // ── Ignition + batch injection fallback at TDC reference ──
                    // Runs each cycle when synced. Injection is skipped when
                    // sequential mode is active.
                    if state.tooth_index == 0
                        && matches!(
                            state.sync,
                            SyncState::CrankSynced | SyncState::FullSync
                        )
                    {
                        let tdc_angles = tdc_angles_from_firing_order(&cfg.firing_order);
                        let batch_inj = !seq_inj.is_sequential();

                        for (i, &cyl) in cfg.firing_order.iter().enumerate() {
                            let tdc_deg = tdc_angles[i];

                            // Skip spark entirely above the RPM limit.
                            if !spark_cut {
                                if let Some(ign) = compute_ignition(&cfg, &sensors, tdc_deg) {
                                    ignition.coil_charge(cyl);
                                    hal::timer::Stm32SystemTimer::sleep_us(
                                        (ign.dwell_ms * 1000.0) as u64,
                                    )
                                    .await;
                                    ignition.coil_fire(cyl);
                                    defmt::debug!(
                                        "IGN cyl{} @{}° dwell={}ms",
                                        cyl,
                                        tdc_deg,
                                        ign.dwell_ms
                                    );
                                }
                            }

                            if batch_inj {
                                if let Some(inj) =
                                    compute_injection(&cfg, &sensors, airmass)
                                {
                                    injector.open(cyl);
                                    hal::timer::Stm32SystemTimer::sleep_us(
                                        (inj.pulse_ms * 1000.0) as u64,
                                    )
                                    .await;
                                    injector.close(cyl);
                                    defmt::debug!("BATCH INJ cyl{} {}ms", cyl, inj.pulse_ms);
                                }
                            }
                        }
                    }
                }
                Err(e) => {
                    defmt::warn!("Trigger error: {:?}", defmt::Debug2Format(&e));
                    decoder.reset();
                }
            }
        }

        // ── Cam pulse processing → 720° phase sync ────────────────────────
        // Each cam pulse allows the decoder to determine cam_phase and advance
        // to FullSync, enabling sequential injection to activate.
        while let Some(ts) = trigger.read_cam_timestamp() {
            match decoder.process(TriggerSignal::CamRise, ts) {
                Ok(state) => {
                    if state.sync == SyncState::FullSync {
                        defmt::debug!("FullSync: cam_phase={}", state.cam_phase);
                    }
                }
                Err(e) => {
                    defmt::warn!("Cam sync error: {:?}", defmt::Debug2Format(&e));
                }
            }
        }

        embassy_futures::yield_now().await;
    }
}

/// Carburetor control loop (fuel-fi disabled) — single-cylinder ignition only.
async fn control_loop_carb(
    cfg: EngineConfig,
    mut trigger: hal::trigger::Stm32TriggerInput,
    mut ignition: hal::ignition::Stm32IgnitionOutput,
    mut adc: hal::adc::Stm32AdcInput,
    _timer: hal::timer::Stm32SystemTimer,
) {
    use rusefi_core::sensors::{adc_to_volts, AdcChannel, IirFilter, SensorData};
    use rusefi_core::ignition::compute_ignition;

    let mut decoder = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: cfg.trigger_total_teeth,
        missing_teeth: cfg.trigger_missing_teeth,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });

    let mut clt_filter = IirFilter::new(0.1);
    let mut map_filter = IirFilter::new(0.2);

    defmt::info!("Control loop started (carburetor)");

    loop {
        while let Some(ts) = trigger.read_crank_timestamp() {
            match decoder.process(TriggerSignal::CrankRise, ts) {
                Ok(state) => {
                    if state.tooth_index == 0
                        && matches!(
                            state.sync,
                            SyncState::CrankSynced | SyncState::FullSync
                        )
                    {
                        if let Some(rpm) = state.rpm {
                            let sensors = SensorData {
                                rpm: Some(rpm),
                                ..Default::default()
                            };
                            if let Some(ign) = compute_ignition(&cfg, &sensors, 0.0) {
                                ignition.coil_charge(0);
                                hal::timer::Stm32SystemTimer::sleep_us(
                                    (ign.dwell_ms * 1000.0) as u64,
                                )
                                .await;
                                ignition.coil_fire(0);
                                defmt::debug!("IGN cyl0 rpm={}", rpm);
                            }
                        }
                    }
                }
                Err(e) => {
                    defmt::warn!("Trigger error: {:?}", defmt::Debug2Format(&e));
                    decoder.reset();
                }
            }
        }

        // Cam pulse → phase sync (improves TDC accuracy even in carb mode)
        while let Some(ts) = trigger.read_cam_timestamp() {
            let _ = decoder.process(TriggerSignal::CamRise, ts);
        }

        let _map_kpa = map_filter.update(adc_to_volts(adc.read_raw(AdcChannel::Map)) * 50.0);
        let _clt_filtered = clt_filter.update(adc_to_volts(adc.read_raw(AdcChannel::Clt)));

        embassy_futures::yield_now().await;
    }
}
