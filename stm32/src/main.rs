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

use rusefi_core::comms::rdp::{board, capability};
use rusefi_core::comms::{self, OutputChannels, TuneState};
use rusefi_core::comms::{DeviceIdentity, RdpContext, RdpServer};
#[cfg(feature = "fuel-fi")]
use rusefi_core::hal::InjectorOutput;
use rusefi_core::hal::{
    AdcInput, IgnitionOutput, PwmOutput, RelayOutput, SystemTimer, TriggerInput,
};
use rusefi_core::{
    config::EngineConfig,
    trigger::{
        missing_tooth::{MissingToothConfig, MissingToothDecoder},
        SyncEdge, SyncState, TriggerSignal,
    },
};

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

/// Latest live telemetry, published by the control loop and read by the comms task.
static OUTPUTS: BlockingMutex<CriticalSectionRawMutex, Cell<OutputChannels>> =
    BlockingMutex::new(Cell::new(OutputChannels::zeroed()));

/// Live engine configuration shared between the RDP comms task (writer) and the
/// control loop (reader). The control loop keeps a local copy and re-clones it
/// only when [`CONFIG_EPOCH`] changes, so the lock is never taken in the
/// per-tooth hot path.
static CONFIG: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<EngineConfig>>> =
    BlockingMutex::new(RefCell::new(None));

/// Bumped by the comms task after each accepted configuration edit.
static CONFIG_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Re-clone the shared config into the control loop's local copy when it has
/// changed, rebuilding the trigger decoder if the wheel geometry moved.
fn refresh_config(cfg: &mut EngineConfig, decoder: &mut MissingToothDecoder, last_epoch: &mut u32) {
    let epoch = CONFIG_EPOCH.load(Ordering::Relaxed);
    if epoch == *last_epoch {
        return;
    }
    *last_epoch = epoch;
    let new_cfg = CONFIG.lock(|c| c.borrow().clone());
    if let Some(new_cfg) = new_cfg {
        if new_cfg.trigger_total_teeth != cfg.trigger_total_teeth
            || new_cfg.trigger_missing_teeth != cfg.trigger_missing_teeth
        {
            *decoder = MissingToothDecoder::new(MissingToothConfig {
                total_teeth: new_cfg.trigger_total_teeth,
                missing_teeth: new_cfg.trigger_missing_teeth,
                engine_cycle_deg: 720.0,
                sync_edge: SyncEdge::Rise,
            });
        }
        *cfg = new_cfg;
    }
}

/// RDP identity for the board selected at compile time.
fn device_identity(cfg: &EngineConfig) -> DeviceIdentity {
    #[cfg(feature = "stm32f4")]
    let (board_id, mcu) = (board::MICRO_RUSEFI, "STM32F407");
    #[cfg(feature = "stm32f7")]
    let (board_id, mcu) = (board::PROTEUS, "STM32F767");
    #[cfg(feature = "uaefi")]
    let (board_id, mcu) = (board::UAEFI, "STM32F407");
    #[cfg(feature = "stm32f4-huge")]
    let (board_id, mcu) = (board::HUGE, "STM32F407");
    #[cfg(feature = "stm32f4-nano")]
    let (board_id, mcu) = (board::NANO, "STM32F407");

    #[allow(unused_mut)]
    let mut capabilities = capability::IGNITION;
    #[cfg(feature = "fuel-fi")]
    {
        capabilities |= capability::FUEL | capability::SEQUENTIAL;
    }

    DeviceIdentity {
        fw_version: "RustEMS 0.1.0",
        board: board_id,
        mcu,
        cylinders: cfg.firing_order.len() as u8,
        capabilities,
        // MCU UID wiring is pending; an all-zero ID marks "unset".
        device_id: [0u8; 12],
    }
}

const CONFIG_PAGE_LEN: usize = 256;
const TS_SIGNATURE: &[u8] = b"rusEFI RustEMS 2026.05";
const TS_VERSION: &[u8] = b"RustEMS 0.1.0";

// ─── Stub PWM / relay outputs ────────────────────────────────────────────────
// Placeholder drivers until board-specific PWM timer and GPIO drivers are wired.

struct StubPwmOutput {
    duty: f32,
}

impl PwmOutput for StubPwmOutput {
    fn set_duty(&mut self, duty_pct: f32) {
        self.duty = duty_pct.clamp(0.0, 100.0);
    }
    fn duty(&self) -> f32 {
        self.duty
    }
}

struct StubRelayOutput {
    on: bool,
}

impl RelayOutput for StubRelayOutput {
    fn on(&mut self) {
        self.on = true;
    }
    fn off(&mut self) {
        self.on = false;
    }
    fn is_on(&self) -> bool {
        self.on
    }
}

// ─── Embassy entry point ─────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("rusEFI STM32 firmware starting");

    let (trigger_in, producers) = hal::trigger::Stm32TriggerInput::init();

    #[cfg(feature = "stm32f4")]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(p.PE14, p.PE13, p.PE12, p.PE11);
    #[cfg(any(feature = "stm32f7", feature = "stm32f4-huge"))]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(
        p.PE4, p.PE5, p.PE6, p.PE7, p.PE8, p.PE9, p.PE10, p.PE11, p.PE12, p.PE13, p.PE14, p.PE15,
    );
    #[cfg(feature = "stm32f4-nano")]
    let ignition_out = hal::ignition::Stm32IgnitionOutput::new(p.PE14, p.PE13);
    #[cfg(feature = "uaefi")]
    let ignition_out =
        hal::ignition::Stm32IgnitionOutput::new(p.PE14, p.PE13, p.PE12, p.PE11, p.PE10, p.PE9);

    let adc_in = hal::adc::Stm32AdcInput::new(p.ADC1, p.PA0, p.PA1, p.PC3, p.PC0, p.PC1);

    let timer = hal::timer::Stm32SystemTimer::new();

    #[cfg(all(feature = "fuel-fi", feature = "stm32f4"))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(p.PB9, p.PB8, p.PD15, p.PD14);
    #[cfg(all(
        feature = "fuel-fi",
        any(feature = "stm32f7", feature = "stm32f4-huge")
    ))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(
        p.PF0, p.PF1, p.PF2, p.PF3, p.PF4, p.PF5, p.PF6, p.PF7, p.PF8, p.PF9, p.PF10, p.PF11,
    );
    #[cfg(all(feature = "fuel-fi", feature = "stm32f4-nano"))]
    let injector_out = hal::injector::Stm32InjectorOutput::new(p.PB9, p.PB8);
    #[cfg(all(feature = "fuel-fi", feature = "uaefi"))]
    let injector_out =
        hal::injector::Stm32InjectorOutput::new(p.PB9, p.PB8, p.PD15, p.PD14, p.PD13, p.PD12);

    let cfg = engine_config();
    CONFIG.lock(|c| *c.borrow_mut() = Some(cfg.clone()));
    CONFIG_EPOCH.store(1, Ordering::Relaxed);

    if let Ok(token) = crank_task(p.PA8, p.EXTI8, producers.crank) {
        spawner.spawn(token);
    }
    if let Ok(token) = cam_task(p.PA5, p.EXTI5, producers.cam) {
        spawner.spawn(token);
    }

    if let Some(uart) = hal::uart::init(p.USART1, p.PA10, p.PA9) {
        if let Ok(token) = comms_task(uart) {
            spawner.spawn(token);
        }
    }

    #[cfg(feature = "fuel-fi")]
    control_loop(cfg, trigger_in, ignition_out, adc_in, timer, injector_out).await;
    #[cfg(not(feature = "fuel-fi"))]
    control_loop_carb(cfg, trigger_in, ignition_out, adc_in, timer).await;
}

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

// ─── PC tuning comms task (dual-stack: legacy TunerStudio + RDP) ─────────────
//
// Protocol routing heuristic on the shared UART: legacy TS frames start with a
// big-endian u16 payload length — host→device requests are < 256 bytes, so the
// first byte is always 0x00. RDP frames are COBS-encoded and therefore never
// contain 0x00 except as their trailing delimiter, so a non-zero first byte
// means RDP.

#[embassy_executor::task]
async fn comms_task(mut uart: embassy_stm32::usart::BufferedUart<'static>) {
    use embassy_futures::select::{select, Either};
    use embedded_io_async::{Read, Write};
    use rusefi_device_api as wire;

    let mut config_page = [0u8; CONFIG_PAGE_LEN];
    let mut rx = [0u8; 1024];
    let mut filled = 0usize;

    // ── RDP state ──────────────────────────────────────────────────────────
    let mut ram = engine_config();
    let mut flash = engine_config();
    let defaults = engine_config();
    let identity = device_identity(&ram);
    let mut server = RdpServer::new(identity, &flash);
    let mut defrag = wire::Defragmenter::<1024>::new();
    let mut scratch = [0u8; 600];
    let mut resp = [0u8; 2048];
    let mut frame_out = [0u8; 2304];
    let mut push_seq: u16 = 0;

    defmt::info!("PC-tuning comms task started @ 115200 (TS + RDP)");

    loop {
        // Read with a periodic timeout so telemetry/event pushes keep flowing
        // even when the host is silent.
        match select(
            uart.read(&mut rx[filled..]),
            embassy_time::Timer::after_millis(10),
        )
        .await
        {
            Either::First(Ok(0)) => {}
            Either::First(Ok(n)) => filled += n,
            Either::First(Err(_)) => filled = 0,
            Either::Second(()) => {
                let outputs = OUTPUTS.lock(|c| c.get());
                let now_ms = embassy_time::Instant::now().as_millis() as u32;
                if let Some(len) = server.poll_telemetry(&outputs, now_ms, &mut resp) {
                    if let Ok(flen) = wire::encode_message(
                        wire::Flags::none(),
                        push_seq,
                        &resp[..len],
                        &mut frame_out,
                    ) {
                        push_seq = push_seq.wrapping_add(1);
                        let _ = uart.write_all(&frame_out[..flen]).await;
                    }
                }
                if let Some(len) = server.poll_event(&mut resp) {
                    if let Ok(flen) = wire::encode_message(
                        wire::Flags::none(),
                        push_seq,
                        &resp[..len],
                        &mut frame_out,
                    ) {
                        push_seq = push_seq.wrapping_add(1);
                        let _ = uart.write_all(&frame_out[..flen]).await;
                    }
                }
            }
        }

        loop {
            if filled == 0 {
                break;
            }

            if rx[0] != 0x00 {
                // ── RDP frame: consume up to the 0x00 delimiter ────────────
                let Some(zi) = rx[..filled].iter().position(|&b| b == 0) else {
                    if filled == rx.len() {
                        filled = 0;
                    }
                    break;
                };
                let mut reply: Option<(u16, usize, bool)> = None;
                if let Ok((header, payload)) = wire::decode_frame(&rx[..zi], &mut scratch) {
                    match defrag.feed(&header, payload) {
                        Ok(Some(request)) => {
                            let outputs = OUTPUTS.lock(|c| c.get());
                            let now_ms = embassy_time::Instant::now().as_millis() as u32;
                            let engine_running = outputs.rpm > 50.0;
                            // High byte of OP (LE at request[1..3]): 0x03 = Config.
                            let is_config_op = request.len() >= 3 && request[2] == 0x03;
                            let mut ctx = RdpContext {
                                ram: &mut ram,
                                flash: &flash,
                                defaults: &defaults,
                                outputs: &outputs,
                                now_ms,
                                engine_running,
                            };
                            let (rlen, actions) = server.handle(request, &mut ctx, &mut resp);
                            reply = Some((header.seq, rlen, is_config_op));
                            if actions.save {
                                flash = ram.clone();
                                // Flash persistence driver is pending; the RAM
                                // copy stays authoritative until then.
                                defmt::info!("RDP: config save requested");
                            }
                            if actions.reboot {
                                defmt::warn!("RDP: reboot requested");
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                            if actions.enter_bootloader {
                                defmt::warn!("RDP: bootloader entry requested (not wired)");
                            }
                        }
                        Ok(None) => {}
                        Err(_) => defrag.reset(),
                    }
                }
                rx.copy_within(zi + 1..filled, 0);
                filled -= zi + 1;

                if let Some((seq, rlen, is_config_op)) = reply {
                    if is_config_op {
                        // Publish the edited config to the control loop.
                        CONFIG.lock(|c| *c.borrow_mut() = Some(ram.clone()));
                        CONFIG_EPOCH.fetch_add(1, Ordering::Relaxed);
                    }
                    if rlen > 0 {
                        if let Ok(flen) = wire::encode_message(
                            wire::Flags::none(),
                            seq,
                            &resp[..rlen],
                            &mut frame_out,
                        ) {
                            let _ = uart.write_all(&frame_out[..flen]).await;
                        }
                    }
                }
            } else {
                // ── Legacy TunerStudio frame ───────────────────────────────
                match comms::decode_frame(&rx[..filled]) {
                    Ok((payload, consumed)) => {
                        let outputs = OUTPUTS.lock(|c| c.get()).to_bytes();
                        let mut state = TuneState {
                            signature: TS_SIGNATURE,
                            firmware_version: TS_VERSION,
                            config: &mut config_page,
                            outputs: &outputs,
                            burn_pending: false,
                        };
                        let mut resp_payload = [0u8; CONFIG_PAGE_LEN + 8];
                        let resp_len =
                            comms::handle_request(payload, &mut state, &mut resp_payload);
                        let burned = state.burn_pending;

                        rx.copy_within(consumed..filled, 0);
                        filled -= consumed;

                        if let Some(len) = resp_len {
                            let mut frame = [0u8; CONFIG_PAGE_LEN + 16];
                            if let Some(flen) =
                                comms::encode_frame(&resp_payload[..len], &mut frame)
                            {
                                let _ = uart.write_all(&frame[..flen]).await;
                            }
                        }
                        if burned {
                            defmt::info!("tune page burned");
                        }
                    }
                    Err(comms::FrameError::Incomplete) => break,
                    Err(_) => {
                        filled = 0;
                        break;
                    }
                }
            }
        }

        if filled == rx.len() {
            filled = 0;
        }
    }
}

// ─── Control loop (fuel injection) ───────────────────────────────────────────

#[cfg(feature = "fuel-fi")]
async fn control_loop(
    cfg: EngineConfig,
    mut trigger: hal::trigger::Stm32TriggerInput,
    mut ignition: hal::ignition::Stm32IgnitionOutput,
    mut adc: hal::adc::Stm32AdcInput,
    timer: hal::timer::Stm32SystemTimer,
    mut injector: hal::injector::Stm32InjectorOutput,
) {
    use rusefi_core::actuators::{BoostConfig, BoostController, IdleConfig, IdleController};
    use rusefi_core::config::MAX_CYLINDERS;
    use rusefi_core::engine_cycle::SequentialInjection;
    use rusefi_core::fuel::{
        compute_injection, estimate_airmass_g,
        ltft::LtftState,
        wall_wetting::{MultiCylWallWetting, WallWettingConfig},
        AccelEnrichmentConfig, AccelEnrichmentController, ClosedLoopConfig, ClosedLoopController,
        DfcoConfig, DfcoController,
    };
    use rusefi_core::ignition::{
        compute_ignition, tdc_angles_from_firing_order, OverdwellConfig, OverdwellController,
        RpmLimiter, RpmLimiterConfig,
    };
    use rusefi_core::outputs::{FanController, FanMode, FuelPumpConfig, FuelPumpController};
    use rusefi_core::protection::ProtectionMonitor;
    use rusefi_core::sensors::{
        adc_to_volts, AdcChannel, IirFilter, LambdaSensor, LambdaSensorConfig, SensorData,
    };

    // ── Live-tunable configuration (updated by the RDP comms task) ────────
    let mut cfg = cfg;
    let mut cfg_epoch = CONFIG_EPOCH.load(Ordering::Relaxed);

    // ── Decoders / sequencing ─────────────────────────────────────────────
    let mut decoder = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: cfg.trigger_total_teeth,
        missing_teeth: cfg.trigger_missing_teeth,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });
    let mut seq_inj = SequentialInjection::new(&cfg.firing_order, 90.0);
    let mut cycle_injected = [false; MAX_CYLINDERS];

    // ── IIR filters ───────────────────────────────────────────────────────
    let mut clt_filter = IirFilter::new(0.1);
    let mut iat_filter = IirFilter::new(0.1);
    let mut map_filter = IirFilter::new(0.2);
    let mut tps_filter = IirFilter::new(0.3);
    let mut oil_filter = IirFilter::new(0.05);
    let mut fuel_filter = IirFilter::new(0.05);

    // ── RPM limiter ───────────────────────────────────────────────────────
    let mut rpm_limiter = RpmLimiter::new(RpmLimiterConfig::default());

    // ── Overdwell controllers (one per cylinder) ──────────────────────────
    let mut overdwell: [OverdwellController; MAX_CYLINDERS] =
        core::array::from_fn(|_| OverdwellController::new(OverdwellConfig::default()));

    // ── Fuel pipeline controllers ─────────────────────────────────────────
    let num_cylinders = cfg.firing_order.len() as u8;
    let mut dfco = DfcoController::new(DfcoConfig::default());
    let mut accel = AccelEnrichmentController::new(AccelEnrichmentConfig::default());
    let mut closed_loop = ClosedLoopController::new(ClosedLoopConfig {
        enabled: true,
        ..ClosedLoopConfig::default()
    });
    let mut ltft = LtftState::new();
    let mut wall_wetting = MultiCylWallWetting::new(WallWettingConfig::default(), num_cylinders);
    let lambda_sensor = LambdaSensor::new(LambdaSensorConfig::default());

    // ── Output actuator controllers ───────────────────────────────────────
    let mut fuel_pump = FuelPumpController::new(FuelPumpConfig::default());
    let mut fan = FanController::default_engine();
    let mut idle = IdleController::new(IdleConfig::default_4cyl());
    let mut boost = BoostController::new(BoostConfig::default_turbo());
    let mut protection = ProtectionMonitor::new();

    // ── Stub PWM / relay outputs ──────────────────────────────────────────
    let mut iac_pwm = StubPwmOutput { duty: 0.0 };
    let mut boost_pwm = StubPwmOutput { duty: 0.0 };
    let mut fuel_pump_relay = StubRelayOutput { on: false };
    let mut fan_relay = StubRelayOutput { on: false };

    // Key-on: prime the fuel pump
    fuel_pump.on_key_on();
    fuel_pump_relay.on();

    // ── Per-cycle correction state ────────────────────────────────────────
    let mut dfco_active = false;
    let mut cl_correction = 1.0f32;
    let mut ltft_correction = 1.0f32;
    let mut accel_mult = 1.0f32;
    let knock_retard_deg = 0.0f32;

    // ── Telemetry state ───────────────────────────────────────────────────
    let mut last_adv = 0.0f32;
    let mut last_inj_ms = 0.0f32;
    let mut last_lambda = 1.0f32;
    let mut last_oil_kpa = 0.0f32;
    let mut last_fuel_pct = 0.0f32;

    // ── Time tracking ─────────────────────────────────────────────────────
    let mut last_us: u64 = 0;
    let mut actuator_tick_us: u64 = 0;

    defmt::info!("Control loop started (fuel-injection, full pipeline)");

    loop {
        // ── Pick up config edits from the comms task ──────────────────────
        refresh_config(&mut cfg, &mut decoder, &mut cfg_epoch);

        // ── Current time and dt ───────────────────────────────────────────
        let now_us = timer.now_us();
        let dt_s = if last_us > 0 {
            (now_us.saturating_sub(last_us) as f32 / 1_000_000.0).clamp(0.00005, 0.1)
        } else {
            0.001
        };
        let dt_ms = (dt_s * 1000.0) as u32;
        last_us = now_us;

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

        // Lambda, oil pressure, fuel level
        let lambda1_v = adc_to_volts(adc.read_raw(AdcChannel::Lambda1));
        let lambda_meas = lambda_sensor.voltage_to_lambda(lambda1_v).unwrap_or(1.0);
        let oil_kpa =
            oil_filter.update(adc_to_volts(adc.read_raw(AdcChannel::OilPressure)) * 140.0);
        let fuel_pct = fuel_filter.update(
            (adc_to_volts(adc.read_raw(AdcChannel::FuelLevel)) / 3.3 * 100.0).clamp(0.0, 100.0),
        );
        // ── Actuator updates (~10 ms cadence) ─────────────────────────────
        if now_us.saturating_sub(actuator_tick_us) >= 10_000 {
            actuator_tick_us = now_us;
            last_lambda = lambda_meas;
            last_oil_kpa = oil_kpa;
            last_fuel_pct = fuel_pct;

            // Build sensor snapshot for protection and actuators
            let sens_snap = SensorData {
                clt_celsius: Some(clt_c),
                iat_celsius: Some(iat_c),
                battery_volts: Some(vbatt_v),
                oil_pressure_kpa: Some(oil_kpa),
                ..Default::default()
            };

            // Protection / limp mode
            let _prot_rpm_limit = protection.update(&sens_snap, dt_ms);

            // Fuel pump relay
            let rpm_now = 0.0f32; // best guess before crank event; updated below per tooth
            if fuel_pump.update(rpm_now, dt_ms) {
                fuel_pump_relay.on();
            } else {
                fuel_pump_relay.off();
            }

            // Cooling fan relay
            match fan.update(clt_c) {
                FanMode::On | FanMode::Pwm(_) => fan_relay.on(),
                FanMode::Off => fan_relay.off(),
            }

            // IAC (idle air control) PWM
            let is_cranking = false; // updated per crank tooth
            let iac = idle.update(0.0, clt_c, tps_pct, is_cranking, dt_s * 1000.0, false);
            iac_pwm.set_duty(iac);

            // Boost / wastegate solenoid PWM (natural-aspiration default = 0%)
            let boost_duty = boost.update(map_kpa, 0.0, tps_pct, false, dt_s * 1000.0);
            boost_pwm.set_duty(boost_duty);
        }

        // ── Crank pulse processing ────────────────────────────────────────
        while let Some(ts) = trigger.read_crank_timestamp() {
            match decoder.process(TriggerSignal::CrankRise, ts) {
                Ok(state) => {
                    if state.tooth_index == 0 {
                        cycle_injected = [false; MAX_CYLINDERS];
                    }

                    let rpm = match state.rpm {
                        Some(r) if r > 50.0 => r,
                        _ => continue,
                    };

                    // Update fuel pump with live RPM
                    if fuel_pump.update(rpm, 0) {
                        fuel_pump_relay.on();
                    }

                    let spark_cut = rpm_limiter.update(rpm);

                    let sensors = SensorData {
                        rpm: Some(rpm),
                        load_pct: Some(map_kpa / 101.325 * 100.0),
                        clt_celsius: Some(clt_c),
                        iat_celsius: Some(iat_c),
                        tps_pct: Some(tps_pct),
                        map_kpa: Some(map_kpa),
                        battery_volts: Some(vbatt_v),
                        lambda1_voltage: Some(lambda1_v),
                        oil_pressure_kpa: Some(oil_kpa),
                        fuel_level_pct: Some(fuel_pct),
                        ..Default::default()
                    };

                    let airmass = estimate_airmass_g(map_kpa, cfg.displacement_cc_per_cyl, 0.85);

                    // ── Per-cycle updates at TDC reference (tooth 0) ──────
                    if state.tooth_index == 0
                        && matches!(state.sync, SyncState::CrankSynced | SyncState::FullSync)
                    {
                        // DFCO: cut fuel on deceleration (closed throttle + high RPM)
                        dfco_active = dfco.update(rpm, map_kpa, tps_pct, ts);
                        if dfco_active {
                            closed_loop.trigger_pause();
                        }

                        // Closed-loop lambda correction
                        cl_correction = closed_loop.update(rpm, clt_c, lambda_meas, dt_s);

                        // LTFT: learn long-term trim from lambda feedback
                        let load_pct = map_kpa / 101.325 * 100.0;
                        ltft.update(&sensors, 1.0);
                        ltft_correction = ltft.get_trim(rpm, load_pct);

                        // Acceleration enrichment (TPS rate-of-change)
                        accel_mult = accel.update(tps_pct, ts);

                        // Update idle controller with live RPM
                        let is_cranking_now = rpm < cfg.cranking_rpm;
                        let iac =
                            idle.update(rpm, clt_c, tps_pct, is_cranking_now, dt_s * 1000.0, false);
                        iac_pwm.set_duty(iac);

                        // Boost controller update
                        let boost_duty = boost.update(map_kpa, rpm, tps_pct, false, dt_s * 1000.0);
                        boost_pwm.set_duty(boost_duty);

                        defmt::debug!(
                            "Cycle: rpm={} dfco={} cl={} ltft={} accel={}",
                            rpm,
                            dfco_active,
                            cl_correction,
                            ltft_correction,
                            accel_mult
                        );

                        // ── Ignition + batch injection at TDC ─────────────
                        let tdc_angles = tdc_angles_from_firing_order(&cfg.firing_order);
                        let batch_inj = !seq_inj.is_sequential();

                        for (i, &cyl) in cfg.firing_order.iter().enumerate() {
                            let tdc_deg = tdc_angles[i];

                            // Ignition
                            if !spark_cut {
                                if let Some(ign) = compute_ignition(&cfg, &sensors, tdc_deg) {
                                    // Apply knock retard
                                    let effective_advance = ign.advance_deg - knock_retard_deg;
                                    last_adv = effective_advance;

                                    let od = &mut overdwell[cyl as usize % MAX_CYLINDERS];
                                    od.start_charge(ts);
                                    ignition.coil_charge(cyl);
                                    hal::timer::Stm32SystemTimer::sleep_us(
                                        (ign.dwell_ms * 1000.0) as u64,
                                    )
                                    .await;
                                    od.end_charge();
                                    ignition.coil_fire(cyl);
                                    defmt::debug!(
                                        "IGN cyl{} adv={}deg (retard={}deg) dwell={}ms",
                                        cyl,
                                        effective_advance,
                                        knock_retard_deg,
                                        ign.dwell_ms
                                    );
                                }
                            }

                            // Batch injection (when not in sequential mode)
                            if batch_inj && !dfco_active {
                                if let Some(inj) = compute_injection(&cfg, &sensors, airmass) {
                                    let corrected_mass = inj.fuel_mass_g
                                        * cl_correction
                                        * ltft_correction
                                        * accel_mult;
                                    let ww_mass =
                                        wall_wetting.compensate(cyl, corrected_mass, clt_c, dt_s);
                                    let flow_g_s = cfg.injector_flow_cc_per_min * 0.755 / 60.0;
                                    let deadtime = inj.pulse_ms - inj.open_ms;
                                    let pulse_ms =
                                        (ww_mass / flow_g_s * 1000.0 + deadtime).max(0.0);
                                    last_inj_ms = pulse_ms;

                                    injector.open(cyl);
                                    hal::timer::Stm32SystemTimer::sleep_us(
                                        (pulse_ms * 1000.0) as u64,
                                    )
                                    .await;
                                    injector.close(cyl);
                                    defmt::debug!("BATCH INJ cyl{} {}ms", cyl, pulse_ms);
                                }
                            }
                        }
                    }

                    // ── Sequential injection (FullSync only) ─────────────
                    if let Some(cyl) = seq_inj.update(&state) {
                        let ci = cyl as usize;
                        if ci < cycle_injected.len() && !cycle_injected[ci] && !dfco_active {
                            cycle_injected[ci] = true;
                            if let Some(inj) = compute_injection(&cfg, &sensors, airmass) {
                                let corrected_mass =
                                    inj.fuel_mass_g * cl_correction * ltft_correction * accel_mult;
                                let ww_mass =
                                    wall_wetting.compensate(cyl, corrected_mass, clt_c, dt_s);
                                let flow_g_s = cfg.injector_flow_cc_per_min * 0.755 / 60.0;
                                let deadtime = inj.pulse_ms - inj.open_ms;
                                let pulse_ms = (ww_mass / flow_g_s * 1000.0 + deadtime).max(0.0);
                                last_inj_ms = pulse_ms;

                                injector.open(cyl);
                                hal::timer::Stm32SystemTimer::sleep_us((pulse_ms * 1000.0) as u64)
                                    .await;
                                injector.close(cyl);
                                defmt::debug!("SEQ INJ cyl{} {}ms", cyl, pulse_ms);
                            }
                        }
                    }

                    // Publish telemetry
                    OUTPUTS.lock(|c| {
                        c.set(OutputChannels {
                            rpm,
                            clt_c,
                            iat_c,
                            map_kpa,
                            tps_pct,
                            battery_v: vbatt_v,
                            lambda: last_lambda,
                            inj_pulse_ms: last_inj_ms,
                            advance_deg: last_adv,
                            spark_cut,
                            sequential: seq_inj.is_sequential(),
                            dfco_active,
                            knock_retard_deg,
                            ltft_correction,
                            cl_correction,
                            oil_pressure_kpa: last_oil_kpa,
                            fuel_level_pct: last_fuel_pct,
                            fuel_pump_on: fuel_pump_relay.is_on(),
                            fan_on: fan_relay.is_on(),
                            limp_active: protection.is_protection_active(),
                            iac_duty_pct: iac_pwm.duty(),
                            boost_duty_pct: boost_pwm.duty(),
                        })
                    });
                }
                Err(e) => {
                    defmt::warn!("Trigger error: {:?}", defmt::Debug2Format(&e));
                    decoder.reset();
                }
            }
        }

        // ── Cam pulse processing → 720° phase sync ────────────────────────
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

// ─── Carburetor control loop ──────────────────────────────────────────────────

#[allow(dead_code)]
async fn control_loop_carb(
    cfg: EngineConfig,
    mut trigger: hal::trigger::Stm32TriggerInput,
    mut ignition: hal::ignition::Stm32IgnitionOutput,
    mut adc: hal::adc::Stm32AdcInput,
    timer: hal::timer::Stm32SystemTimer,
) {
    use rusefi_core::config::MAX_CYLINDERS;
    use rusefi_core::ignition::{compute_ignition, OverdwellConfig, OverdwellController};
    use rusefi_core::outputs::{FanController, FanMode, FuelPumpConfig, FuelPumpController};
    use rusefi_core::protection::ProtectionMonitor;
    use rusefi_core::sensors::{adc_to_volts, AdcChannel, IirFilter, SensorData};

    let mut cfg = cfg;
    let mut cfg_epoch = CONFIG_EPOCH.load(Ordering::Relaxed);

    let mut decoder = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: cfg.trigger_total_teeth,
        missing_teeth: cfg.trigger_missing_teeth,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });

    let mut clt_filter = IirFilter::new(0.1);
    let mut map_filter = IirFilter::new(0.2);
    let mut overdwell: [OverdwellController; MAX_CYLINDERS] =
        core::array::from_fn(|_| OverdwellController::new(OverdwellConfig::default()));

    let mut fuel_pump = FuelPumpController::new(FuelPumpConfig::default());
    let mut fan = FanController::default_engine();
    let mut protection = ProtectionMonitor::new();

    let mut fuel_pump_relay = StubRelayOutput { on: false };
    let mut fan_relay = StubRelayOutput { on: false };

    let mut last_us: u64 = 0;
    fuel_pump.on_key_on();
    fuel_pump_relay.on();

    defmt::info!("Control loop started (carburetor)");

    loop {
        refresh_config(&mut cfg, &mut decoder, &mut cfg_epoch);

        let now_us = timer.now_us();
        let dt_ms = if last_us > 0 {
            (now_us.saturating_sub(last_us) / 1_000) as u32
        } else {
            1
        };
        last_us = now_us;

        let clt_c = clt_filter.update((adc_to_volts(adc.read_raw(AdcChannel::Clt)) - 0.5) / 0.01);
        let map_kpa = map_filter.update(adc_to_volts(adc.read_raw(AdcChannel::Map)) * 50.0);

        let sens = SensorData {
            clt_celsius: Some(clt_c),
            map_kpa: Some(map_kpa),
            ..Default::default()
        };
        let _prot = protection.update(&sens, dt_ms);

        if fuel_pump.update(0.0, dt_ms) {
            fuel_pump_relay.on();
        } else {
            fuel_pump_relay.off();
        }
        match fan.update(clt_c) {
            FanMode::On | FanMode::Pwm(_) => fan_relay.on(),
            FanMode::Off => fan_relay.off(),
        }

        while let Some(ts) = trigger.read_crank_timestamp() {
            match decoder.process(TriggerSignal::CrankRise, ts) {
                Ok(state) => {
                    if state.tooth_index == 0
                        && matches!(state.sync, SyncState::CrankSynced | SyncState::FullSync)
                    {
                        if let Some(rpm) = state.rpm {
                            if fuel_pump.update(rpm, 0) {
                                fuel_pump_relay.on();
                            }
                            let sensors = SensorData {
                                rpm: Some(rpm),
                                clt_celsius: Some(clt_c),
                                ..Default::default()
                            };
                            if let Some(ign) = compute_ignition(&cfg, &sensors, 0.0) {
                                let od = &mut overdwell[0];
                                od.start_charge(ts);
                                ignition.coil_charge(0);
                                hal::timer::Stm32SystemTimer::sleep_us(
                                    (ign.dwell_ms * 1000.0) as u64,
                                )
                                .await;
                                od.end_charge();
                                ignition.coil_fire(0);
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

        while let Some(ts) = trigger.read_cam_timestamp() {
            let _ = decoder.process(TriggerSignal::CamRise, ts);
        }

        embassy_futures::yield_now().await;
    }
}
