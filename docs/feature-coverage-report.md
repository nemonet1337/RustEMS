# Feature Coverage Report

Generated: 2026-06-11

Comparison of rusEFI C++ reference implementation (`old-src/`) vs. Rust implementation.

---

## HAL Trait Implementation Status

| Trait | hal-sim | hal-stm32-common | hal-microrusefi | hal-proteus | hal-uaefi | hal-huge | hal-nano |
|-------|---------|-----------------|-----------------|-------------|-----------|----------|----------|
| `TriggerInput` | ✅ Full | Stub | Stub | Stub | Stub | Stub | Stub |
| `IgnitionOutput` | ✅ Full | Stub | Partial | Partial | Partial | Partial | Partial |
| `AdcInput` | ✅ Full | Stub | Stub | Stub | Stub | Stub | Stub |
| `SystemTimer` | ✅ Full | Stub | Stub | Stub | Stub | Stub | Stub |
| `InjectorOutput` | ✅ Full | Stub | Partial | Partial | Partial | Partial | Partial |
| `CanBus` | ✅ Full | Stub | Stub | Stub | Stub | Stub | Stub |
| `UartPort` | ✅ Full | Stub | Partial | Partial | Partial | Partial | Partial |
| `PwmOutput` | ✅ Full | — | Stub\* | Stub\* | Stub\* | Stub\* | Stub\* |
| `RelayOutput` | ✅ Full | — | Stub\* | Stub\* | Stub\* | Stub\* | Stub\* |

\* Inline stub in stm32/main.rs; board HAL crates do not yet have dedicated drivers.

---

## Control Logic Feature Matrix

### Engine Cycle & Trigger

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Missing-tooth decoder (36-1, 60-2, …) | ✅ | ✅ | 7 presets |
| Hall-effect EXTI trigger input | ✅ | ✅ | embassy EXTI |
| VR (variable reluctance) trigger | ✅ | ❌ | Pending |
| 4-stroke cam sync | ✅ | Partial | Decoder stub |
| Instant RPM | ✅ | ✅ | IIR-smoothed |
| Trigger noise filter | ✅ | ✅ | Debounce + glitch |
| Stall detection | ✅ | ✅ | |

### Ignition

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Speed-density ignition table | ✅ | ✅ | |
| Alpha-N ignition table | ✅ | ✅ | |
| Dwell control | ✅ | ✅ | |
| Overdwell protection | ✅ | ✅ | Per-cylinder |
| Knock retard | ✅ | ✅ | Wired in loop |
| RPM limiter (hard/soft cut) | ✅ | ✅ | |
| Multi-spark | ✅ | ✅ | |
| Launch control (ignition side) | ✅ | Partial | Speed input not wired |
| Traction control | ✅ | Partial | **New**: `TractionController` (slip → progressive retard → cut); wheel-speed inputs not wired |

### Fuel Injection

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Speed-density VE table | ✅ | ✅ | |
| Alpha-N fuel table | ✅ | ✅ | |
| MAF-based air mass | ✅ | ✅ | |
| Batch injection | ✅ | ✅ | |
| Sequential injection | ✅ | Partial | Needs cam sync |
| Cranking enrichment | ✅ | ✅ | |
| CLT fuel correction | ✅ | ✅ | |
| IAT fuel correction | ✅ | ✅ | |
| DFCO (decel fuel cut-off) | ✅ | ✅ | Wired in loop |
| Closed-loop lambda | ✅ | ✅ | Wired in loop |
| Long-term fuel trim (LTFT) | ✅ | ✅ | Wired in loop |
| Acceleration enrichment | ✅ | ✅ | Wired in loop |
| Wall wetting (Aquino model) | ✅ | ✅ | Per-cylinder |
| Flex fuel | ✅ | ✅ | |
| Small pulse correction | ✅ | ✅ | |

### Sensors

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| CLT (Steinhart-Hart NTC) | ✅ | ✅ | |
| IAT (Steinhart-Hart NTC) | ✅ | ✅ | |
| TPS (linear voltage) | ✅ | ✅ | |
| MAP (linear voltage) | ✅ | ✅ | |
| Vbatt (linear voltage) | ✅ | ✅ | |
| MAF (voltage curve) | ✅ | ✅ | |
| Wideband lambda (analog) | ✅ | ✅ | |
| Wideband heater control | ✅ | Partial | **New**: `HeaterController` (condensation / ramp / battery-compensated hold); PWM output not wired |
| Narrowband lambda (analog) | ✅ | ✅ | |
| Oil pressure (linear) | ✅ | ✅ | |
| Fuel level (linear) | ✅ | ✅ | |
| IIR sensor filtering | ✅ | ✅ | |
| Flex fuel sensor (frequency) | ✅ | Partial | ADC path only |
| Redundant (dual) TPS plausibility | ✅ | ✅ | **New**: in `EtbController` |

### Actuators & Outputs

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Idle air control (IAC PWM) | ✅ | ✅ | Wired with PID |
| Boost control (wastegate PWM) | ✅ | ✅ | Open/closed loop |
| VVT solenoid | ✅ | Partial | Controller exists, not wired to PWM output |
| ETB (electronic throttle) | ✅ | Partial | **New**: `EtbController` (PID + dual-TPS plausibility + limp fail-safe); H-bridge HAL driver pending |
| Fuel pump relay | ✅ | ✅ | Wired in loop |
| Radiator fan relay | ✅ | ✅ | Wired with hysteresis |
| Tachometer output | ✅ | ✅ | |
| General-purpose PWM | ✅ | ✅ | AuxPwmController |

### Engine Management

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Engine protection (overheat/oil) | ✅ | ✅ | Limp mode |
| Engine start/stop sequencing | ✅ | ✅ | |
| Shutdown sequencer | ✅ | ✅ | |
| TCU (transmission) | ✅ | ✅ | Basic gear logic |
| Bootloader / OTA prep | ✅ | Partial | Module exists |

### Communication & Tuning

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| TunerStudio binary protocol | ✅ | ✅ | Full framing + commands (legacy, to be retired) |
| TunerStudio output channels | ✅ | ✅ | 20-byte layout + extended |
| RDP wire layer (COBS/CRC16/framing/defrag/CBOR) | — | ✅ | `device-api`, no_std, fragmentation both directions |
| RDP self-describing catalogs | — | ✅ | Static `ParamMeta`/`TableMeta`/`ChannelMeta` + `schema_hash` |
| RDP device handler (all opcodes) | — | ✅ | System / Descriptor / Config / Telemetry / Control / Diagnostics |
| RDP config staging (RAM→flash) | — | ✅ | Save/Discard/ResetDefaults/Status + dirty/CRC (flash driver pending) |
| RDP telemetry subscription streams | — | ✅ | Rate-limited packed push frames, multi-stream |
| RDP faults (DTC) + async events | — | ✅ | FaultSet/Cleared, Knock, ProtectionCut, SyncState, … |
| RDP control (bench/override/calibrate) | — | ✅ | Engine-running gating, timeout fail-safe |
| RDP on firmware UART | — | ✅ | stm32 comms task: TS+RDP dual-stack, config epoch → control loop |
| RDP on PC simulator | — | ✅ | `rusefi-sim --serve <port>` (TCP) |
| RDP host client + CLI | — | ✅ | `client/src/rdp.rs`, `rusefi rdp …` subcommands |
| CAN OBD2 responder | ✅ | Partial | Frames built; bus not wired |
| CAN dashboard (Haltech/Honda/BMW) | ✅ | Partial | Frames built; bus not wired |
| UART/Serial transport | ✅ | ✅ | |
| USB CDC-ACM | ✅ | ❌ | Pending |
| Bluetooth (SPP / BLE GATT) | ✅ | ❌ | Pending |

---

## Performance Notes

- Table axis lookup (`maps/interpolation.rs`) now uses binary search
  (O(log N)) with `#[inline]`, replacing the linear scan in the per-tooth
  ignition/injection hot path.
- The stm32 control loop shares configuration with the comms task through an
  epoch counter: the per-tooth path never takes a lock; the config is
  re-cloned only after an accepted edit (trigger decoder rebuilt only when
  wheel geometry changes).
- RDP telemetry uses subscription-based packed integer frames instead of
  polling a fixed f32 block, cutting telemetry bandwidth roughly in half for
  typical channel sets.
- The simulator CSV writer no longer allocates a `String` per trigger event.

---

## Test & Build Verification (2026-06-11)

| Package | Command | Result |
|---------|---------|--------|
| `rusefi-core` (cyl-4, fuel-fi) | `cargo test --lib` | ✅ 347 passed |
| `rusefi-core` (cyl-12, fuel-fi) | `cargo test --lib` | ✅ 346 passed |
| `rusefi-core` (cyl-1, fuel-carb) | `cargo test --lib` | ✅ 280 passed |
| `rusefi-device-api` | `cargo test` | ✅ 36 passed |
| `rusefi-protocol` | `cargo test --lib` | ✅ 13 passed |
| `rusefi-client` | `cargo test` | ✅ 20 passed (incl. RDP duplex round-trips) |
| `rusefi-sim` (cyl-4, fuel-fi) | `cargo build` / `cargo test` | ✅ 24 passed |
| `rusefi-cli` | `cargo build` | ✅ |
| `rusefi-stm32` (all 5 boards) | `cargo build-arm …` | ✅ stm32f4 / stm32f7 / uaefi / stm32f4-huge / stm32f4-nano |
| clippy (`client`, `cli`, `device-api`) | `cargo clippy` | ✅ no warnings (workspace lints enforced) |
| RDP end-to-end | sim `--serve` + `rusefi rdp …` | ✅ hello/params/set/get/table/status/save/watch/faults |

---

## Gap Summary

### Blockers for Hardware Bring-up

1. **STM32 PWM timer driver** — IAC, boost, ETB, heater control stub out until a real TIM channel driver is wired
2. **STM32 GPIO relay driver** — Fuel pump and fan relay stub until Output pin driver added
3. **STM32 flash driver** — RDP `ConfigSave` persists to a RAM snapshot until flash write is wired

### Notable Functional Gaps vs C++

- Sequential injection requires cam sync (decoder infrastructure exists)
- CAN bus not yet driven (frame builders complete)
- VR trigger not supported (Hall-effect only)
- USB CDC-ACM / Bluetooth transports pending (RDP currently on UART / TCP)
- ETB / traction / heater controllers implemented but not yet driven by real
  actuator HAL drivers
