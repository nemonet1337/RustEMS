# Feature Coverage Report

Generated: 2026-06-03

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
| Narrowband lambda (analog) | ✅ | ✅ | |
| Oil pressure (linear) | ✅ | ✅ | |
| Fuel level (linear) | ✅ | ✅ | |
| IIR sensor filtering | ✅ | ✅ | |
| Flex fuel sensor (frequency) | ✅ | Partial | ADC path only |

### Actuators & Outputs

| Feature | C++ rusEFI | Rust RustEMS | Notes |
|---------|-----------|-------------|-------|
| Idle air control (IAC PWM) | ✅ | ✅ | Wired with PID |
| Boost control (wastegate PWM) | ✅ | ✅ | Open/closed loop |
| VVT solenoid | ✅ | Partial | Controller exists, not wired to PWM output |
| ETB (electronic throttle) | ✅ | ❌ | Not implemented |
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
| TunerStudio binary protocol | ✅ | ✅ | Full framing + commands |
| TunerStudio output channels | ✅ | ✅ | 20-byte layout + extended |
| RDP parameter catalog | — | ✅ | New: type-safe u16 accessors |
| RDP device handler | — | ✅ | New: all opcodes implemented |
| CAN OBD2 responder | ✅ | Partial | Frames built; bus not wired |
| CAN dashboard (Haltech/Honda/BMW) | ✅ | Partial | Frames built; bus not wired |
| UART/Serial transport | ✅ | ✅ | |
| USB CDC-ACM | ✅ | ❌ | Pending |

---

## Build Verification

| Package | Command | Result |
|---------|---------|--------|
| `rusefi-core` (cyl-4, fuel-fi) | `cargo check` | ✅ |
| `rusefi-core` (cyl-4, fuel-fi) | `cargo test --lib` (299 tests) | ✅ |
| `rusefi-sim` (cyl-4, fuel-fi) | `cargo build` | ✅ (4 minor warnings) |
| `rusefi-stm32` (stm32f4, cyl-4, fuel-fi) | `cargo check --target thumbv7em-none-eabihf` | N/A (ARM toolchain not installed in dev env) |
| `rusefi-protocol` | `cargo test --lib` | ✅ |
| `rusefi-client` | `cargo test --lib` | ✅ |

---

## Gap Summary

### Blockers for Hardware Bring-up

1. **STM32 PWM timer driver** — IAC and boost control stub out until a real TIM channel driver is wired
2. **STM32 GPIO relay driver** — Fuel pump and fan relay stub until Output pin driver added
3. **ARM cross-compilation toolchain** — `thumbv7em-none-eabihf` target needed for full firmware check

### Notable Functional Gaps vs C++

- No ETB control (throttle-by-wire)
- Sequential injection requires cam sync (decoder infrastructure exists)
- CAN bus not yet driven (frame builders complete)
- VR trigger not supported (Hall-effect only)
