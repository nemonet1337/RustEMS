# Feature Coverage TODO

Priority-ordered list of remaining implementation items.

## Critical

| Item | Status | Notes |
|------|--------|-------|
| HAL STM32 PWM timer driver | Pending | `StubPwmOutput` used in stm32/main.rs; replace with embassy-stm32 TIM driver |
| HAL STM32 GPIO relay driver | Pending | `StubRelayOutput` used; replace with embassy-stm32 Output pin |
| 4-stroke full cam sync | Pending | Sequential injection requires cam sync; decoder stubs exist |
| Sequential injection end-to-end | Partial | `SeqInjController` wired in control loop; needs cam sync to activate |

## High

| Item | Status | Notes |
|------|--------|-------|
| RDP device-side handler (all opcodes) | **Done** | `engine-core/comms/rdp.rs`: System/Descriptor/Config/Telemetry/Control/Diagnostics |
| RDP handler in comms task | **Done** | stm32 `comms_task` runs TS + RDP dual-stack on one UART (first-byte routing); config edits reach the control loop via `CONFIG_EPOCH` |
| RDP telemetry subscription streams | **Done** | `comms/telemetry.rs` — channel catalog + rate-limited packed push frames |
| RDP host client + CLI | **Done** | `client/src/rdp.rs`, `rusefi rdp ...` subcommands; sim serves RDP over TCP (`--serve`) |
| CAN communication (OBD2 responder) | Partial | `can::obd2` module exists with frame builders; not wired to CAN bus |
| CAN dashboard broadcast | Partial | `can::dash` module with Haltech/Honda/BMW frames; not wired |
| STM32 CAN peripheral driver | Pending | HAL crate stubs exist; embassy-stm32 CAN driver not integrated |
| USB CDC-ACM transport | Pending | Would allow tuning over USB instead of UART |
| Bluetooth transport | Pending | Low-power BLE option for mobile tuning apps |

## Medium

| Item | Status | Notes |
|------|--------|-------|
| ETB (Electronic Throttle Body) control | Partial | `actuators::etb::EtbController` implemented (PID + dual-TPS plausibility + limp fail-safe); H-bridge PWM HAL driver pending |
| Lambda wideband heater control | Partial | `sensors::heater::HeaterController` implemented (condensation / ramp / battery-compensated hold); PWM output wiring pending |
| Variable reluctance trigger input | Pending | Only Hall-effect EXTI trigger implemented |
| Knock sensor ADC channel | Pending | `KnockController` wired but raw knock ADC not read |
| VVT solenoid PWM output | Partial | `VvtController` exists; not wired to PWM output |
| Flex fuel sensor (frequency input) | Partial | `FlexFuelSensor` exists; not wired to timer capture |
| STM32 ADC driver (all channels) | Partial | `Stm32AdcInput` stub; only 5 channels mapped |
| Oil pressure & fuel level ADC | Pending | Channels defined; STM32 ADC stub returns 0 |

## Low

| Item | Status | Notes |
|------|--------|-------|
| Launch control (speed sensor) | Partial | `LaunchControl` in ignition; speed input not wired |
| Traction control | Partial | `traction::TractionController` implemented (slip → progressive retard → spark cut); wheel-speed inputs not wired |
| Lua scripting engine | Pending | No scripting runtime for custom logic |
| Over-the-air firmware update | Partial | `bootloader` module exists; OTA transport not wired |
| TunerStudio INI generation | Partial | `codegen` crate exists; superseded by the RDP self-describing catalog for new clients |
| Limp-home mode actuator cuts | Partial | `ProtectionMonitor` sets limp flag; ETB/boost cuts not wired |

## Completed (this session)

- [x] RDP full device-side handler (`comms/rdp.rs`): Hello/Ping/Reboot/EnterBootloader,
      paged Descriptor catalogs (params / tables / telemetry + categories + schema_hash),
      ParamGet/Set with validation (range / read-only / engine-stopped-only → Busy),
      TableGet/SetCell/SetAxis, ConfigSave/Discard/ResetDefaults/Status
      (RAM staging + dirty/CRC tracking), Subscribe/Unsubscribe/ReadOnce +
      packed telemetry push, BenchTest/SetOverride/ClearOverride/Calibrate,
      GetFaults/ClearFaults + async event push
- [x] `device-api`: Control/Diagnostics/ReadOnce/Event CBOR bodies,
      sender-side fragmentation (`encode_message`), minicbor re-export
- [x] `params.rs`: static `ParamMeta`/`TableMeta` catalogs, `TableId` space,
      whole-table accessors, `catalog_hash` / `config_crc` (FNV-1a)
- [x] `comms/telemetry.rs`: channel catalog + subscription stream manager
- [x] `comms/faults.rs`: structured fault store (DTC) + event queue
- [x] `comms/control.rs`: overrides (timeout fail-safe), bench tests, calibrations
- [x] stm32: TS+RDP dual-stack comms task, shared `CONFIG` / `CONFIG_EPOCH`
      with lock-free hot path, trigger decoder rebuild on geometry change
- [x] sim: RDP TCP serve mode (`--serve <port>`, synthetic engine model)
- [x] client/cli: `RdpClient` (framing, defrag, catalogs, telemetry decode)
      + `rusefi rdp` subcommands
- [x] `actuators/etb.rs`: ETB PID controller with dual-TPS plausibility & limp
- [x] `traction.rs`: wheel-slip traction control
- [x] `sensors/heater.rs`: wideband heater three-phase controller
- [x] Performance: binary-search axis lookup in `maps/interpolation.rs`
      (+ `#[inline]`), epoch-based config sharing in the stm32 control loop

## Completed (previous sessions)

- [x] `PwmOutput` and `RelayOutput` HAL traits; `SimPwmOutput` / `SimRelayOutput`
- [x] Expanded ADC channels: MAF, fuel level, oil pressure, lambda1, lambda2
- [x] Extended `OutputChannels` telemetry fields
- [x] Control loop wiring: Idle, Boost, DFCO, ClosedLoop, LTFT, AccelEnrichment,
      Knock, Overdwell, MultiCylWallWetting, FuelPump, Fan, Protection
- [x] `params` module: ParamId catalog, get/set scalar/table/array
- [x] `device-api` wire layer: COBS, CRC16, framing, defragmenter, CBOR codec
- [x] Simulator updated with all controller wiring and CSV telemetry
