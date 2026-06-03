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
| CAN communication (OBD2 responder) | Partial | `can::obd2` module exists with frame builders; not wired to CAN bus |
| CAN dashboard broadcast | Partial | `can::dash` module with Haltech/Honda/BMW frames; not wired |
| STM32 CAN peripheral driver | Pending | HAL crate stubs exist; embassy-stm32 CAN driver not integrated |
| USB CDC-ACM transport | Pending | Would allow tuning over USB instead of UART |
| Bluetooth transport | Pending | Low-power BLE option for mobile tuning apps |
| RDP handler in comms task | Pending | `handle_rdp` implemented; not yet integrated alongside TunerStudio comms |

## Medium

| Item | Status | Notes |
|------|--------|-------|
| ETB (Electronic Throttle Body) control | Pending | No ETB controller module yet |
| Variable reluctance trigger input | Pending | Only Hall-effect EXTI trigger implemented |
| Knock sensor ADC channel | Pending | `KnockController` wired but raw knock ADC not read |
| VVT solenoid PWM output | Partial | `VvtController` exists; not wired to PWM output |
| Flex fuel sensor (frequency input) | Partial | `FlexFuelSensor` exists; not wired to timer capture |
| STM32 ADC driver (all channels) | Partial | `Stm32AdcInput` stub; only 5 channels mapped |
| Oil pressure & fuel level ADC | Pending | Channels defined; STM32 ADC stub returns 0 |
| Lambda wideband heater control | Pending | Heater duty PWM not implemented |

## Low

| Item | Status | Notes |
|------|--------|-------|
| Launch control (speed sensor) | Partial | `LaunchControl` in ignition; speed input not wired |
| Traction control | Pending | No wheel-speed input or torque reduction logic |
| Lua scripting engine | Pending | No scripting runtime for custom logic |
| Over-the-air firmware update | Partial | `bootloader` module exists; OTA transport not wired |
| TunerStudio INI generation | Partial | `codegen` crate exists; not all params covered |
| Limp-home mode actuator cuts | Partial | `ProtectionMonitor` sets limp flag; ETB/boost cuts not wired |

## Completed (this session)

- [x] `PwmOutput` and `RelayOutput` HAL traits
- [x] `SimPwmOutput` and `SimRelayOutput` (hal-sim)
- [x] Expanded ADC channels: MAF, fuel level, oil pressure, lambda1, lambda2
- [x] Extended `OutputChannels` telemetry fields
- [x] Control loop wiring: IdleController, BoostController, DfcoController
- [x] Control loop wiring: ClosedLoopController, LtftState, AccelEnrichmentController
- [x] Control loop wiring: KnockController, OverdwellController, MultiCylWallWetting
- [x] Control loop wiring: FuelPumpController, FanController, ProtectionMonitor
- [x] `params` module: ParamId catalog, get/set scalar/table/array
- [x] `comms::rdp` module: RDP device-side handler (all opcodes)
- [x] Simulator updated with all controller wiring and CSV telemetry
