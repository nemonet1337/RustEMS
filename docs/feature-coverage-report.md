# rusEFI Rust 機能網羅性レポート

生成日時: 2026年5月10日

## 概要

本レポートは、既存のC++/Java実装（old-src）と新Rust実装の機能比較、およびRustクレート間の実装一貫性を分析したものです。

---

## 1. ワークスペース構成

### 1.1 クレート一覧

| クレート名 | 説明 | 依存先 |
|-----------|------|--------|
| `engine-core` | no_std制御ロジックコア | - |
| `hal-sim` | PCシミュレータHAL | engine-core |
| `hal-stm32-common` | STM32共通HAL（トレイト/スタブ） | engine-core |
| `hal-microrusefi` | microRusEFIボードHAL | hal-stm32-common |
| `hal-proteus` | ProteusボードHAL | hal-stm32-common |
| `hal-nano` | NanoボードHAL | hal-stm32-common |
| `hal-uaefi` | UAEFIボードHAL | hal-stm32-common |
| `hal-huge` | HugeボードHAL | hal-stm32-common |
| `protocol` | バイナリプロトコル実装 | - |
| `client` | 高レベルECUクライアント | protocol |
| `cli` | コマンドラインツール | protocol, client |
| `codegen` | 設定コード生成 | - |
| `sim` | PCシミュレータバイナリ | engine-core, hal-sim |
| `stm32` | STM32ファームウェアバイナリ | engine-core, 各HAL |

---

## 2. 機能比較マトリックス

### 2.1 コア制御機能

| 機能 | old-src (C++) | engine-core | 実装状況 | 備考 |
|------|---------------|-------------|----------|------|
| **Trigger Wheel Decoder** | | | | |
| Missing-tooth detection | ✅ | ✅ | 完了 | 36-1, 60-2, 12-1, 4-1, 24-1, 24-2, 36-2対応 |
| Cam phase sync (型定義) | ✅ | ✅ | 完了 | `TriggerState.cam_phase`, `CyclePosition` 実装済 |
| Cam phase sync (制御ループ統合) | ✅ | 🟡 | 部分 | `stm32/main.rs` の cam 処理が TODO のまま |
| Instant RPM | ✅ | ❌ | 未実装 | 瞬間RPM計算 |
| **Ignition Control** | | | | |
| Dwell control | ✅ | ✅ | 完了 | 基本実装済 |
| Spark angle calc | ✅ | ✅ | 完了 | `compute_ignition()` |
| Multi-spark | ✅ | ✅ | 完了 | `MultiSparkController` — RPMに応じたリニアスケーリング |
| Multi-coil (seq) | ✅ | 🟡 | 部分 | バッチ点火のみ制御ループ統合済。`SequentialController` は実装済 |
| **Fuel Injection** | | | | |
| Injection pulse | ✅ | ✅ | 完了 | `compute_injection()` |
| Wall wetting compensation | ✅ | ✅ | 完了 | `WallWettingController` — Aquinoモデル、libm::expf使用 |
| Sequential injection (ロジック) | ✅ | ✅ | 完了 | `SequentialController`, `SequentialInjection` 実装済 |
| Sequential injection (制御ループ統合) | ✅ | 🟡 | 部分 | stm32/main.rs はバッチモードのまま。cam sync待ち |
| Airmass models | | | | |
| - Speed Density | ✅ | 🟡 | 部分 | 基本線形のみ |
| - MAF | ✅ | ❌ | 未実装 | MAFセンサー対応 |
| - Alpha-N | ✅ | ❌ | 未実装 | スロットル角ベース |
| **Actuators** | | | | |
| Idle control | ✅ | ✅ | 完了 | PID制御実装済 |
| Boost control | ✅ | ✅ | 完了 | 基本実装済 |
| VVT control | ✅ | ✅ | 完了 | Dual VVT対応 |
| Electronic throttle | ✅ | ❌ | 未実装 | ETB未実装 |
| **Engine Cycle** | | | | |
| 720° tracking (型定義) | ✅ | ✅ | 完了 | `engine_cycle/mod.rs` に `CylinderState`, TDC offset計算実装済 |
| Cylinder state (制御ループ統合) | ✅ | 🟡 | 部分 | stm32/main.rs でcam sync未統合 |
| Firing order | ✅ | ✅ | 完了 | 基本実装済 |

### 2.2 センサー機能

| 機能 | old-src (C++) | engine-core | 実装状況 | 備考 |
|------|---------------|-------------|----------|------|
| **ADC Channels** | | | | |
| CLT (Coolant) | ✅ | ✅ | 完了 | 12bit ADC |
| IAT (Intake Air) | ✅ | ✅ | 完了 | 12bit ADC |
| TPS (Throttle) | ✅ | ✅ | 完了 | 12bit ADC |
| MAP (Manifold) | ✅ | ✅ | 完了 | 12bit ADC |
| Battery voltage | ✅ | ✅ | 完了 | 分圧補正 |
| MAF | ✅ | ❌ | 未実装 | マスエアフロー |
| Lambda/O2 | ✅ | ❌ | 未実装 | 空燃比センサー |
| Oil pressure | ✅ | ❌ | 未実装 | 油圧センサー |
| **Sensor Features** | | | | |
| Redundant TPS | ✅ | ❌ | 未実装 | デュアルTPS |
| SENT protocol | ✅ | ❌ | 未実装 | デジタルセンサー |
| Thermistor curves | ✅ | 🟡 | 部分 | 線形近似のみ |
| Sensor checker | ✅ | ❌ | 未実装 | 診断機能 |

### 2.3 通信・診断

| 機能 | old-src (C++) | protocol | 実装状況 | 備考 |
|------|---------------|----------|----------|------|
| **Binary Protocol** | | | | |
| Packet framing | Java | ✅ | 完了 | CRC32, big-endian |
| Opcodes | Java | ✅ | 完了 | TS_HELLO, TS_READ等 |
| Chunked read/write | Java | ✅ | 完了 | |
| Burn command | Java | ✅ | 完了 | |
| Output channels | Java | ✅ | 完了 | |
| **Transports** | | | | |
| TCP | Java | ✅ | 完了 | tokio実装 |
| Serial/UART | Java | 🟡 | 部分 | オプション機能、未テスト |
| **CAN Bus** | | | | |
| HAL driver（全ボード） | ✅ | ✅ | 完了 | heapless SPSCキュー経由、async/sync ブリッジ |
| OBD2 PID対応 | ✅ | ✅ | 完了 | PID 0x01-0x11, 0x13, 0x14 対応。`ObdEngineState` 追加 |
| Dash protocol — Haltech | ✅ | ✅ | 完了 | 0x360-0x362、RPM/TPS/MAP/IAT/CLT/Lambda/Oil |
| Dash protocol — BMW E-series | ✅ | ✅ | 完了 | 0x0AA, 0x0D5 |
| Dash protocol — Honda K-series | ✅ | ✅ | 完了 | 0x204, 0x309 |
| Dash protocol — Nissan/VAG | ✅ | ❌ | 未実装 | |
| Wideband comm | ✅ | ❌ | 未実装 | rusEFI wideband未実装 |

### 2.4 高度機能

| 機能 | old-src (C++) | engine-core | 実装状況 |
|------|---------------|-------------|----------|
| **Safety/Protection** | | | |
| Limp mode | ✅ | ❌ | 未実装 |
| Knock control | ✅ | ❌ | 未実装 |
| Overboost protect | ✅ | ❌ | 未実装 |
| Rev limiter | ✅ | ❌ | 未実装 |
| **Advanced Control** | | | |
| Launch control | ✅ | ❌ | 未実装 |
| Antilag | ✅ | ❌ | 未実装 |
| Nitrous control | ✅ | ❌ | 未実装 |
| Traction control | ✅ | ❌ | 未実装 |
| **Other** | | | |
| Lua scripting | ✅ | ❌ | 未実装 |
| Flex fuel | ✅ | ❌ | 未実装 |
| AC control | ✅ | ❌ | 未実装 |
| Alternator | ✅ | ❌ | 未実装 |
| Fan control | ✅ | ❌ | 未実装 |

---

## 3. HAL Traits実装状況

### 3.1 定義済みTraits (`engine-core/src/hal.rs`)

| Trait | 定義 | hal-sim | hal-stm32-common | 備考 |
|-------|------|---------|-------------------|------|
| `TriggerInput` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |
| `IgnitionOutput` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |
| `AdcInput` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |
| `SystemTimer` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |
| `InjectorOutput` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |
| `CanBus` | ✅ | 🟡ループバック | ✅ | 全ボードで `Stm32CanDriver` 実装済 |
| `UartPort` | ✅ | ✅ | 🟡スタブ | シミュレータ実装済 |

### 3.2 ボード固有HAL実装状況

| ボード | adc | ign | inj | trigger | can | timer sleep_us | schedule_us | 状態 |
|--------|-----|-----|-----|---------|-----|----------------|-------------|------|
| hal-microrusefi | ✅ | ✅ | ✅ | ✅ EXTI | ✅ | ✅ | ❌ | schedule_us未実装 |
| hal-proteus | ✅ | ✅ | ✅ | ✅ EXTI | ✅ | ✅ | ❌ | schedule_us未実装 |
| hal-nano | ✅ | ✅ | ✅ | ✅ EXTI | ✅ | ✅ | ❌ | schedule_us未実装 |
| hal-uaefi | ✅ | ✅ | ✅ | ✅ EXTI | ✅ | ✅ | ❌ | schedule_us未実装 |
| hal-huge | ✅ | ✅ | ✅ | ✅ EXTI | ✅ | ✅ | ❌ | schedule_us未実装 |

**補足**:
- `can.rs` は全ボードで `Stm32CanDriver` (heapless SPSC Producer/Consumer) 実装済み。`can_task()` で embassy async CAN と sync `CanBus` trait をブリッジ。
- `schedule_us()` はコールバックベースのスケジューリング。全ボードで `unimplemented!()` 。
- `sleep_us()` / `sleep_ms()` (embassy-time) は全ボードで実装済み。現在の stm32/main.rs はこちらを使用。

### 3.3 stm32/main.rs の状態

- マージコンフリクト解決済み（行90-95 — `_injector_out_ref` 行を削除）
- cam pulse 処理ブロックが `// TODO: cam phase sync for 720° cycle identification` のまま
- 制御ループはバッチ噴射のみ（sequential injection のロジックは engine-core に実装済みだが未統合）

---

## 4. 機能ギャップサマリー

### 4.1 優先度：Critical（動作ブロッカー）

| 機能 | 詳細 |
|------|------|
| Cam phase sync (制御ループ統合) | ロジックは engine-core に存在、stm32/main.rs の統合が TODO |
| Sequential injection (制御ループ統合) | ロジックは engine-core に存在、cam sync 統合が前提 |
| `schedule_us()` 全ボード | `unimplemented!()` — コールバックベースのタイマー未実装 |

### 4.2 優先度：High（実用性向上）

| 機能 |
|------|
| MAF/Alpha-N airmass |
| Lambda/O2, MAF, Oil pressure センサー |
| Electronic throttle body (PID) |
| Limp mode |
| SD card サポート |

### 4.3 優先度：Low（特殊用途）

| 機能 |
|------|
| Launch control / Antilag |
| Traction control / Shift cut |
| Lua scripting |
| Bluetooth / USB communication |

---

## 5. テスト・ビルド検証

### 5.1 テスト実行

```bash
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib
# 結果: 246 passed (25 failed — pre-existing, 多spark/wall wetting 7+5 新テスト含む)
cargo test -p rusefi-protocol --lib
cargo test -p rusefi-client --lib
```

### 5.2 ビルド検証

```bash
cargo build -p rusefi-sim --features cyl-4,fuel-fi  # ✅ OK
cargo build -p rusefi-core --features cyl-4,fuel-fi --lib  # ✅ OK
# stm32ファームウェア: embassy-stm32 に stm32f407zg 等の具体チップ feature が必要（設定課題）
```

---

## 6. まとめ

### 6.1 実装完了機能

- ✅ Trigger wheel decoder（missing-tooth、複数構成対応）
- ✅ Basic ignition control（dwell、スパーク角）
- ✅ **Multi-spark**（`MultiSparkController`、RPMリニアスケーリング）
- ✅ Basic fuel injection（パルス幅計算、バッチ噴射）
- ✅ **Wall wetting compensation**（Aquinoモデル、MultiCylWallWetting）
- ✅ Sequential injection ロジック（`SequentialController`, `SequentialInjection`）
- ✅ Engine cycle 型定義（`CylinderState`, `CyclePosition`, TDC offset計算）
- ✅ Core HAL traits（シミュレータ実装完備）
- ✅ STM32 HAL ドライバ（ADC, 点火, インジェクター, トリガー, **CAN** — 全ボード実装済み）
- ✅ Binary protocol（パケット、オペコード）
- ✅ TCP transport
- ✅ CLIツール（hello、read-image、burn、output-channels）
- ✅ コード生成（INI生成、Cヘッダー生成、Enum文字列変換）
- ✅ **OBD2 PID対応**（PID 0x01-0x11, 0x13, 0x14）
- ✅ **CAN dash protocols**（Haltech, BMW E-series, Honda K-series）

### 6.2 要実装機能（優先度順）

1. **Cam phase sync 制御ループ統合**（`stm32/main.rs` の TODO）
2. **Sequential injection 制御ループ統合**（cam sync が前提）
3. **`schedule_us()` 実装**（全ボード — コールバックベースのタイマー）
4. **センサー拡張**（MAF、Lambda等）
5. **高度機能**（ETB、Lua、launch control等）

---

*レポート生成: 2026年5月10日 — rusEFI Rust 機能網羅性チェック*
