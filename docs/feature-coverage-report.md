# rusEFI Rust 機能網羅性レポート

最終更新: 2026年5月30日

## 概要

本レポートは、既存のC++/Java実装（old-src）と新Rust実装の機能比較、Rustクレート
間の実装一貫性、および5ボード（microRusEFI / Proteus / uaEFI / Huge / Nano）の
動作状況をまとめたものです。

> 本更新では、`engine-core` の壊れていた制御ロジック（30件のテスト失敗）を修正し、
> 気筒数を最大12気筒まで一般化し、Huge / Proteus を12チャンネル駆動に拡張、IAT/TPS
> センサーとRPMリミッターをファームウェアに統合しました。

---

## 1. ビルド・テスト検証

### 1.1 ホストテスト（全て成功）

| クレート | 結果 |
|----------|------|
| `rusefi-core`（cyl-4,fuel-fi） | 273 passed |
| `rusefi-core`（cyl-12,fuel-fi） | 272 passed |
| `rusefi-protocol` | 14 passed |
| `rusefi-client` | 7 passed |
| `rusefi-codegen` | 21 passed |
| `rusefi-hal-sim`（cyl-4,fuel-fi） | 3 passed |
| `rusefi-sim`（cyl-4,fuel-fi、回帰+統合） | 24 + 17 passed |

```bash
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib      # 273 passed
cargo test -p rusefi-sim  --features cyl-4,fuel-fi            # 全成功
```

> 注: `cargo test --workspace` はボードHALが各々別チップの embassy feature を要求する
> ため同時ビルドできません（stm32-metapac が「Multiple stm32xx features」でpanic）。
> これは複数ボード構成の設計上の制約であり、各ボードは個別にビルド/テストします。

### 1.2 ファームウェアビルド（全5ボード成功 / thumbv7em-none-eabihf）

| ボード | feature | dev | release |
|--------|---------|-----|---------|
| microRusEFI | `stm32f4,cyl-4,fuel-fi` | ✅ | ✅ |
| Proteus | `stm32f7,cyl-12,fuel-fi` | ✅ | ✅ |
| uaEFI | `uaefi,cyl-6,fuel-fi` | ✅ | ✅ |
| Huge | `stm32f4-huge,cyl-12,fuel-fi` | ✅ | ✅ |
| Nano | `stm32f4-nano,cyl-2,fuel-fi` | ✅ | ✅ |

```bash
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4-huge,cyl-12,fuel-fi
```

---

## 2. 気筒数サポート（本更新の主要成果）

- `MAX_CYLINDERS = 12` を導入。`EngineConfig.firing_order` を 4 → 12 容量に拡張。
- `SequentialController` / `MultiCylWallWetting` を 4 → 12 気筒に一般化。
- `EngineConfig::default_{3,5,6,8,10,12}cyl()` を発火順序付きで追加
  （全て 0..N の妥当な置換であることをテストで検証）。
- ファームウェアは `cyl-N` feature に対応する設定を選択（以前は cyl-6/8/12 が
  誤って4気筒設定で動作していた）。
- Huge / Proteus の点火・噴射ドライバを12チャンネルに拡張（coils PE4..PE15,
  injectors PF0..PF11、`Vec<Output, MAX_CYLINDERS>`）。

| ボード | 定格気筒 | 点火ch | 噴射ch | 状態 |
|--------|----------|--------|--------|------|
| Nano | 2 | 2 | 8（バッチ） | ✅ |
| microRusEFI | 4 | 4 | 4 | ✅ |
| uaEFI | 6 | 6 | 6 | ✅ |
| Proteus | 12 | 12 | 12 | ✅ |
| Huge | 12 | 12 | 12 | ✅ |

> ⚠ Huge/Proteus の12chピン割当（PE4..PE15 / PF0..PF11）は公称値です。実機の
> 回路図に合わせた検証が必要です（ADC=PA/PC, trigger=PA5/PA8, CAN=PD0/PD1 とは非衝突）。

---

## 3. 機能比較マトリックス（engine-core）

### 3.1 コア制御

| 機能 | old-src | engine-core | ファーム統合 | 備考 |
|------|---------|-------------|--------------|------|
| Missing-tooth デコーダ | ✅ | ✅ | ✅ | 36-1,60-2,12-1,4-1,24-1/2,36-2 |
| Cam phase sync | ✅ | ✅ | ✅ | FullSync→Sequential自動切替 |
| Instant RPM | ✅ | ✅ | — | `InstantRpmCalculator` |
| 点火（dwell/advance） | ✅ | ✅ | ✅ | `compute_ignition` |
| CLT/IAT 点火補正 | ✅ | ✅ | ✅ | **本更新で配線** |
| Multi-spark | ✅ | ✅ | — | `MultiSparkController` |
| Overdwell 保護 | ✅ | ✅ | — | `OverdwellController` |
| RPM リミッター | ✅ | ✅ | ✅ | **本更新でファーム統合** |
| 噴射パルス | ✅ | ✅ | ✅ | `compute_injection` |
| Sequential injection | ✅ | ✅ | ✅ | 最大12気筒 |
| Wall wetting | ✅ | ✅ | — | Aquinoモデル, 最大12気筒 |
| Accel enrichment | ✅ | ✅ | — | `AccelEnrichmentController` |
| DFCO | ✅ | ✅ | — | `DfcoController` |
| Closed-loop（λ） | ✅ | ✅ | — | `ClosedLoopController` |
| LTFT | ✅ | ✅ | — | `LtftController` + storage |
| Speed Density | ✅ | ✅ | ✅ | VE table |
| MAF airmass | ✅ | ✅ | — | `MafFuelCalculator` |
| Alpha-N | ✅ | ✅ | — | 点火負荷/燃料の両方 |

### 3.2 センサー

| 機能 | engine-core | 備考 |
|------|-------------|------|
| CLT/IAT/TPS/MAP/Vbatt | ✅ | ファームで全ch読取 |
| Thermistor（Steinhart-Hart） | ✅ | 3点校正 |
| Lambda（Narrow/Wideband） | ✅ | 区分線形（2.5V=λ1.0） |
| Oil pressure | ✅ | `OilPressureSensor` |
| Fuel level | ✅ | `FuelLevelSensor` |
| IIRフィルタ | ✅ | 各chに適用 |

### 3.3 アクチュエータ

| 機能 | engine-core | 備考 |
|------|-------------|------|
| Idle（PID） | ✅ | CLT目標RPM, AC idle-up |
| Boost | ✅ | `BoostController` |
| VVT（Dual） | ✅ | `DualVvtController` |
| 汎用PWM/Aux PID | ✅ | `AuxPidController` |
| Fuel pump | ✅ | prime→run, 始動で再励磁 |
| Fan / AC | ✅ | ヒステリシス |

### 3.4 保護・高度制御

| 機能 | engine-core | 備考 |
|------|-------------|------|
| Limp mode / Protection | ✅ | `ProtectionMonitor`（過熱/油圧/センサー） |
| Knock 制御 | ✅ | `KnockController`（ノック時は当該周期で進角回復しない） |
| Start/Stop | ✅ | `StartStopController` |
| Shutdown | ✅ | `ShutdownController` |
| TCU | ✅ | `Tcu`（手動/自動, ロックアップ） |

### 3.5 通信

| 機能 | 実装 | 備考 |
|------|------|------|
| バイナリプロトコル（TS互換） | ✅ | CRC32, opcodes, chunked |
| TCP/Serial transport | ✅ | tokio |
| CAN driver（全ボード） | ✅ | heapless SPSC, async/sync ブリッジ |
| OBD2 PID | ✅ | 0x01-0x11,0x13,0x14 |
| Dash（Haltech/BMW/Honda） | ✅ | |

---

## 4. ファームウェア制御ループ（stm32/main.rs）

統合済:
- トリガーデコード → 点火 → 噴射（バッチ/シーケンシャル自動切替）
- Cam pulse → FullSync → Sequential injection 起動
- センサー: MAP/CLT/IAT/TPS/Vbatt 読取 + IIRフィルタ
- CLT/IAT 点火補正、RPMリミッター（スパークカット）
- `cyl-N` に応じた設定選択、最大12気筒

engine-core に存在するがループ未統合（今後の統合候補）:
- Idle / Boost / VVT 出力、Knock 退避、Closed-loop λ、Accel enrichment、
  DFCO、Wall wetting、Fuel pump リレー、Protection/limp、CAN/OBD2 定期送信。
  ※ これらは専用HAL出力（PWM/リレー）と実機タイミング検証が前提のため未配線。

---

## 5. 残ギャップ（詳細は feature-coverage-todo.md）

- 上記アクチュエータ/補正のファームウェア統合（HAL出力の追加が前提）
- Launch control / Antilag / Traction control / Nitrous（未実装）
- Lua scripting、Bluetooth/USB、SDカードロギング（未実装）
- SENT デジタルセンサー、冗長TPS、Wideband CAN（未実装）
- `[workspace.lints]` が各クレートで未適用（`.expect()`/`unimplemented!()` が
  ビルドを通る）。ポリシー強制は今後の課題。

---

*レポート生成: 2026年5月30日 — rusEFI Rust 機能網羅性チェック*
