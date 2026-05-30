# rusEFI Rust 機能網羅性レポート

最終更新: 2026年5月30日

## 概要

本レポートは、既存のC++/Java実装（old-src）と新Rust実装の機能比較、Rustクレート
間の実装一貫性、および5ボード（microRusEFI / Proteus / uaEFI / Huge / Nano）の
動作状況をまとめたものです。

---

## 1. ビルド・テスト検証

### 1.1 ホストテスト（全て成功）

| クレート | テスト結果 |
|----------|------------|
| `rusefi-core`（cyl-4, fuel-fi） | 273 passed |
| `rusefi-core`（cyl-12, fuel-fi） | 272 passed |
| `rusefi-protocol` | 14 passed |
| `rusefi-client` | 7 passed |
| `rusefi-codegen` | 21 passed |
| `rusefi-hal-sim`（cyl-4, fuel-fi） | 3 passed |
| `rusefi-sim`（cyl-4, fuel-fi、回帰＋統合） | 41 passed |

```bash
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib      # 273 passed
cargo test -p rusefi-sim  --features cyl-4,fuel-fi            # 41 passed
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

## 2. 気筒数サポート

| ボード | 定格気筒 | 点火ch | 噴射ch |
|--------|----------|--------|--------|
| Nano | 2 | 2 | 2（バッチ4可） |
| microRusEFI | 4 | 4 | 4 |
| uaEFI | 6 | 6 | 6 |
| Proteus | 12 | 12 | 12 |
| Huge | 12 | 12 | 12 |

- `MAX_CYLINDERS = 12`。`firing_order`, `SequentialController`, `MultiCylWallWetting`
  は全て12気筒まで一般化済み。
- `EngineConfig::default_{1,2,3,4,5,6,8,10,12}cyl()` を発火順序付きで提供。
- ファームウェアは `cyl-N` feature に対応する設定を選択。

> ⚠ Huge/Proteus の12chピン割り当て（PE4–PE15 / PF0–PF11）は公称値です。
> 実機の回路図との照合が必要です（ADC=PA/PC, trigger=PA5/PA8, CAN=PD0/PD1 とは
> 非衝突を確認済み）。

---

## 3. 機能比較マトリックス（engine-core）

### 3.1 コア制御

| 機能 | old-src | engine-core | ファーム統合 | 備考 |
|------|---------|-------------|--------------|------|
| Missing-tooth デコーダ | ✅ | ✅ | ✅ | 36-1, 60-2, 12-1, 4-1, 24-1/2, 36-2 |
| Cam phase sync | ✅ | ✅ | ✅ | CrankSynced → FullSync 自動切替 |
| Instant RPM | ✅ | ✅ | ✅ | tooth interval から推定 |
| 点火（dwell/advance） | ✅ | ✅ | ✅ | `compute_ignition`, 16×16テーブル |
| CLT/IAT 点火補正 | ✅ | ✅ | ✅ | ファーム制御ループに配線済み |
| Overdwell 保護 | ✅ | ✅ | — | `OverdwellController`, 最大10ms |
| RPM リミッター | ✅ | ✅ | ✅ | スパークカット、ヒステリシス付き |
| 噴射パルス | ✅ | ✅ | ✅ | `compute_injection`, VE 16×16テーブル |
| Sequential injection | ✅ | ✅ | ✅ | FullSync時、バッチにフォールバック |
| Wall wetting | ✅ | ✅ | — | Aquinoモデル, MultiCylWallWetting |
| Accel enrichment | ✅ | ✅ | — | `AccelEnrichmentController`, TPS変化率 |
| DFCO | ✅ | ✅ | — | `DfcoController` |
| Closed-loop（λ） | ✅ | ✅ | — | `ClosedLoopController` |
| LTFT | ✅ | ✅ | — | `LtftState`, 8セルテーブル |
| Speed Density | ✅ | ✅ | ✅ | VEテーブル + MAP |
| MAF airmass | ✅ | ✅ | — | `MafFuelCalculator` |
| Alpha-N | ✅ | ✅ | — | `AlphaNFuelCalculator` |

### 3.2 センサー

| 機能 | engine-core | ファーム統合 | 備考 |
|------|-------------|-------------|------|
| CLT/IAT/TPS/MAP/Vbatt | ✅ | ✅ | ADC1 + IIRフィルタ |
| Thermistor（Steinhart-Hart） | ✅ | — | 3点校正 |
| Lambda（Narrow/Wideband） | ✅ | — | 区分線形（2.5V=λ1.0） |
| Oil pressure | ✅ | — | `OilPressureSensor` |
| Fuel level | ✅ | — | `FuelLevelSensor` |
| IIRフィルタ | ✅ | ✅ | CLT/IAT α=0.1, TPS α=0.3, MAP α=0.2 |

### 3.3 アクチュエータ

| 機能 | engine-core | ファーム統合 | 備考 |
|------|-------------|-------------|------|
| Idle PID | ✅ | — | CLT目標RPMテーブル, AC idle-up |
| Boost | ✅ | — | 開ループ + PID補正 |
| VVT（Dual） | ✅ | — | 開ループ / PID, 8×8テーブル |
| 汎用PWM/Aux PID | ✅ | — | `AuxPidController` |
| Fuel pump | ✅ | — | prime → run, 始動で再励磁 |
| Fan / AC | ✅ | — | ヒステリシス |

### 3.4 保護・高度制御

| 機能 | engine-core | ファーム統合 | 備考 |
|------|-------------|-------------|------|
| Protection / Limp mode | ✅ | — | `ProtectionMonitor`（過熱/油圧/センサー） |
| Knock 制御 | ✅ | — | `KnockController`（ノック検出・退避フレーム） |
| Start/Stop | ✅ | — | `StartStopController` |
| Shutdown | ✅ | — | `ShutdownController` |
| TCU | ✅ | — | `Tcu`（手動/自動, ロックアップ） |

### 3.5 通信

| 機能 | 実装 | ファーム統合 | 備考 |
|------|------|-------------|------|
| デバイス側プロトコル（TS互換） | ✅ | ✅ | no_std, CRC32, 全opcodeに対応 |
| UART TunerStudio（USART1） | ✅ | ✅ | 115200 8N1, BufferedUart, comms_task |
| TCP/Serial transport（ホスト） | ✅ | — | tokio ベース CLI / client 向け |
| CAN driver（全ボード） | ✅ | — | heapless SPSC, async/sync ブリッジ |
| OBD2 PID | ✅ | — | 0x01–0x11, 0x13, 0x14 |
| Dash（Haltech/BMW/Honda） | ✅ | — | `can/dash.rs` |
| TunerStudio INI | ✅ | — | `tunerstudio/rustems.ini` |

---

## 4. ファームウェア制御ループ（stm32/main.rs）

### Embassy タスク構成

```
main() [Embassy executor]
├── crank_task   — PA8 EXTI割り込み → クランクタイムスタンプキュー
├── cam_task     — PA5 EXTI割り込み → カムタイムスタンプキュー
├── comms_task   — USART1 TunerStudio通信（115200 8N1）
└── control_loop — インライン非同期：センサー → 点火 → 噴射
```

### 統合済み機能

- トリガーデコード（missing-tooth）→ 点火（dwell/advance + CLT/IAT補正）
- Sequential injection（FullSync時）, バッチ injection（CrankSync以上のフォールバック）
- Cam pulse → FullSync 確立 → Sequential injection 起動
- センサー: MAP/CLT/IAT/TPS/Vbatt ADC読取 + IIRフィルタ
- RPM リミッター（スパークカット）
- PC チューニング: TunerStudio バイナリプロトコル over USART1
- ライブテレメトリ: RPM/CLT/IAT/MAP/TPS/Vbatt/Lambda/噴射パルス幅/点火進角/
  spark_cut フラグ / sequential フラグ

### ファーム未統合（engine-coreに実装済み）

以下は engine-core に実装済みだが、ファームウェアの制御ループには未配線。
専用HAL出力（PWM/リレー）の実装と実機タイミング検証が前提のため。

- Idle / Boost / VVT 出力（IAC/ソレノイドPWM HAL が必要）
- Knock 退避→点火進角反映（ノックセンサー入力HAL が必要）
- Closed-loop λ 補正（λセンサー読取 HAL が必要）
- Accel enrichment / Wall wetting（噴射計算への統合）
- DFCO（減速時燃料カット）
- Fuel pump リレー制御
- Protection / limp mode
- CAN/OBD2 / dash 定期送信タスク

---

## 5. コード生成（codegen）

| 機能 | 状態 | 備考 |
|------|------|------|
| `rusefi_config.txt` パーサー | ✅ | 設定定義言語を解析 |
| AST モデル / レジストリ | ✅ | シンボル解決 |
| Cヘッダー生成 | ✅ | `CHeaderGenerator` |
| TunerStudio INI 生成 | ✅ | `TsIniGenerator` |
| Java レジストリ生成 | ✅ | `JavaRegistryGenerator` |
| codegen ↔ comms レイアウト一元化 | ❌ | 未実装（単一ソース化が前提） |
| 定数ページ ↔ EngineConfig マッピング | ❌ | 未実装（ライブ編集の前提） |

---

## 6. 残ギャップ（詳細は feature-coverage-todo.md）

- 上記アクチュエータ/補正のファームウェア統合（HAL出力追加が前提）
- Launch control / Antilag / Traction control / Nitrous（未実装）
- Lua scripting、Bluetooth/USB、SDカードロギング（未実装）
- SENT デジタルセンサー、冗長TPS、Wideband CAN（未実装）
- Flex fuel センサー、ETB（DBW）制御（未実装）
- `[workspace.lints]` の各クレートへの適用（`.expect()`/`unimplemented!()` が現状
  ビルドを通る）。`stm32/main.rs` の整理が前提。
- codegen と comms レイアウトの一元化（INI ↔ ファームウェア定数ページの自動同期）

---

*レポート生成: 2026年5月30日 — rusEFI Rust 機能網羅性チェック*
