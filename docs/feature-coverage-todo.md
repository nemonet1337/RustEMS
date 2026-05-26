# rusEFI Rust 機能実装 TODOリスト

生成日時: 2026年5月10日

## 優先度：Critical（動作ブロッカー）

### Cam Phase Sync 統合
- [ ] `stm32/main.rs` でのFullSync遷移後の動作確認（実機テスト）
  - デコーダーへのCamRise通知は実装済み
  - FullSync後のSequentialInjection自動切替は実装済み

### STM32ファームウェアビルド
- [x] embassy-stm32 0.6 API 移行完了（全5ボード）
  - microRusEFI (STM32F407) — ✅
  - Huge (STM32F407) — ✅
  - Nano (STM32F407) — ✅
  - UAEFI (STM32F407) — ✅
  - Proteus (STM32F767) — ✅
  - `Peri<'a,T>`, `Output<'a>`, `ExtiInput` 4引数, `SampleTime`, `Frame`, heapless 0.9 SPSC 対応完了

## 優先度：High（実用性向上）

### Airmass Models
- [ ] MAF-based airmass（MAFエアマス）
- [ ] Alpha-N airmass（スロットル角ベース）
- [ ] Speed Density enhancement（VE table lookup、補正因子適用）

### Sensors
- [ ] Lambda/O2 sensor（Narrowband / Wideband対応）
- [ ] MAF sensor（マスエアフロー）
- [ ] Flex fuel sensor（フレキシブル燃料）
- [ ] Oil pressure sensor（油圧）
- [ ] Redundant TPS（冗長TPS）
- [ ] SENT protocol（デジタルセンサー）
- [ ] Thermistor curves（線形近似 → 真のNTC曲線）

### Actuators
- [ ] Electronic throttle body（PID制御、冗長ポテンショメータ）
- [ ] Alternator control
- [ ] AC compressor control
- [ ] Cooling fan control
- [ ] Idle valve control（PWM制御実体）

### Protection
- [ ] Limp mode（過ブースト、過熱、センサー故障検出）
- [ ] Overboost protection

### Storage
- [ ] SD card サポート（全ボード — 現状 lib.rs が 0 を返すスタブ）

## 優先度：Low（特殊用途）

### Advanced Control
- [ ] Launch control（2ステップ、アンチラグ）
- [ ] Traction control
- [ ] Shift cut
- [ ] Nitrous control

### TCU (Transmission Control)
- [ ] Shift control
- [ ] Torque converter lockup

### Connectivity
- [ ] Bluetooth integration
- [ ] SD card logging
- [ ] USB communication

### Scripting
- [ ] Lua scripting（ランタイム統合、API公開）

---

## 実装済み（チェック不要）

- ✅ HAL ドライバ実装（ADC, 点火, インジェクター, トリガー）— 全ボード実装済み
- ✅ Sequential injection ロジック（`SequentialController`, `SequentialInjection`）— engine-core 実装済み
- ✅ 4ストローク型定義（`CylinderState`, `CyclePosition`, TDC offset）— engine-core 実装済み
- ✅ コード生成（INI生成, Cヘッダー生成, Enum文字列変換）— codegen 実装済み
- ✅ `stm32/main.rs` マージコンフリクト解決済み
- ✅ **Wall wetting compensation** — `engine-core/src/fuel/wall_wetting.rs` 実装済み（Aquinoモデル）
- ✅ **Multi-spark** — `engine-core/src/ignition/mod.rs` に `MultiSparkController` 実装済み
- ✅ **CAN driver（全ボード）** — 全5ボードに `can.rs` 実装済み（heapless SPSC, async/sync ブリッジ）
- ✅ **OBD2 PID対応** — `engine-core/src/can/obd2.rs` 拡張済み（PID 0x01-0x11, 0x13, 0x14）
- ✅ **CAN dash protocols** — `engine-core/src/can/dash.rs`（Haltech, BMW E-series, Honda K-series）
- ✅ **`schedule_us()` 実装** — 全5ボードで embassy_time Instant によるブロッキングスピン実装済み
- ✅ **Cam phase sync 統合** — `stm32/main.rs` でCamRiseをデコーダーに渡しFullSyncを確立
- ✅ **Sequential injection 統合** — FullSync確立後に自動切替、バッチ注射からシーケンシャルへ

---

## 検証コマンド

```bash
# ビルド検証
cargo build -p rusefi-sim --features cyl-4,fuel-fi
cargo build -p rusefi-core --features cyl-4,fuel-fi --lib

# テスト実行
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib
cargo test -p rusefi-protocol --lib
cargo test -p rusefi-client --lib

# ドキュメント生成
cargo doc --workspace --no-deps
```

---

## メモ

- 優先度Critical の 残件は STM32 ファームウェアビルド設定と実機確認のみ
- 優先度High: センサー拡張・アクチュエーター制御が次の実装対象
- 優先度Low: 競技・特殊用途向け
