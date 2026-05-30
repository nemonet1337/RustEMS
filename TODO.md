# rusEFI Rust 実装 TODOリスト

最終更新: 2026年5月30日（新 Device API 設計および機能網羅性レポートに基づき統合）

## 優先度：High（実用性向上）

### ファームウェア統合（engine-core に実装済み、制御ループ未配線）
- [ ] Idle 制御出力（IAC PWM HAL が前提）
- [ ] Boost 制御出力（ソレノイドPWM HAL が前提）
- [ ] VVT 制御出力（VVTソレノイドPWM HAL が前提）
- [ ] Knock 退避を点火進角に反映（ノックセンサー入力が前提）
- [ ] Closed-loop λ 補正を噴射に反映（λセンサー読取が前提）
- [ ] Acceleration enrichment / Wall wetting を噴射に反映
- [ ] DFCO（減速時燃料カット）をループに反映
- [ ] Fuel pump リレー制御（リレー出力 HAL が前提）
- [ ] Protection / limp mode をループに反映
- [ ] CAN/OBD2 / dash の定期送信タスク
- [ ] Overdwell 保護のループ反映（`OverdwellController`）
- [ ] LTFT（学習補正）を噴射計算に反映（`LtftState`）
- [ ] MAF / Alpha-N 吸入空気量計算モデルのループ反映
- [ ] センサー読取の配線（Thermistor, Lambda, Oil pressure, Fuel level）
- [ ] 汎用PWM/Aux PID制御（`AuxPidController`）
- [ ] ファン / ACリレー制御
- [ ] Start/Stop および Shutdown 制御の配線

### センサー拡張（未実装）
- [ ] SENT プロトコル（デジタルセンサー）
- [ ] 冗長TPS（デュアルTPS整合チェック）
- [ ] Flex fuel センサー
- [ ] Wideband CAN（rusEFI wideband）

### ストレージ
- [ ] SDカードロギング（全ボード）
- [ ] フラッシュ設定永続化の実機検証（`MemoryStorage` は実装済、フラッシュHAL未配線）

### 新 Device API（RDP）— TunerStudio 互換を廃止し USB/Bluetooth 直結へ
> 設計資料: `docs/api/`（README + 01〜05）。旧 TunerStudio 互換は段階的に置き換える。
- [ ] 新クレート `device-api`（no_std）— フレーミング(COBS+CRC16)/断片化/メッセージ型/CBORコーデック
- [ ] パラメータカタログ生成（`codegen` を INI ではなく `ParamDesc`/`TableDesc` 生成へ転用）
- [ ] param_id ↔ `EngineConfig` フィールドのアクセサ表（型安全・オフセット非露出）
- [ ] デバイス側ハンドラ（`engine-core/comms` を RDP に対応：System/Descriptor/Config/Telemetry/Control/Diagnostics）
- [ ] テレメトリ購読型ストリーム（旧固定20B `OutputChannels` を置換）
- [ ] RAM ステージング→フラッシュ確定（ConfigSave/Discard/ResetDefaults）+ フラッシュ書込み配線
- [ ] USB CDC-ACM トランスポート（`embassy-usb`）
- [ ] Bluetooth トランスポート（SPP=UART ストリーム / BLE GATT ブリッジ）
- [ ] ホスト側クライアント刷新（`client`/`cli` を RDP 対応、PC/スマホ向け）

### 旧 TunerStudio 互換（廃止予定・移行期のみ維持）
- [ ] RDP 移行完了後に `protocol` クレート・INI 生成・TS 互換応答を撤去

## 優先度：Low（特殊用途）

- [ ] Launch control（2ステップ、アンチラグ）
- [ ] Traction control
- [ ] Shift cut / Nitrous control
- [ ] TCU 実出力（`Tcu` ロジックは実装済、ソレノイド出力未配線）
- [ ] ETB / DBW 制御（Huge/Proteus/uaEFI はデュアルETBサポート、HALドライバ未実装）
- [ ] Lua scripting（ランタイム統合）

## コード生成（codegen）関連の未実装
- [ ] codegen ↔ comms レイアウトの一元化（INI ↔ ファームウェア定数ページの自動同期）
- [ ] 定数ページ ↔ `EngineConfig` マッピング（ライブ編集の前提）

## 技術的負債

- [ ] `[workspace.lints]` を各クレートで適用（`[lints] workspace = true`）。
      現状 `.expect()` / 旧 `unimplemented!()` がビルドを通る。適用すると既存の
      `stm32/main.rs` の `.expect()` 等が clippy deny に該当するため要整理。
- [ ] Huge/Proteus の12chピン割当（PE4..PE15 / PF0..PF11）を実機回路図で検証。

---

## 検証コマンド

```bash
# コア（気筒数別）
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib    # 273 passed
cargo test -p rusefi-core --features cyl-12,fuel-fi --lib   # 272 passed

# プロトコル / クライアント / コード生成 / シミュレータ
cargo test -p rusefi-protocol --lib
cargo test -p rusefi-client --lib
cargo test -p rusefi-codegen
cargo test -p rusefi-sim --features cyl-4,fuel-fi

# 全5ボードのファームウェア
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4,cyl-4,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f7,cyl-12,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features uaefi,cyl-6,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4-huge,cyl-12,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4-nano,cyl-2,fuel-fi
```
