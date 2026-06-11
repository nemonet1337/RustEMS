# rusEFI Rust 実装 TODOリスト

最終更新: 2026年6月10日（RDP デバイス/ホスト両側のフル実装・性能改善を反映）

## 優先度：High（実用性向上）

### ファームウェア統合（engine-core に実装済み、制御ループ配線状況）
- [x] Idle 制御出力（`IdleController` を sim / stm32 ループに配線、PWM はスタブ出力）
- [x] Boost 制御出力（`BoostController` を配線、ソレノイド PWM HAL はスタブ）
- [ ] VVT 制御出力（VVTソレノイドPWM HAL が前提）
- [x] Knock 退避を点火進角に反映（stm32 ループで `knock_retard_deg` 適用。生ノックADC入力は未配線）
- [x] Closed-loop λ 補正を噴射に反映（`ClosedLoopController` 配線済）
- [x] Acceleration enrichment / Wall wetting を噴射に反映
- [x] DFCO（減速時燃料カット）をループに反映
- [x] Fuel pump リレー制御（`FuelPumpController` 配線済、リレーHALはスタブ）
- [x] Protection / limp mode をループに反映（`ProtectionMonitor` 配線済）
- [ ] CAN/OBD2 / dash の定期送信タスク（フレーム生成は実装済、CANドライバ未配線）
- [x] Overdwell 保護のループ反映（`OverdwellController`）
- [x] LTFT（学習補正）を噴射計算に反映（`LtftState`）
- [ ] MAF / Alpha-N 吸入空気量計算モデルのループ反映（Speed-Density のみ配線済）
- [x] センサー読取の配線（CLT/IAT/TPS/MAP/VBatt/Lambda/Oil pressure/Fuel level）
- [ ] 汎用PWM/Aux PID制御（`AuxPidController`、出力先HALが前提）
- [x] ファン リレー制御（`FanController` 配線済）。ACリレーは未配線
- [ ] Start/Stop および Shutdown 制御の配線

### センサー・車両制御拡張
- [ ] SENT プロトコル（デジタルセンサー）
- [x] 冗長TPS（デュアルTPS整合チェック — `EtbController` の plausibility 監視として実装）
- [ ] Flex fuel センサー（`FlexFuelSensor` 実装済、タイマキャプチャ未配線）
- [ ] Wideband CAN（rusEFI wideband）
- [x] Wideband λ ヒータ制御（`sensors::heater::HeaterController` — 結露/ランプ/電圧補償ホールドの3相制御）

### ストレージ
- [ ] SDカードロギング（全ボード）
- [ ] フラッシュ設定永続化の実機検証（`MemoryStorage` は実装済、フラッシュHAL未配線。
      RDP の ConfigSave はフラッシュスナップショット（RAM）まで配線済）

### 新 Device API（RDP）— TunerStudio 互換を廃止し USB/Bluetooth 直結へ
> 設計資料: `docs/api/`（README + 01〜05）。旧 TunerStudio 互換は段階的に置き換える。
- [x] 新クレート `device-api`（no_std）— フレーミング(COBS+CRC16)/断片化/メッセージ型/CBORコーデック
- [x] パラメータカタログ（静的 `ParamMeta`/`TableMeta`/`ChannelMeta` を `engine-core` に実装。
      `codegen` 転用は不要になった — カタログはファームウェア常駐の `&'static` データ）
- [x] param_id ↔ `EngineConfig` フィールドのアクセサ表（型安全・オフセット非露出、`params.rs`）
- [x] デバイス側ハンドラ（`engine-core/comms/rdp.rs` — System/Descriptor/Config/Telemetry/Control/Diagnostics 全オペコード）
- [x] テレメトリ購読型ストリーム（`comms/telemetry.rs` — レート制御・seq・密パック、旧固定20Bを置換）
- [x] RAM ステージング→フラッシュ確定（ConfigSave/Discard/ResetDefaults/Status + dirty/CRC追跡。
      実機フラッシュ書込みドライバのみ未配線）
- [x] フォルト（DTC）ストア / 非同期イベント（`comms/faults.rs`）
- [x] オーバーライド・ベンチテスト・キャリブレーション（`comms/control.rs`、タイムアウト・切断フェイルセーフ付き）
- [x] stm32 comms タスクへの統合（UART 上で TS とデュアルスタック、設定エポックで制御ループへ反映）
- [x] PC シミュレータ RDP サーブモード（`rusefi-sim --serve <port>`、TCP）
- [ ] USB CDC-ACM トランスポート（`embassy-usb`）
- [ ] Bluetooth トランスポート（SPP=UART ストリーム / BLE GATT ブリッジ）
- [x] ホスト側クライアント刷新（`client/src/rdp.rs` + `rusefi rdp ...` サブコマンド群）

### 旧 TunerStudio 互換（廃止予定・移行期のみ維持）
- [ ] RDP 移行完了後に `protocol` クレート・INI 生成・TS 互換応答を撤去

## 優先度：Low（特殊用途）

- [ ] Launch control 2ステップ/アンチラグ（`LaunchControl` 実装済、車速入力未配線）
- [x] Traction control（`traction::TractionController` — スリップ率→漸進リタード→スパークカット）
- [ ] Shift cut / Nitrous control
- [ ] TCU 実出力（`Tcu` ロジックは実装済、ソレノイド出力未配線）
- [x] ETB / DBW 制御ロジック（`actuators::etb::EtbController` — PID + デュアルTPS plausibility + リンプ。
      H ブリッジ PWM HAL ドライバは未実装）
- [ ] Lua scripting（ランタイム統合）

## コード生成（codegen）関連

- [ ] codegen ↔ comms レイアウトの一元化（旧TS INI 用。RDP では静的カタログ + schema_hash で代替済み、
      TS 撤去とあわせて再評価）
- [x] 定数ページ ↔ `EngineConfig` マッピング（RDP の `params.rs` アクセサ表で実現）

## パフォーマンス改善（実施済み）

- [x] テーブル補間の bin 探索を線形走査 → 二分探索化（`maps/interpolation.rs`、
      点火+噴射の per-tooth ホットパスで軸ルックアップ多数のため）+ `#[inline]` 付与
- [x] stm32 制御ループの設定共有をエポック方式に（per-tooth ホットパスでロックを取らない。
      `CONFIG_EPOCH` 変化時のみ再クローン）
- [x] テレメトリは購読型・密パック整数（固定 f32 ブロックのポーリング比で帯域削減）

## 技術的負債

- [ ] `[workspace.lints]` を各クレートで適用（`[lints] workspace = true`）。
      現状 `.expect()` / 旧 `unimplemented!()` がビルドを通る。適用すると既存の
      `stm32/main.rs` の `.expect()` 等が clippy deny に該当するため要整理。
- [ ] Huge/Proteus の12chピン割当（PE4..PE15 / PF0..PF11）を実機回路図で検証。
- [ ] stm32 の RDP オーバーライド/ベンチテストを制御ループの実アクチュエータへ作用させる
      （現状 comms タスク内の `RdpServer` 状態まで。共有が必要）
- [ ] MCU UID を `HelloInfo.device_id` に配線（現状ゼロ埋め）

---

## 検証コマンド

```bash
# コア（気筒数別）
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib    # 347 passed
cargo test -p rusefi-core --features cyl-12,fuel-fi --lib   # 346 passed

# RDP ワイヤ層 / プロトコル / クライアント / コード生成 / シミュレータ
cargo test -p rusefi-device-api
cargo test -p rusefi-protocol --lib
cargo test -p rusefi-client --lib
cargo test -p rusefi-codegen
cargo test -p rusefi-sim --features cyl-4,fuel-fi

# RDP エンドツーエンド（別端末で）
cargo run -p rusefi-sim --features cyl-4,fuel-fi -- --serve 29002
cargo run -p rusefi-cli -- rdp hello
cargo run -p rusefi-cli -- rdp watch --channels 1,2,7 --rate 10 --count 20

# 全5ボードのファームウェア
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4,cyl-4,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f7,cyl-12,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features uaefi,cyl-6,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4-huge,cyl-12,fuel-fi
cargo build-arm -p rusefi-stm32 --no-default-features --features stm32f4-nano,cyl-2,fuel-fi
```
