# 05. テレメトリ・制御・診断

## 1. テレメトリ（Telemetry）

旧 `OutputChannels`（固定 20B のポーリング）を、**カタログ駆動の購読型ストリーム**に置き換える。

### 1.1 チャンネル記述子（ChannelDesc）

`Descriptor.GetTelemetryCatalog` が返す。`engine-core` の `OutputChannels` /
`SensorData` を初期ソースとする。

```
ChannelDesc {
  id: u16,
  key: str,        // 例 "rpm", "clt_c", "lambda"
  label: str,
  vtype: ValueType,// 通常 f32 だが wire は scale 済み整数
  unit: str,       // "rpm","°C","kPa","%","V","λ","ms","deg"
  scale: f32,      // 物理値 = wire_raw * scale
  wire_type: enum(u16|i16|u8|bit), // 帯域節約のための実送出型
  group: u8,       // UI グルーピング用
}
```

初期チャンネル（`comms/output.rs` 由来）：
`rpm, clt_c, iat_c, map_kpa, tps_pct, battery_v, lambda, inj_pulse_ms, advance_deg,
spark_cut(bit), sequential(bit)`。拡張で `oil_pressure_kpa, maf_voltage, fuel_level_pct,
lambda2, knock_level, idle_position, boost_kpa, vvt_angle` 等を追加可能。

### 1.2 購読

```
Subscribe { channels:[u16], rate_hz:u16 } → Response { stream_id, layout:[u16] }
```

- `layout` は確定したチャンネル順。以後の `TelemFrame` はこの順の**密パック**で届く。
- 複数 `stream_id` を併用可（例：高頻度の RPM/λ を 50Hz、温度系を 2Hz）。
- `rate_hz` はデバイス能力に丸められ、実レートが応答に反映される。

### 1.3 テレメトリフレーム（push, KIND=Telemetry）

```
TelemFrame {
  stream_id: u8,
  seq: u16,        // 連番（欠落検出）
  ts_ms: u32,      // デバイス起動からの経過 ms
  data: bytes,     // layout 順に wire_type で密パック（CBOR 不使用）
}
```

- ビット型（spark_cut 等）は 1 バイトにパックして末尾に置く。
- `seq` 欠落でホストはドロップを検出できる（BLE で有用）。
- フレーム長は `layout` から静的に決まるため、ホストはカタログ既知なら追加メタ不要。

## 2. 制御 / アクション（Control）

### 2.1 ベンチテスト（BenchTest）

整備時のアクチュエータ単体作動。**安全のためエンジン非稼働（RPM≈0）時のみ受理**、
稼働中は `Error{ Busy }`。

```
BenchTest { target, index:u8, on_ms:u16, off_ms:u16, count:u16 }
```

| target | index の意味 | 例 |
|--------|--------------|----|
| Injector | 気筒/インジェクタ番号 | #2 を 3ms×5 回 |
| IgnitionCoil | コイル番号 | 火花テスト |
| FuelPump | — | プライミング |
| Fan | ファン番号 | 動作確認 |
| Idle | ステップ/PWM | アイドルバルブ駆動 |
| Tachometer | — | タコ出力テスト |

### 2.2 一時オーバーライド（SetOverride / ClearOverride）

診断・セッティング用に制御量を一時的に上書き。`timeout_ms` 経過か `ClearOverride`、
または接続切断で自動解除（フェイルセーフ）。

| target | value の意味 |
|--------|-------------|
| SparkCut | 1=全気筒点火カット |
| FuelCut | 1=全気筒燃料カット |
| TimingFix | 固定進角[deg]（点火タイミング確認用） |
| IdlePosition | アイドル開度固定[%] |
| BoostDuty | wastegate デューティ固定[%] |
| InjectorDuty | 噴射デューティ加算[%]（λ確認） |

### 2.3 キャリブレーション（Calibrate）

```
Calibrate { routine, args:[f32] } → Response { result:[f32] }
```

| routine | 説明 | args / result |
|---------|------|---------------|
| TpsClosed | 現在の TPS を全閉として学習 | result=[adc] |
| TpsOpen | 全開学習 | result=[adc] |
| MapBaro | 大気圧を MAP で学習 | result=[kpa] |
| ClearAdaptive | 学習値（λトリム等）クリア | — |

## 3. 診断 / イベント（Diagnostics）

### 3.1 故障コード（Fault / DTC）

`engine-core/protection` の `ProtectionMonitor` と `SensorResult`（TooLow/TooHigh）を源泉に、
構造化フォルトを提供する。

```
Fault {
  code: u16,         // 一意コード
  severity: enum(Info|Warn|Critical),
  active: bool,      // 現在も継続中か
  count: u16,        // 発生回数
  first_ts_ms: u32, last_ts_ms: u32,
  detail: u16,       // 文脈（センサ id, 気筒 等）
}
```

- `GetFaults` で一覧、`ClearFaults{ mask }` で消去（active な物理要因が残れば即再発）。
- 代表コード例：`SensorCltShort/Open`, `SensorIatShort/Open`, `MapImplausible`,
  `Overrev`, `Overboost`, `LowOilPressure`, `OverTemp`, `TriggerSyncLoss`,
  `LambdaImplausible`, `BatteryLow/High`。

### 3.2 非同期イベント（Event, push, KIND=Event）

ホストがポーリングせずに重要事象を即受信する。

```
Event {
  kind: enum,
  ts_ms: u32,
  a: i32, b: i32,   // kind 依存の付帯値
}
```

| kind | a / b | 用途 |
|------|-------|------|
| FaultSet | code / detail | 新規フォルト発生 |
| FaultCleared | code / — | フォルト解消 |
| Knock | cylinder / retard_milli_deg | ノック検出 |
| ProtectionCut | reason / — | 保護による点火/燃料カット |
| LimpMode | reason / — | リンプ（退避）モード遷移 |
| SyncState | gained(1)/lost(0) / teeth | トリガー同期の獲得/喪失 |
| ConfigChanged | source / — | 外部要因で設定変化（多クライアント整合） |

- イベントは BLE では専用 Notify 特性、ストリーム系では `KIND=Event` フレームで届く。
- ホスト接続直後はアクティブフォルトを `GetFaults` で取得し、以後は Event で差分追従する。

## 4. 安全方針（横断）

- ベンチテスト・オーバーライドは**エンジン稼働中は原則拒否**（`Busy`）、
  または短い `timeout_ms` と切断時自動解除で必ずフェイルセーフへ戻す。
- `EnterBootloader` / `ConfigResetDefaults` 等の不可逆操作は確認マジック値を必須にする。
- 制御系オーバーライドの実装は `unsafe` 禁止・`panic` 禁止（`workspace.lints`）を厳守し、
  入力は必ず `min/max` クランプを経てから `EngineConfig`/制御へ反映する。
