# 04. パラメータモデル（設定の表現）

旧プロトコルの「page/offset で生メモリを叩く」方式をやめ、**ID・型・メタデータを持つ
パラメータカタログ**で設定を表現する。クライアントは INI を必要とせず、デバイスから
取得したカタログだけで UI を構築できる（自己記述）。

## 1. パラメータ ID 空間

- パラメータは安定した `u16` の **param_id** で参照する（メモリオフセット非露出）。
- テーブル（マップ）は別空間の `u16` **table_id** で参照する。
- ID は `codegen` のパラメータ定義から決定的に採番し、`schema_hash` に織り込む。
  ID の意味は版を越えて不変（後方互換）。削除する場合も欠番化し再利用しない。

## 2. 値の型（ValueType）

| 値 | 型 | 備考 |
|----|----|------|
| 0 | `u8` | |
| 1 | `i8` | |
| 2 | `u16` | |
| 3 | `i16` | |
| 4 | `u32` | |
| 5 | `i32` | |
| 6 | `f32` | 物理量の既定 |
| 7 | `bool` | |
| 8 | `enum` | 別途 `enum_labels` を持つ |
| 9 | `str` | 固定長（heapless） |

`ParamValue` は `(type, raw)` のタグ付きで CBOR エンコードする。

## 3. パラメータ記述子（ParamDesc）

`Descriptor.GetParamCatalog` が返す 1 要素。

```
ParamDesc {
  id: u16,
  key: str,            // 機械可読キー 例 "fuel.injector_flow_cc_min"
  label: str,          // 表示名 例 "インジェクタ流量"
  category: u8,        // カテゴリ index（GetSchemaInfo の categories と対応）
  vtype: ValueType,
  unit: str,           // 例 "cc/min", "deg", "ms", "%", "kPa"
  scale: f32,          // 物理値 = raw * scale + offset（整数型の固定小数表現用）
  offset: f32,
  min: f32, max: f32,  // 入力検証範囲（物理値）
  default: f32,        // 既定値（物理値）
  digits: u8,          // 表示小数桁
  flags: u8,           // bit0=ReadOnly, bit1=要再起動, bit2=エンジン停止時のみ可
  enum_labels: [str]?, // vtype=enum のときラベル列
}
```

> UI はこの記述子だけでスライダ/入力欄/単位/範囲/列挙ドロップダウンを生成できる。

## 4. カテゴリ

`Descriptor.GetSchemaInfo` の `categories` で、UI のタブ/グループを規定する。
`engine-core` の構成に対応した初期カテゴリ案：

| index | category | 対応（engine-core） |
|-------|----------|---------------------|
| 0 | Engine（エンジン諸元） | 排気量・気筒数・発火順序 |
| 1 | Trigger（トリガー） | 総歯数・欠け歯・デコーダ種別 |
| 2 | Ignition（点火） | 点火マップ・dwell・電圧補正・クランキング |
| 3 | Fuel（燃料） | λマップ・VE table・噴射量・デッドタイム・stoich |
| 4 | Enrichment（補正） | CLT/IAT 補正・加速増量・壁流れ |
| 5 | Idle（アイドル） | PID ゲイン・目標 RPM |
| 6 | Boost（過給） | 目標ブースト・wastegate |
| 7 | Vvt（可変バルタイ） | 目標角・ゲイン |
| 8 | Sensors（センサ） | サーミスタ校正・ADCチャンネル・レンジ |
| 9 | Protection（保護） | RPM リミット・油圧/温度しきい値 |

## 5. テーブル（マップ）

`EngineConfig` 内の 2D マップ（例 `ignition_table[LOAD_BINS][RPM_BINS]`,
`ve_table`, `lambda_table` 等）と 1D 補正（例 `clt_fuel_corr[TEMP_BINS]`）を扱う。

```
TableDesc {
  id: u16,
  key: str,            // 例 "fuel.ve_table"
  label: str,
  category: u8,
  dims: u8,            // 1 または 2
  x_size: u16, y_size: u16,
  x_axis_key: str, y_axis_key: str,  // 軸（bins）の参照
  x_unit: str, y_unit: str, cell_unit: str,
  cell_min: f32, cell_max: f32, cell_digits: u8,
}
```

- `Config.TableGet{ table_id }` で `x_axis[] / y_axis[] / cells[]`（行優先 = `[y][x]`）を取得。
- セル単位編集 `TableSetCell{ ix, iy, value }`、軸編集 `TableSetAxis{ axis, values[] }`。
- 大きなテーブル/全カタログは `02` の断片化で複数フレームに分割送信する。
- `RPM_BINS=16, LOAD_BINS=16, TEMP_BINS=8, VOLT_BINS=8, DWELL_BINS=8`（`engine-core/config.rs`）。

## 6. 編集トランザクション（RAM ステージング → フラッシュ確定）

旧 `Burn` 相当を明示的な 3 状態モデルにする。

```
   ┌────────────┐ ParamSet/TableSet ┌────────────┐ ConfigSave ┌────────────┐
   │  Flash 値  │ ───────────────▶ │  RAM(dirty)│ ─────────▶ │ Flash 確定 │
   └────────────┘                   └────────────┘            └────────────┘
        ▲                                  │ ConfigDiscard
        └──────────────────────────────────┘
   ConfigResetDefaults: RAM を既定値で満たす（保存は別途 ConfigSave）
```

- 書込みはまず RAM（稼働中の `EngineConfig`）へ反映 → 即座に制御へ効く。
- `ConfigStatus.dirty` で未保存有無を提示。`ConfigSave` でフラッシュへ確定。
- `ConfigSave` 応答に `crc` を含め、ホストは `ConfigStatus.flash_crc` と突合して検証できる。
- 一部パラメータ（気筒数・発火順序など）は `flags.bit2`（エンジン停止時のみ）を立て、
  稼働中の `ParamSet` には `Error{ Busy }` を返す。

## 7. スキーマハッシュとキャッシュ

- `schema_hash` はパラメータ/テーブル/テレメトリの全記述子集合から決定的に算出する。
- クライアントは `hash → カタログ` を永続キャッシュし、`Hello.schema_hash` 一致時は
  カタログ取得を省略する（特に BLE の低帯域で有効）。
- ファーム更新でレイアウトが変われば hash が変わり、自動で再取得される。

## 8. 実装メモ

- `codegen` のパラメータ定義（`ConfigField` / `ts_info` の unit/scale/offset/min/max/digits）を
  **TunerStudio INI ではなく**、RDP のパラメータカタログ（`ParamDesc`/`TableDesc`）生成へ転用する。
- デバイス側は param_id → `EngineConfig` フィールドのアクセサ表を生成し、
  オフセット計算を排して型安全に読み書きする。
- カタログ記述子はフラッシュ常駐の静的データ（`&'static`）として持ち、RAM を消費しない。
