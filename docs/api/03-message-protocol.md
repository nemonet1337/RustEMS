# 03. メッセージ層とメッセージカタログ

フレーミング層（`02-transport-and-framing.md`）の `PAYLOAD` に載るメッセージの意味論を定義する。

## 1. メッセージの種別

| 種別 | 方向 | 説明 |
|------|------|------|
| **Request** | ホスト → デバイス | 操作要求。`SEQ` で応答と相関。 |
| **Response** | デバイス → ホスト | 要求 `SEQ` を反映した応答（成功 or `Error`）。 |
| **Event** | デバイス → ホスト | 非同期通知（要求と無関係、`SEQ` は連番）。 |
| **Telemetry** | デバイス → ホスト | 購読中の高頻度フレーム。 |

## 2. ペイロード構造

```
+--------+--------+--------------------+
| KIND   | OP     | BODY               |
| u8     | u16 LE | CBOR or 構造化バイト |
+--------+--------+--------------------+
```

- `KIND` : `0=Request, 1=Response, 2=Event, 3=Telemetry`。
- `OP` : オペレーションコード（下記カタログ）。Response は対応する Request の `OP` を反映。
- `BODY` : メッセージ本体。

### 本体エンコード方針
- **制御系・カタログ系**（可変・構造的なもの）は **CBOR**（`minicbor`, `no_std` 対応）で表現する。
  自己記述に近く、フィールド追加で後方互換を保ちやすい。
- **テレメトリフレーム**（高頻度・固定構造）は CBOR を使わず、購読時に確定した
  チャンネル順の**密パック binary**（スケール済み整数）で送る（帯域最小化）。

## 3. エラー表現

Response が失敗の場合、`OP` を反映したまま `BODY` に共通 `Error` を入れる。

```
Error {
  code: u16,        // 列挙（下表）
  detail: u16,      // 文脈依存（対象 param_id 等）
  message: str?     // 任意・デバッグ用（省略可、組込みでは通常省略）
}
```

| code | 名称 | 意味 |
|------|------|------|
| 0 | Ok | 成功（Response 既定） |
| 1 | UnknownOp | 未対応オペコード |
| 2 | BadRequest | 本体の形式不正 |
| 3 | NotFound | param_id / table_id / channel が存在しない |
| 4 | OutOfRange | 値が min/max を逸脱 |
| 5 | ReadOnly | 書込み不可パラメータ |
| 6 | Busy | エンジン稼働中で不可（例：気筒数変更） |
| 7 | NotSupported | 当該ボード/ビルドで非対応機能 |
| 8 | Fragmentation | 断片再結合の失敗/超過 |
| 9 | VersionMismatch | プロトコル/スキーマ非互換 |
| 10 | Unauthorized | 認証必要（将来用） |

## 4. オペコード・カタログ

OP は **カテゴリ上位バイト**で分類する（`0xCC_NN`）。

| 範囲 | カテゴリ |
|------|----------|
| `0x01_*` | System |
| `0x02_*` | Descriptor |
| `0x03_*` | Config |
| `0x04_*` | Telemetry |
| `0x05_*` | Control |
| `0x06_*` | Diagnostics |

### A. System（`0x01_*`）

| OP | 名称 | Request | Response |
|----|------|---------|----------|
| `0x0101` | Hello | （なし） | `HelloInfo`（下記） |
| `0x0102` | Ping | `{ nonce:u32 }` | `{ nonce:u32, uptime_ms:u32 }` |
| `0x0103` | Reboot | `{ mode:enum(Normal) }` | `Ok` |
| `0x0104` | EnterBootloader | `{ confirm:u32=0xB007 }` | `Ok`（以後 DFU） |

```
HelloInfo {
  proto_major: u8, proto_minor: u8,   // メジャー不一致は切断
  fw_version: str,
  board: enum(Nano|MicroRusefi|Uaefi|Proteus|Huge|Sim),
  mcu: str,                            // 例 "STM32F407"
  cylinders: u8,
  capabilities: u32(bitflags),         // Fuel,Ignition,Boost,Vvt,Knock,Can,Sequential...
  schema_hash: u32,                    // パラメータ/テレメトリカタログのハッシュ
  max_payload: u16,                    // フレーム最大ペイロード
  device_id: [u8;12],                  // 一意 ID（MCU UID 等）
}
```

### B. Descriptor（`0x02_*`） — 詳細は `04-parameter-model.md`

| OP | 名称 | Request | Response |
|----|------|---------|----------|
| `0x0201` | GetSchemaInfo | （なし） | `{ schema_hash, param_count, table_count, categories:[Category] }` |
| `0x0202` | GetParamCatalog | `{ page:u16 }` | `{ page, total_pages, items:[ParamDesc] }` |
| `0x0203` | GetTableCatalog | `{ page:u16 }` | `{ page, total_pages, items:[TableDesc] }` |
| `0x0204` | GetTelemetryCatalog | `{ page:u16 }` | `{ page, total_pages, items:[ChannelDesc] }` |

### C. Config（`0x03_*`） — 詳細は `04-parameter-model.md`

| OP | 名称 | Request | Response |
|----|------|---------|----------|
| `0x0301` | ParamGet | `{ ids:[u16] }` | `{ values:[ParamValue] }` |
| `0x0302` | ParamSet | `{ entries:[(id:u16, value)] }` | `{ results:[(id, code)] }` |
| `0x0303` | TableGet | `{ table_id:u16 }` | `{ x_axis:[..], y_axis:[..], cells:[..] }` |
| `0x0304` | TableSetCell | `{ table_id, ix:u16, iy:u16, value:f32 }` | `Ok` |
| `0x0305` | TableSetAxis | `{ table_id, axis:enum(X\|Y), values:[f32] }` | `Ok` |
| `0x0306` | ConfigSave | （なし） | `{ saved_bytes:u32, crc:u32 }` |
| `0x0307` | ConfigDiscard | （なし） | `Ok`（RAM を flash 値へ戻す） |
| `0x0308` | ConfigResetDefaults | `{ confirm:u32=0xDEFA }` | `Ok` |
| `0x0309` | ConfigStatus | （なし） | `{ dirty:bool, ram_crc:u32, flash_crc:u32 }` |

### D. Telemetry（`0x04_*`） — 詳細は `05-...md`

| OP | 名称 | Request | Response/Push |
|----|------|---------|---------------|
| `0x0401` | Subscribe | `{ channels:[u16], rate_hz:u16 }` | `{ stream_id:u8, layout:[u16] }` |
| `0x0402` | Unsubscribe | `{ stream_id:u8 }` | `Ok` |
| `0x0403` | ReadOnce | `{ channels:[u16] }` | `{ values:[..] }` |
| `0x04F0` | **Frame**(push) | — | `TelemFrame`（KIND=Telemetry, 密パック） |

### E. Control（`0x05_*`） — 詳細は `05-...md`

| OP | 名称 | Request | Response |
|----|------|---------|----------|
| `0x0501` | BenchTest | `{ target:enum, index:u8, on_ms:u16, off_ms:u16, count:u16 }` | `Ok` |
| `0x0502` | SetOverride | `{ target:enum, value:f32, timeout_ms:u16 }` | `Ok` |
| `0x0503` | ClearOverride | `{ target:enum }` | `Ok` |
| `0x0504` | Calibrate | `{ routine:enum, args:[f32] }` | `{ result:[f32] }` |

### F. Diagnostics（`0x06_*`） — 詳細は `05-...md`

| OP | 名称 | Request | Response/Push |
|----|------|---------|---------------|
| `0x0601` | GetFaults | （なし） | `{ faults:[Fault] }` |
| `0x0602` | ClearFaults | `{ mask:u32 }` | `{ cleared:u16 }` |
| `0x06F0` | **Event**(push) | — | `Event`（KIND=Event） |

## 5. 代表シーケンス

### 初回接続〜カタログ取得
```
Host → Request(Hello)
Dev  → Response(Hello, HelloInfo{ schema_hash=H, ... })
（Host 側キャッシュに H が無ければ）
Host → Request(GetSchemaInfo) ; GetParamCatalog(page..) ; GetTelemetryCatalog(page..)
Dev  → Response(... カタログ ...)        # ページ毎に断片化可
Host : カタログを schema_hash=H で永続キャッシュ
```

### マップ編集〜保存
```
Host → Request(TableGet{ ve_table })          → Response(axes+cells)
Host → Request(TableSetCell{ ix, iy, value }) → Response(Ok)   # RAM 反映
Host → Request(ConfigStatus)                  → Response(dirty=true)
Host → Request(ConfigSave)                    → Response(saved, crc)
```

### テレメトリ購読
```
Host → Request(Subscribe{ [rpm,clt,iat,map,lambda], 25Hz })
Dev  → Response(stream_id=1, layout=[...])
Dev  → Telemetry(Frame: 連番 + 密パック値)   # 以後 25Hz で push
...
Host → Request(Unsubscribe{ stream_id=1 })    → Response(Ok)
```

### 非同期イベント
```
（任意のタイミングで）
Dev  → Event{ kind=Knock, cylinder=3, retard_deg=4.0, ts_ms=... }
Dev  → Event{ kind=ProtectionCut, reason=Overboost, ts_ms=... }
```

## 6. バージョニング規約

- `proto_major` 不一致 → 接続不可（破壊的変更）。`proto_minor` はオペコード追加等の後方互換拡張。
- 未知 `OP` には `Error{ UnknownOp }` を返し、クライアントは機能フォールバックする。
- `schema_hash` は `EngineConfig` レイアウト＋カタログ定義から導出。変化時のみカタログ再取得。
