# RustEMS Device API 設計ドキュメント

TunerStudio 互換バイナリプロトコルを廃止し、ハードウェア（ECU）と PC / スマートフォンを
**USB / Bluetooth** で接続して各種パラメータ・機能を設定するための、新しい自己記述型 API を
ゼロから設計する。

このディレクトリには新 API（コードネーム **RDP: RustEMS Device Protocol**）の設計資料を置く。

## なぜ TunerStudio 互換をやめるのか

現状の `protocol` クレートは TunerStudio 互換であり、以下の制約がある。

- **メモリレイアウト密結合**: `ReadPage(page, offset, length)` / `WriteChunk(page, offset, data)` の
  ように生のメモリオフセットでアクセスする。クライアントは別途 INI ファイルで
  「どのオフセットが何のパラメータか」を知る必要があり、ファームとツールのバージョンずれで壊れる。
- **自己記述性がない**: バイナリ image を読んでも、スキーマ無しでは意味が分からない。
- **固定 20 バイトの output channels**: テレメトリ項目を増減するとレイアウトが破壊的に変わる。
- **単バイト opcode**: 拡張余地が乏しく、リクエスト/レスポンスの相関やイベント通知の概念がない。
- **PC 専用前提**: TCP ゲートウェイ(29001) 経由が中心で、スマホ / BLE を想定していない。

新 API はこれらを解消し、**名前と型を持つパラメータ**・**スキーマ自己記述**・
**購読型テレメトリ**・**USB と Bluetooth 両対応**を最初から備える。

## ドキュメント構成

| ファイル | 内容 |
|----------|------|
| [`01-requirements.md`](./01-requirements.md) | 目標・スコープ・必要な API カテゴリの洗い出し（まず読む） |
| [`02-transport-and-framing.md`](./02-transport-and-framing.md) | USB CDC / Bluetooth(SPP/BLE) トランスポートとフレーミング層 |
| [`03-message-protocol.md`](./03-message-protocol.md) | メッセージ層・全メッセージカタログ・シーケンス |
| [`04-parameter-model.md`](./04-parameter-model.md) | パラメータ / テーブル / スキーマ（設定モデル）の表現 |
| [`05-telemetry-control-diagnostics.md`](./05-telemetry-control-diagnostics.md) | テレメトリ購読・アクチュエータ制御・診断/イベント |

## 設計原則（サマリ）

1. **自己記述（Self-describing）** — デバイスがパラメータ・テレメトリの全カタログを返す。
   クライアントは事前の INI を必要としない。
2. **レイアウト非依存** — パラメータは安定した数値 ID で参照し、メモリオフセットを露出しない。
3. **トランスポート抽象** — 1 つのメッセージ層を、ストリーム系（USB CDC / Bluetooth SPP）と
   パケット系（BLE GATT）の両方の上に載せる。
4. **組込み制約準拠** — デバイス側は `no_std` + `heapless`、ヒープ禁止、`unsafe` 禁止。
   BLE の小さい MTU でも動くようチャンク分割を前提にする。
5. **バージョニングと能力ネゴシエーション** — プロトコル版・スキーマハッシュ・機能フラグで
   前方/後方互換を確保する。
6. **要求/応答 + 非同期イベント + ストリーム** — 単純なポーリングではなく購読型を採用する。

## 実装ロードマップ（提案）

詳細は各ドキュメント末尾を参照。要点のみ：

- 新クレート `device-api`（`no_std`）にメッセージ型とコーデックを定義し、
  デバイス側（`engine-core/comms`）とホスト側（`client`）で共有する。
- 既存 `protocol` / `client` / `cli` は移行期間中は併存させ、最終的に置き換える。
- `codegen` のパラメータ定義をスキーマ（パラメータカタログ）生成にも再利用する。

## 実装状況

| 層 | 場所 | 状態 |
|----|------|------|
| CRC-16/CCITT | `device-api/src/crc16.rs` | ✅ 実装・テスト済 |
| COBS | `device-api/src/cobs.rs` | ✅ 実装・テスト済 |
| フレーム（VER/FLAGS/SEQ/LEN/PAYLOAD/CRC16, 断片化フラグ） | `device-api/src/frame.rs` | ✅ 実装・テスト済（送信側断片化 `encode_message` 含む） |
| メッセージヘッダ・オペコード・エラー/値型の列挙 | `device-api/src/message.rs` | ✅ 実装・テスト済 |
| 断片化の再結合ヘルパ | `device-api/src/defrag.rs` | ✅ 実装・テスト済 |
| CBOR ボディコーデック（全カテゴリのボディ型） | `device-api/src/cbor.rs` | ✅ 実装・テスト済 |
| パラメータ/テーブル/テレメトリの静的カタログ + schema_hash | `engine-core/src/params.rs`, `engine-core/src/comms/telemetry.rs` | ✅ 実装・テスト済（`codegen` 転用は不要になり静的カタログ方式に変更） |
| デバイス側ハンドラ（全オペコード：System/Descriptor/Config/Telemetry/Control/Diagnostics） | `engine-core/src/comms/rdp.rs` | ✅ 実装・テスト済 |
| テレメトリ購読ストリーム（密パック push） | `engine-core/src/comms/telemetry.rs` | ✅ 実装・テスト済 |
| フォルト（DTC）/ 非同期イベント | `engine-core/src/comms/faults.rs` | ✅ 実装・テスト済 |
| オーバーライド / ベンチテスト / キャリブレーション | `engine-core/src/comms/control.rs` | ✅ 実装・テスト済（制御ループへの作用は sim で配線） |
| RAM ステージング→フラッシュ確定トランザクション | `engine-core/src/comms/rdp.rs` | ✅ 実装（実機フラッシュ書込みドライバは未配線） |
| UART 上の RDP/TS デュアルスタック | `stm32/src/main.rs` `comms_task` | ✅ 配線済（先頭バイト 0x00=TS / 非0x00=RDP のルーティング） |
| PC シミュレータの RDP TCP サーブモード | `sim`（`--serve <port>`） | ✅ 実装 |
| ホスト側クライアント | `client/src/rdp.rs` / `cli`（`rdp` サブコマンド） | ✅ 実装・テスト済 |
| USB CDC-ACM / Bluetooth トランスポート | `stm32` / host | ⏳ 未着手（現状は UART / TCP） |

> `device-api` は `no_std`・ヒープ無し・`unsafe` 禁止・`panic`/`unwrap`/`expect` 不使用。
> `thumbv7em-none-eabihf` ビルドと clippy（workspace lints）を確認済み。
