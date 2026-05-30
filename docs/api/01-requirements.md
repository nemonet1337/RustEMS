# 01. 目標・スコープ・必要な API カテゴリ

「ハードウェア ⇔ PC / スマホを USB / Bluetooth で接続し、各種パラメータや機能を設定する」
ために、**どのような API が必要か**を洗い出す。

## 1. 利用シナリオ（ユースケース）

| # | シナリオ | 必要になる機能 |
|---|----------|----------------|
| U1 | スマホアプリで初回接続し、ECU が何者かを知る | デバイス識別・能力取得 |
| U2 | PC ツールで燃調マップ(VE table)を編集して書き込む | パラメータ/テーブル読み書き |
| U3 | 編集内容を電源を切っても保持する | フラッシュ保存 / 破棄 / 既定値リセット |
| U4 | エンジン始動中にリアルタイムで RPM・水温・λ を表示する | テレメトリ購読 |
| U5 | 整備時にインジェクタ/コイルを単体テスト発火する | アクチュエータ ベンチテスト |
| U6 | TPS の全閉/全開を学習させる | キャリブレーション操作 |
| U7 | 故障コード(DTC)を確認して消去する | 診断（フォルト一覧/クリア） |
| U8 | ノック検出・保護作動を即時に画面へ出す | 非同期イベント通知 |
| U9 | ファーム更新のため DFU/ブートローダへ入る | システム制御 |
| U10 | アプリ側でパラメータ一覧を動的に描画する（INI 不要） | スキーマ（カタログ）取得 |

## 2. 機能要件

- F1: デバイスの**識別情報・能力**を構造化して取得できること。
- F2: パラメータを**名前・型・単位・範囲・既定値つき**で列挙（カタログ）できること。
- F3: パラメータを **ID 指定で個別/一括** 読み書きできること（オフセット非露出）。
- F4: ルックアップテーブル（1D/2D マップ）を**軸・セル単位**で読み書きできること。
- F5: 変更を **RAM ステージング → フラッシュ確定 / 破棄 / 既定値復帰** で管理できること。
- F6: テレメトリを**チャンネル選択・レート指定で購読**し、push 配信を受けられること。
- F7: アクチュエータの**ベンチテスト**と、点火/燃料カット等の**一時オーバーライド**ができること。
- F8: **故障コードの取得・消去**ができること。
- F9: フォルト/ノック/保護作動などの**非同期イベント**を受信できること。
- F10: **再起動 / ブートローダ遷移**ができること。
- F11: 同一メッセージ意味論を **USB と Bluetooth の両方**で利用できること。

## 3. 非機能要件

- N1: デバイス側は `no_std` / `heapless` / ヒープ禁止 / `unsafe` 禁止（`workspace.lints` 準拠）。
- N2: BLE の MTU（最小 ~20B、交渉後 ~244B）でも成立する**チャンク分割**を備える。
- N3: 通信誤りに対する **CRC + フレーム同期回復**（自己同期）。
- N4: **要求/応答の相関**（シーケンス番号）と**タイムアウト**を持つ。
- N5: **プロトコル版・スキーマ版**でバージョン非互換を検出できる。
- N6: 1 つの物理リンクの帯域内で、テレメトリ push と設定 RPC が**多重化**できる。
- N7: スループット目安 — USB ≥ 数百 kB/s、BLE ≥ 数 kB/s（テレメトリ 10〜50 Hz）。

## 4. 必要な API カテゴリ（全体像）

新 API は次の 6 カテゴリで構成する。詳細メッセージは `03-message-protocol.md` に定義する。

### A. システム / ハンドシェイク（System）
- `Hello`：プロトコル版・ファーム版・ボード・気筒数・能力フラグ・スキーマハッシュ・最大ペイロード。
- `Ping`：キープアライブ / 死活監視。
- `Reboot` / `EnterBootloader`：システム制御。

### B. スキーマ / カタログ（Descriptor）
- `GetSchemaInfo`：スキーマ版・ハッシュ・パラメータ総数・カテゴリ一覧。
- `GetParamCatalog(page)`：パラメータ記述子の一覧（ページング）。
- `GetTelemetryCatalog`：テレメトリchの記述子（id/名前/単位/型/スケール）。
- クライアントはハッシュでカタログをキャッシュし、変化時のみ再取得する。

### C. パラメータ / 設定（Config）
- `ParamGet` / `ParamSet`（単一・一括、ID 指定）。
- `TableGet` / `TableSetCell` / `TableSetAxis`（マップ編集）。
- `ConfigSave`（RAM→Flash）/ `ConfigDiscard` / `ConfigResetDefaults` / `ConfigIsDirty`。

### D. テレメトリ（Telemetry）
- `TelemSubscribe(channels, rate_hz)` / `TelemUnsubscribe`。
- `TelemReadOnce(channels)`：単発取得。
- `TelemFrame`（デバイス→ホストの push）。

### E. 制御 / アクション（Control）
- `BenchTest(target, index, duration, repeat)`：インジェクタ/コイル/燃料ポンプ/ファン/アイドル。
- `SetOverride(target, value)` / `ClearOverride`：点火カット・燃料カット・アイドル位置 等。
- `Calibrate(routine, args)`：TPS 全閉/全開、その他学習。

### F. 診断 / イベント（Diagnostics）
- `GetFaults` / `ClearFaults`：DTC 取得・消去。
- `Event`（push）：フォルト set/clear、ノック、保護作動、リミットモード遷移。

## 5. スコープ外（将来検討）

- セキュリティ（BLE ペアリング/認証トークン）。v1 ではフィールドを予約のみ。
- 完全なデータロガー（SD/フラッシュ格納のダウンロード）。テレメトリ購読で代替し、別途設計。
- Lua 等スクリプトの転送。`docs/feature-coverage-todo.md` の Low に準ずる。

## 6. 既存実装との対応（移行の手掛かり）

| 旧（TunerStudio 互換） | 新（RDP） |
|------------------------|-----------|
| `Hello 'S'` / `GetFirmwareVersion 'V'` | `System.Hello` に統合・構造化 |
| `ReadPage 'R'` / `WriteChunk 'C'`（page/offset） | `Config.ParamGet/Set`・`TableGet/Set`（ID） |
| `Burn 'B'` | `Config.ConfigSave` |
| `OutputChannels 'O'`（固定20B ポーリング） | `Telemetry.Subscribe` + `TelemFrame`（push） |
| `Execute 'X'`（サブコマンド） | `Control.BenchTest / SetOverride / Calibrate` |
| `CrcCheck 'k'` | `Config` の CRC は `GetSchemaInfo`/保存時に内包 |

`engine-core/src/comms/output.rs` の `OutputChannels` と `sensors::SensorData`、
`config.rs` の `EngineConfig` が、それぞれテレメトリカタログとパラメータカタログの初期ソースになる。
