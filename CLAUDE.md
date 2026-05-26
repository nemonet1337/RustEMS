# rusEFI Rust

オープンソースのエンジン制御ユニット（ECU）ファームウェア。バイク（1-4気筒）・乗用車（4-12気筒）対応のガソリンエンジン用燃料噴射・点火制御システム。

## 技術スタック

- **Rust Edition 2021**（MSRV: 1.95）
- **no_std** エンベデッド環境対応
- **embassy-stm32** - STM32用async runtime
- **heapless** - スタックベースコレクション
- **libm** - no_std数学関数
- **defmt** - 組込み向けログ
- **tokio** - PCシミュレータ・CLI向け非同期ランタイム

## ディレクトリ構成

17クレートのワークスペース構成：

```
rust-efi/
├── engine-core/        # 制御ロジックコア（no_std）
├── hal-sim/           # PCシミュレータHAL
├── hal-stm32-common/  # STM32共通HAL（トレイト/スタブ）
├── hal-microrusefi/   # microRusEFIボードHAL
├── hal-proteus/       # ProteusボードHAL
├── hal-nano/          # NanoボードHAL
├── hal-uaefi/         # uaEFIボードHAL
├── hal-huge/          # HugeボードHAL
├── protocol/          # バイナリプロトコル実装
├── client/            # 高レベルECUクライアント
├── cli/               # コマンドラインツール
├── codegen/           # 設定コード生成
├── sim/               # PCシミュレータバイナリ
├── stm32/             # STM32ファームウェアバイナリ
├── docs/              # ドキュメント
├── hardware/          # ハードウェア設計ファイル
└── old-src/           # 旧C++/Java実装（参考用）
```

## 各ディレクトリの実装内容

### engine-core
- トリガーホイールデコーダ（missing-tooth: 36-1, 60-2, 12-1, 4-1, 24-1, 24-2, 36-2対応）
- 点火制御（dwell制御、スパーク角計算）
- 燃料噴射（パルス幅計算、バッチ噴射）
- アイドル制御、ブースト制御、VVT制御
- 4ストロークエンジンサイクル管理

### HAL層（Hardware Abstraction Layer）
7つの核心トレイト定義：
- `TriggerInput` - トリガー信号入力
- `IgnitionOutput` - イグニッション出力
- `AdcInput` - アナログ入力
- `SystemTimer` - システムタイマー
- `InjectorOutput` - インジェクター出力
- `CanBus` - CAN通信
- `UartPort` - UART通信

**実装状況：**
- `hal-sim`: 全トレイト実装済（PCシミュレータ）
- `hal-stm32-common`: トレイト定義・スタブ実装
- `hal-*`各ボード: ピン定義のみ、ドライバ未実装

### protocol / client / cli
- TunerStudio連携用バイナリプロトコル
- TCP/シリアル通信トランスポート
- CLIツール（hello, read-image, burn, output-channelsコマンド）

### codegen
- TunerStudio用INIファイル生成
- Cヘッダー生成
- Enum文字列変換

## 対応デバイス仕様

詳細は `hardware-compatibles.md` を参照：

| ボード | MCU | 最大気筒 | 主な特徴 |
|--------|-----|----------|----------|
| Nano | STM32F4 | 2 (バッチ4可) | 超小型、単/2気筒最適 |
| Huge | STM32F4 | 12 | デュアルETB、最上位モデル |
| uaEFI | STM32F4 | 6 | デュアルETB、低価格 |
| Proteus | STM32F767 | 12 | デュアルETB、高出力 |
| microRusEFI | STM32F407 | 4 | シングルETB、オープンHW |

## タスクリスト管理

`docs/feature-coverage-todo.md` を常に把握し更新すること：

- **Critical**: HAL実体化、4ストローク完全同期、Sequential injection
- **High**: Sequential injection、CAN communication、OBD2対応
- **Medium**: MAF/Alpha-Nエアマス、各種センサー、ETB制御
- **Low**: Launch control、Traction control、Lua scripting

## 機能網羅性レポート

`docs/feature-coverage-report.md` は実装内容の確認の際に毎度書き直すこと：

- old-src (C++) とRust実装の機能比較マトリックス
- HALトレイト実装状況
- 機能ギャップサマリー
- テスト・ビルド検証結果

## 開発環境・ビルド要件

### 必須ツール
- Rust 1.95+
- target: `thumbv7em-none-eabihf`（STM32向け）
- cargo-embed（書き込みツール）

### 依存関係
```toml
[workspace.dependencies]
embassy-stm32 = "0.6"
tokio = { version = "1", features = ["io-util", "net", "time", "rt", "macros"] }
nom = "7"
clap = { version = "4", features = ["derive"] }
```

## ビルド・テスト手順

### ビルド
```bash
# PCシミュレータ
cargo build -p rusefi-sim

# STM32ファームウェア（microRusEFI）
cargo build -p rusefi-stm32 --features stm32f4,cyl-4,fuel-fi

# CLIツール
cargo build -p rusefi-cli

# ワークスペース全体
cargo build --workspace
```

### テスト
```bash
# コアライブラリテスト
cargo test -p rusefi-core --features cyl-4,fuel-fi --lib

# プロトコルテスト
cargo test -p rusefi-protocol --lib

# クライアントテスト
cargo test -p rusefi-client --lib

# ワークスペース全体
cargo test --workspace
```

### ドキュメント生成
```bash
cargo doc --workspace --no-deps
```

## コーディング規約

### 厳格禁止事項（workspace.lints）
```toml
[workspace.lints.rust]
unsafe_code = "forbid"

[workspace.lints.clippy]
unwrap_used = "deny"
expect_used = "deny"
panic = "deny"
```

### 命名規則
- スネークケース（Rust標準）
- 定数: `UPPER_SNAKE_CASE`
- トレイト: `PascalCase`
- 型パラメータ: 大文字1文字（`T`, `E`）

## アーキテクチャ方針

### no_std構成
- `engine-core`は完全にno_std（エンベデッド対応）
- ヒープアロケーション禁止（`heapless`使用）
- 数学関数は`libm`使用

### HAL抽象化層
- トレイトベースのハードウェア抽象化
- PCシミュレータ（hal-sim）とSTM32ファームウェアの両対応
- embassy-stm32統合による非同期ドライバ

### エンジンサイクル管理
- 360°クランクサイクルと720°カムサイクルの区別
- 気筒位相追跡（Sequential injection実装の前提）
- 発火順序（firing order）の柔軟な設定

### コミュニケーション
- TunerStudioとのバイナリプロトコル互換
- 将来的なCAN OBD2対応の土台整備

## 関連ドキュメント

- `hardware-compatibles.md` - 対応ハードウェア仕様詳細
- `docs/feature-coverage-todo.md` - 実装TODOリスト（優先度付き）
- `docs/feature-coverage-report.md` - 機能網羅性レポート（定期的に更新）
