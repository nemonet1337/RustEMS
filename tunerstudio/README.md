# RustEMS — 実機への書き込みとPCチューニング

## 1. ファームウェアのビルド

ボードごとに `cyl-N` と `fuel-fi`/`fuel-carb` を選んでビルドします。

```bash
# microRusEFI（4気筒・FI）
cargo build-arm -p rusefi-stm32 --release --no-default-features --features stm32f4,cyl-4,fuel-fi

# Proteus（12気筒）
cargo build-arm -p rusefi-stm32 --release --no-default-features --features stm32f7,cyl-12,fuel-fi

# uaEFI（6気筒）
cargo build-arm -p rusefi-stm32 --release --no-default-features --features uaefi,cyl-6,fuel-fi

# Huge（12気筒）
cargo build-arm -p rusefi-stm32 --release --no-default-features --features stm32f4-huge,cyl-12,fuel-fi

# Nano（2気筒、4気筒はバッチ）
cargo build-arm -p rusefi-stm32 --release --no-default-features --features stm32f4-nano,cyl-2,fuel-fi
```

`memory.x` は `build.rs` がボードのMCUに合わせて自動生成します
（F767=512K RAM / F407=192K RAM）。

## 2. 実機への書き込み（probe-rs）

`.cargo/config.toml` の runner はデフォルトで `STM32F407VGTx` を指定しています。
ボードのMCUに合わせて `--chip` を上書きしてください。

| ボード | MCU | probe-rs chip |
|--------|-----|---------------|
| microRusEFI | STM32F407 | `STM32F407VGTx` |
| uaEFI | STM32F407 | `STM32F407VGTx` |
| Huge | STM32F407 | `STM32F407VGTx` |
| Nano | STM32F4 | `STM32F407VGTx` |
| Proteus | STM32F767 | `STM32F767ZITx` |

```bash
probe-rs run --chip STM32F767ZITx \
  target/thumbv7em-none-eabihf/release/rusefi-stm32
```

> MCU表記は実機rusEFIハードに準拠しています（Proteus=F767, microRusEFI/uaEFI/Huge=F407）。

## 3. PCチューニング（TunerStudio）

ファームウェアは **USART1（TX=PA9, RX=PA10, 115200 8N1）** で
TunerStudio互換のバイナリプロトコルを話します。

1. USB-シリアル変換器で USART1 とPCを接続。
2. TunerStudio で `tunerstudio/rustems.ini` を読み込んでプロジェクト作成。
3. シリアルポート 115200 で接続。

接続すると以下のライブゲージが表示されます（`[OutputChannels]`）:
RPM / 水温 / 吸気温 / MAP / TPS / バッテリ電圧 / λ / 噴射パルス幅 / 点火進角 /
ステータスビット（spark cut, sequential）。

### 現状の制限

- **出力チャンネル（ライブデータ）は完全動作**します。
- **定数ページ（チューン編集）** は現状256バイトのRAMページで、read/write/burn は
  バイト単位で動作しますが、個々のフィールドはまだ `EngineConfig` にマッピング
  されていません。実パラメータのライブ編集は次ステップです
  （`docs/feature-coverage-todo.md` 参照）。

## プロトコル対応コマンド

| コマンド | 機能 | 実装 |
|----------|------|------|
| `S` | hello（署名取得） | ✅ |
| `V` | ファーム版数 | ✅ |
| `F` | プロトコル版数 | ✅ |
| `O` | 出力チャンネル読出 | ✅ |
| `R` | ページ読出 | ✅ |
| `C` | ページ書込 | ✅ |
| `B` | バーン（確定） | ✅ |
| `k` | ページCRC32検査 | ✅ |
