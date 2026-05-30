# rusEFI Rust ハードウェア対応レポート

最終更新: 2026年5月30日

## 1. 対応ボード一覧

| ボード | MCU | feature flag | 最大気筒 | ファームウェアビルド |
|--------|-----|-------------|---------|-----------------|
| microRusEFI | STM32F407ZGT6 | `stm32f4` | 4 | ✅ dev / ✅ release |
| Proteus | STM32F767ZIT6 | `stm32f7` | 12 | ✅ dev / ✅ release |
| uaEFI | STM32F407 | `uaefi` | 6 | ✅ dev / ✅ release |
| Huge | STM32F407 | `stm32f4-huge` | 12 | ✅ dev / ✅ release |
| Nano | STM32F4 | `stm32f4-nano` | 2 (バッチ4可) | ✅ dev / ✅ release |

---

## 2. HALドライバ実装状況（全5ボード）

各ボードは `hal-<board>/src/` に以下のドライバを実装済み（embassy-stm32 使用）。

| ドライバ | microRusEFI | Proteus | uaEFI | Huge | Nano |
|---------|------------|---------|-------|------|------|
| ADC (`adc.rs`) | ✅ | ✅ | ✅ | ✅ | ✅ |
| 点火出力 (`ignition.rs`) | ✅ 4ch | ✅ 12ch | ✅ 6ch | ✅ 12ch | ✅ 2ch |
| インジェクター (`injector.rs`) | ✅ 4ch | ✅ 12ch | ✅ 6ch | ✅ 12ch | ✅ 2ch |
| トリガー入力 (`trigger.rs`) | ✅ | ✅ | ✅ | ✅ | ✅ |
| システムタイマー (`timer.rs`) | ✅ | ✅ | ✅ | ✅ | ✅ |
| CAN (`can.rs`) | ✅ | ✅ | ✅ | ✅ | ✅ |
| UART (`uart.rs`) | ✅ | ✅ | ✅ | ✅ | ✅ |
| SDカード | ⛔ stub | ⛔ stub | ⛔ stub | ⛔ stub | ⛔ stub |

---

## 3. ピン割り当て（ファームウェア実装値）

### 全ボード共通

| 信号 | ピン |
|------|------|
| クランク入力 (EXTI) | PA8 |
| カム入力 (EXTI) | PA5 |
| CAN RX | PD0 |
| CAN TX | PD1 |
| UART TX (TunerStudio) | PA9 |
| UART RX (TunerStudio) | PA10 |

### ADC チャンネル（全ボード共通 ADC1 使用）

| センサー | ピン | 換算 |
|---------|------|------|
| CLT (冷却水温) | PA0 | Steinhart-Hart サーミスタ |
| IAT (吸気温) | PA1 | Steinhart-Hart サーミスタ |
| TPS (スロットル) | PC3 | 0–5V → 0–100% |
| MAP (吸気圧) | PC0 | 0–5V → 0–250kPa |
| Vbatt (電源電圧) | PC1 | × 8.232 倍 |

### 点火出力ピン

| ボード | ピン |
|--------|------|
| microRusEFI (4気筒) | PE14, PE13, PE12, PE11 |
| uaEFI (6気筒) | PE14, PE13, PE12, PE11, PE10, PE9 |
| Huge / Proteus (12気筒) | PE4–PE15 |
| Nano (2気筒) | PE14, PE13 |

### インジェクター出力ピン

| ボード | ピン |
|--------|------|
| microRusEFI (4気筒) | PB6, PB7, PB8, PB9 |
| uaEFI (6気筒) | PB6, PB7, PB8, PB9, PB10, PB11 |
| Huge / Proteus (12気筒) | PF0–PF11 |
| Nano (2気筒) | PB9, PB8 |

> ⚠ Huge / Proteus の12chピン割り当て（PE4–PE15 / PF0–PF11）は実機回路図との
> 照合が未完了です。ADC (PA/PC)、トリガー (PA5/PA8)、CAN (PD0/PD1) との
> 衝突はありませんが、実機での検証が必要です。

---

## 4. メモリマップ（board 別）

`stm32/build.rs` がビルド時に自動生成する `memory.x`。

| MCU | FLASH | RAM |
|-----|-------|-----|
| STM32F407 | 1 MB | 192 KB |
| STM32F767 | 2 MB | 512 KB |

---

## 5. Embassyタスク構成（ファームウェア）

```
main() [Embassy executor]
├── crank_task  — PA8 EXTI割り込み → クランクパルスキュー
├── cam_task    — PA5 EXTI割り込み → カムパルスキュー
├── comms_task  — USART1 TunerStudio通信 (115200 8N1)
└── control_loop — センサー読取 → 点火 → 噴射（インライン非同期）
```

`comms_task` はUART BufferedUartが利用可能な場合のみ起動。
起動できない場合もファームウェアは制御ループを続行する。

---

## 6. TunerStudio連携

- **接続**: USART1（TX=PA9, RX=PA10）、115200 bps 8N1
- **INI定義**: `tunerstudio/rustems.ini`
- **ライブゲージ**: RPM / CLT / IAT / MAP / TPS / バッテリー電圧 /
  Lambda / 噴射パルス幅 / 点火進角 / スパークカット / シーケンシャル状態
- **対応コマンド**: Hello(S), Read(R), Write(C), Burn(B), OutputChannels(O),
  Version(V), CRC(k), Protocol(F)

---

## 7. 未実装（今後の対応項目）

| 項目 | 状況 |
|------|------|
| SDカードロギング | 全ボードでstub（`SdCardPinSet`は定義済み） |
| フラッシュ設定永続化 | `MemoryStorage`実装済・HAL配線未完 |
| 定数ページ↔EngineConfig マッピング | 未実装（ライブ編集の前提） |
| Knock センサー入力 | HAL定義なし（要ドライバ追加） |
| Wideband O2センサー | HAL定義なし |
| ETB (DBW) 制御 | HAL定義なし |

---

*レポート生成: 2026年5月30日 — RustEMS ハードウェア対応確認*
