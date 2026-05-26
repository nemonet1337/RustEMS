| 仕様項目          | Nano [wiki]     | Huge [wiki]      | uaEFI [wiki]  | Proteus [wiki] (現行F7) | Proteus 旧 (0.2前) [wiki] | microRusEFI [wiki] (現行F4) | microRusEFI 旧 (初期) [wiki] |
| ------------- | --------------- | ---------------- | ------------- | --------------------- | ----------------------- | ------------------------- | ------------------------- |
| MCU           | STM32F4         | STM32F4          | STM32F4       | STM32F767             | STM32F4                 | STM32F407                 | STM32F4                   |
| 最大推奨気筒        | 2 (バッチ4可)       | 12 (デュアルETB)     | 6 (デュアルETB)   | 12 (デュアルETB)          | 12                      | 4 (シングルETB)               | 4                         |
| インジェクタ出力      | 8               | 12 High-Z        | 6             | 16 lowside (3A限)      | 16 lowside (3A限)        | 4                         | 4 High-Z                  |
| イグニション出力      | スマート/ダム         | 12 logic         | 6 smart       | 12 (5V/100mA)         | 12 (5V/100mA)           | 4 logic level             | 4 logic level             |
| HS出力 (12V)    | -               | -                | -             | 4x 1A                 | 4x 1A                   | -                         | -                         |
| LS追加出力        | -               | 4 H-bridge       | 4-6           | 複数                    | 複数                      | -                         | 2 high-cur + 4 low        |
| アナログ入力        | 複数              | 13 GP + 2 therm  | 9+2           | 12 GP + 4 therm       | 11 GP + 4 therm         | ~10                       | ~10 (0-5V)                |
| デジタル入力 (Hall) | 複数              | 5                | 複数            | 6                     | 6                       | 2x Hall + 1 VR            | Secondary Hall            |
| VR入力          | 1+              | 3                | 2             | 2                     | 2 (dual VR)             | 1                         | 1 VR/Hall                 |
| WBOコントローラ     | -               | デュアル             | 内蔵            | 内蔵                    | 内蔵                      | -                         | -                         |
| ノック入力         | -               | デュアル             | 内蔵            | 内蔵                    | 内蔵                      | -                         | -                         |
| DBW/ETB       | -               | デュアル             | デュアル          | デュアル                  | デュアル                    | シングル                      | On-board DBW              |
| CANバス         | 1-2             | デュアル             | 最大2           | デュアル                  | デュアル                    | 1                         | 1                         |
| Baro/MAP      | -               | デジタル内蔵           | デジタル内蔵        | 内蔵可能                  | -                       | -                         | -                         |
| SDカード         | あり              | あり               | あり            | あり                    | あり                      | あり                        | USB/SD                    |
| コネクタ          | Superseal 26pin | Superseal 120pin | Molex miniFit | TE Ampseal 93pin      | TE Ampseal              | 48pin waterproof          | 48-pin                    |
| サイズ/PCB       | 超小型             | 大型               | 100x100mm 4層  | 135x82.5mm 4層         | 135x82.5mm              | 小型アルミケース                  | 小型                        |
| 防水            | オプション           | オプション            | -             | IP68                  | IP68                    | 防水コネクタ                    | -                         |
| Bluetooth     | -               | JDY-33 opt       | -             | -                     | -                       | -                         | -                         |
| Protoエリア      | -               | -                | あり            | -                     | -                       | -                         | -                         |
| Flex Fuel     | あり              | あり               | あり            | あり                    | あり                      | あり                        | -                         |
| その他特記事項       | 単/2気筒最適         | 最上位              | 低価格           | 高出力                   | 大型ユニット                  | オープンHW                    | VVT対応                     |