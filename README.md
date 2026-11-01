# ddt-can-bridge

PCのUSBからWaveshare RP2350-CAN経由で、DDT M1502Aモーター2台（CAN ID 1、2）へ速度指令を中継します。
設計は [docs/can_bridge_design.md](docs/can_bridge_design.md) を参照してください。

**実機では未検証です。** プロトコルはM15シリーズ（M1502D）の資料に基づきます。

## 必要なもの

- Pico SDK 2.3.1（`PICO_SDK_PATH`）、CMake、Ninja、arm-none-eabi-gcc
- Python 3.10以上（追加ライブラリなし。macOS/Linuxのみ）

## テスト

```sh
cmake -S tests -B build/tests -G Ninja
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
(cd host && python3 -m unittest)
```

## ビルド

```sh
cmake -S firmware -B build/firmware -G Ninja
cmake --build build/firmware
```

BOOTボタンを押しながらUSB接続し、出てきたドライブに `build/firmware/ddt_can_bridge.uf2` をコピーします。

## 配線

| モーター | 接続先 |
|---|---|
| CAN High（黄） | 基板 CANH |
| CAN Low | 基板 CANL |
| GND | 24V電源のマイナス。基板のGNDとも共通化 |
| V+（赤） | 24V電源のプラス。**基板には絶対に接続しない** |

色の記載が資料間で異なるため、CAN LowとGNDは通電前に確認してください。

## 使い方

まず動かさずに通信を確認します。

```sh
python3 host/ddt_bridge.py /dev/cu.usbmodemXXXX monitor
```

次に車輪を浮かせて、低速で確認します。

```sh
python3 host/ddt_bridge.py /dev/cu.usbmodemXXXX run 10 10 --seconds 2
```

## 安全動作

- **基板はUSB給電のため、USBが抜けると停止処理を実行できません。24Vをすぐ遮断できる非常停止を用意してください。**
- 基板はDISABLED中、0指令と無効化を100ms周期で送り続け、明示的なENABLEまで動かしません。
- 指令200ms途絶、フィードバック100ms途絶、モーター故障、速度モード離脱、CAN送信失敗で、速度0を500ms送ってから無効化します。
- 停止後に自動復帰はしません。
- ファームが停止した場合は100msのウォッチドッグでリセットします。
- 速度上限は検証前のため30rpmです（ファームと`host/ddt_bridge.py`の両方で定義）。
- 上限速度などの調整値は `firmware/src/bridge_controller.hpp` にあります。
