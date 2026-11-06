# ddt-can-bridge

PCのUSBからWaveshare RP2350-CAN経由で、DDT M1502Aモーター2台（CAN ID 1、2）へ速度指令を中継します。
設計は [docs/can_bridge_design.md](docs/can_bridge_design.md) を参照してください。

M1502A 2台を無負荷、10rpm、2秒で駆動して動作を確認しました（2026-11-01）。異常時の停止動作と高速域、負荷時は未検証です。プロトコルはM15シリーズ（M1502D）の資料に基づきます。

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

## ROS 2

`/cmd_vel`（`geometry_msgs/Twist`）を左右の車輪速度へ変換するノードです。ROS 2 JazzyをRoboStack（pixi）で動かします。

```sh
pixi install
pixi run test                 # ビルドしてテスト。コード変更後も実行

pixi run drive                # ノード起動。この時点ではモーターは無効
```

別のターミナルで操作します。別のターミナルでも必ず `pixi run` か `pixi shell` を使います（ROSドメインIDを揃えるため）。

```sh
# 有効化（false で停止）
pixi run ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"

# キーボード操作
pixi run teleop

# パラメータの変更例
pixi run ros2 param set /diff_drive_node max_wheel_rpm 50.0
```

| パラメータ | 初期値 | 内容 |
|---|---|---|
| `port` | 空 | 空なら接続中の基板を自動検出（2枚以上ならエラー）。起動時のみ |
| `wheel_radius_m` | 0.0935 | 車輪半径。カタログ直径187mmから算出 |
| `track_width_m` | 0.26 | 左右の車輪間隔。2026-11-06 に実機で26cm（回転中心から各車輪13cm）と計測 |
| `left_motor_id` | 1 | 左車輪のモーターID。もう一方が右。有効化中は変更不可 |
| `left_sign` / `right_sign` | 1 / -1 | 前進時の回転方向。有効化中は変更不可 |
| `max_wheel_rpm` | 100.0 | 車輪の最大速度（上限210）。飽和時は左右の比を保つ |
| `max_acceleration_rpm_per_s` | 100.0 | 加減速の制限 |
| `command_timeout_s` | 0.5 | `/cmd_vel` が途絶えたら目標を0へ（上限5秒）。減速は200rpm/s以上 |

ノードは実行中の変更を受け付けますが、ファイルには保存しません。確定した値は `ros2/ddt_diff_drive/config/diff_drive.yaml` に書いてください。`pixi run drive` はこのファイルを直接読みます。

## 安全動作

- **基板はUSB給電のため、USBが抜けると停止処理を実行できません。24Vをすぐ遮断できる非常停止を用意してください。**
- 基板はDISABLED中、0指令と無効化を100ms周期で送り続け、明示的なENABLEまで動かしません。
- 指令200ms途絶、フィードバック100ms途絶、モーター故障、速度モード離脱、CAN送信失敗で、速度0を500ms送ってから無効化します。
- 停止後に自動復帰はしません。
- ファームが停止した場合は100msのウォッチドッグでリセットします。
- 速度上限はモーターの指令範囲いっぱいの210rpmです（ファームと`host/ddt_bridge.py`の両方で定義）。定格は115rpmです。
- 上限速度などの調整値は `firmware/src/bridge_controller.hpp` にあります。
