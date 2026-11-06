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
| `track_width_m` | 0.4 | **仮値。** 左右の車輪間隔を実測して設定 |
| `left_motor_id` | 1 | 左車輪のモーターID。もう一方が右。有効化中は変更不可 |
| `left_sign` / `right_sign` | 1 / -1 | 前進時の回転方向。有効化中は変更不可 |
| `max_wheel_rpm` | 100.0 | 車輪の最大速度（上限210）。飽和時は左右の比を保つ |
| `max_acceleration_rpm_per_s` | 100.0 | 加減速の制限 |
| `command_timeout_s` | 0.5 | `/cmd_vel` が途絶えたら目標を0へ（上限5秒）。減速は200rpm/s以上 |

ノードは実行中の変更を受け付けますが、ファイルには保存しません。確定した値は `ros2/ddt_diff_drive/config/diff_drive.yaml` に書いてください。`pixi run drive` はこのファイルを直接読みます。

## ROS 2 を Docker で動かす（ロボット PC）

ロボット PC には pixi を入れず、Docker イメージで `drive`（ノード）、`joy`（ジョイスティック）、`teleop`（`/joy` から `/cmd_vel`）を動かします。ドメインIDは42、DDSの公開範囲は `LOCALHOST`、転送はUDPのみです（変更は環境変数 `ROS_DOMAIN_ID`、`ROS_AUTOMATIC_DISCOVERY_RANGE`）。

### 配布（ビルド機で実行）

送りたいコミットをビルド機の clone で checkout してから実行します。ビルド機からロボット PC へ鍵認証の ssh が通ることが前提です。

```sh
scripts/deploy.sh <ロボットのSSH先>   # ビルドして docker save | gzip | ssh ... docker load。圧縮後およそ370MB
```

ロボット PC には、イメージに加えて `ros2/compose.yaml` と `ros2/config/` を同じ階層で置きます（例: `~/ddt-ros2/compose.yaml`、`~/ddt-ros2/config/teleop_twist_joy.yaml`）。

### 起動（ロボット PC で、compose.yaml のある場所）

```sh
export DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...   # 省略時は /dev/ttyACM0
docker compose up -d drive joy teleop                # この時点ではモーターは無効
```

動かすには車輪を浮かせて、手動で有効化します（コンテナの起動では有効化しません）。

```sh
docker compose run --rm --no-deps drive ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
```

`enable_button`（初期値は0番、Xbox系のA）を**押している間だけ**、左スティックの上下で前後、左右で旋回します。離すと止まります。無効化は `data: false`、またはコンテナの停止です。

```sh
docker compose logs -f drive teleop     # 状態の確認
docker compose stop                     # SIGTERMで停止処理（DISABLE）を実行
```

- ボタン・軸の番号が違うパッドは、`docker compose run --rm --no-deps joy ros2 topic echo /joy` で確認して `config/teleop_twist_joy.yaml` を直し、`docker compose up -d --force-recreate teleop` で反映します。初期値は低速（0.3 m/s、1.0 rad/s）です。
- USBを抜き差ししたら `docker compose up -d --force-recreate drive` で作り直します（`devices:` は作成時に固定されます）。基板が無いと `drive` はコンテナ作成時に失敗します。
- ModemManager が動いていると、挿した直後に基板へ AT コマンドを送られることがあります。症状が出たら `mmcli -L` と `journalctl -u ModemManager` を確認してください。

## 安全動作

- **基板はUSB給電のため、USBが抜けると停止処理を実行できません。24Vをすぐ遮断できる非常停止を用意してください。**
- 基板はDISABLED中、0指令と無効化を100ms周期で送り続け、明示的なENABLEまで動かしません。
- 指令200ms途絶、フィードバック100ms途絶、モーター故障、速度モード離脱、CAN送信失敗で、速度0を500ms送ってから無効化します。
- 停止後に自動復帰はしません。
- ファームが停止した場合は100msのウォッチドッグでリセットします。
- 速度上限はモーターの指令範囲いっぱいの210rpmです（ファームと`host/ddt_bridge.py`の両方で定義）。定格は115rpmです。
- 上限速度などの調整値は `firmware/src/bridge_controller.hpp` にあります。
