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
| `joy_enable_button` | 7 | `/joy` でこのボタンを押すと有効化（Start を仮定）。押した瞬間だけ反応し、デッドマンかスティックが中立でないと拒否。-1 でパッドからの有効化だけ無効（無効化は生きる） |
| `joy_disable_button` | 6 | このボタンを押すと無効化（Back を仮定）。常に効く。`joy_enable_button` が 0 以上なら -1 不可 |
| `joy_deadman_button` | 0 | `teleop_twist_joy.yaml` の `enable_button` と同じ番号にする。押している間は有効化しない |
| `joy_neutral_axes` | `[0, 1]` | 有効化時に中立（絶対値 0.1 以下）を要求する軸。`teleop_twist_joy.yaml` の `axis_linear.x`・`axis_angular.yaw` と同じ番号にする |

3つのボタン番号は互いに異なる必要があります。最初に届いた `/joy` は「押した瞬間」とみなしません（Start を押したまま `joy` を再起動しても有効化されません）。

ノードは実行中の変更を受け付けますが、ファイルには保存しません。確定した値は `ros2/ddt_diff_drive/config/diff_drive.yaml` に書いてください。`pixi run drive` はこのファイルを直接読みます。

## ロボット PC で動かす（Docker）

ロボット PC には pixi を入れず、Docker イメージで `drive`（ノード）、`joy`（ジョイスティック）、`teleop`（`/joy` から `/cmd_vel`）を動かします。Mac では実機を運転できません（Mac の Docker は arm64 で、ビルドと試験のみ）。

### 配布（ビルド機で実行）

Mac からはビルド機へ `ssh <ビルド機>` で入って実行します。送りたいコミットをビルド機の clone で checkout し、作業ツリーを clean にしてから実行します（未 commit の変更があると中止します）。ビルド機からロボット PC へ鍵認証の ssh が通り、両方で `docker` が sudo なしで使えることが前提です。

```sh
scripts/deploy.sh <ロボットのSSH先>
```

ビルド、`docker save | gzip | ssh ... docker load`、`latest` タグ付け、`~/ddt-can-bridge/` への `compose.yaml` と `config/` の送付までを行い、最後に起動コマンドを表示します。圧縮後のイメージはおよそ370MBで、LAN内の1回転送です（所要時間は実測して追記してください）。ビルド機とロボット PC の `uname -m` が違うと中止します。経路だけ試すときは、ビルド機自身（`nyanziba@localhost`）を送り先にします。

### 起動（ロボット PC の `~/ddt-can-bridge/`）

```sh
export DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...   # 省略時は /dev/ttyACM0
docker compose up -d drive joy teleop                # この時点ではモーターは無効
```

動かすには車輪を浮かせて、パッドで有効化します（コンテナの起動では有効化しません）。**Start で有効化、Back で無効化**します（ボタン番号は下の `/joy` で確認）。デッドマン（A）を離し、左スティックを中立に戻してから Start を押してください。そうでないと `enable ignored: ...` の警告が出て有効化しません。

パッドが使えないときの代替として、サービスでも有効化できます（中立条件は見ません）。`exec` は entrypoint を通らないため、`docker-entrypoint.sh` 経由で呼びます。

```sh
docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
```

`enable_button`（初期値は0番、Xbox系のA）を**押している間だけ**、左スティックの上下で前後、左右で旋回します。離すと止まります。無効化は Back、`data: false`、またはコンテナの停止です。

```sh
docker compose logs -f drive teleop     # 状態の確認
docker compose stop                     # SIGTERMで停止処理（DISABLE）を実行（Ctrl-Cや down でも同じ）
```

キーボードで動かすときは、ジョイスティックの代わりに次を使います。

```sh
docker compose run --rm teleop-keyboard
```

- ボタン・軸の番号が違うパッドは、`docker compose exec joy /ws/ros2/docker-entrypoint.sh ros2 topic echo /joy` で確認して `config/teleop_twist_joy.yaml` を直し、`docker compose up -d --force-recreate teleop` で反映します。Start・Back・デッドマンの番号と中立を見る軸は `ros2/ddt_diff_drive/config/diff_drive.yaml`（`joy_*`）にもあるので、**両方を同じ番号に直してください**（片方だけ直すと、有効化の誤操作防止が黙って効かなくなります）。初期値は低速（0.3 m/s、1.0 rad/s）です。
- `joy` は `/dev/input` を丸ごと読めるため、コンテナ内からホストのキーボード入力も読めます。専用のロボット PC を前提にしています。
- USBを抜き差ししたり、基板がリセットしたりしたら、`drive` はコンテナごと終了します（再起動しません）。挿し直して `docker compose up -d drive` で復帰します。`devices:` は作成時に固定されるためです。基板が無いと `drive` はコンテナ作成時に失敗します。
- ModemManager が動いていると、挿した直後の `ttyACM` に AT コマンドを送ることがあります。ノードの排他ロック（TIOCEXCL）は open 後にしか効きません。`mmcli -L` と `journalctl -u ModemManager` で確認し、症状が出たら `sudo systemctl mask --now ModemManager` してください。

### ROS 2 の通信範囲

- 既定は `ROS_DOMAIN_ID=42`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` です。同じ LAN の別 PC のトピックは見えず、`/cmd_vel` も受けません。別 PC（Mac の pixi など）から操作するときだけ、起動前に `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` を付けて `docker compose up -d` し直します。
- LOCALHOST では Fast DDS が同じホスト内の通信に共有メモリを使います（`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` は効きません）。そのため compose は全サービスに `ipc: host` を付けています。compose を使わず `docker run` で同じドメインに参加するコンテナには `--ipc host` が要ります。
- イメージの `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` が効くのは、`SUBNET` などに上書きしたときです。

## 安全動作

- **基板はUSB給電のため、USBが抜けると停止処理を実行できません。24Vをすぐ遮断できる非常停止を用意してください。**
- 基板はDISABLED中、0指令と無効化を100ms周期で送り続け、明示的なENABLEまで動かしません。
- 指令200ms途絶、フィードバック100ms途絶、モーター故障、速度モード離脱、CAN送信失敗で、速度0を500ms送ってから無効化します。
- 停止後に自動復帰はしません。
- ファームが停止した場合は100msのウォッチドッグでリセットします。
- 速度上限はモーターの指令範囲いっぱいの210rpmです（ファームと`host/ddt_bridge.py`の両方で定義）。定格は115rpmです。
- 上限速度などの調整値は `firmware/src/bridge_controller.hpp` にあります。
