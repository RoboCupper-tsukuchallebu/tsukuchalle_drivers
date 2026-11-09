# ddt-can-bridge

PCのUSBからWaveshare RP2350-CAN経由で、DDT M1502Aモーター2台（CAN ID 1、2）へ速度指令を中継します。
設計は [docs/can_bridge_design.md](docs/can_bridge_design.md) を参照してください。

M1502A 2台を無負荷、10rpm、2秒で駆動して動作を確認しました（2026-11-01）。異常時の停止動作と高速域、負荷時は未検証です。プロトコルはM15シリーズ（M1502D）の資料に基づきます。

## 必要なもの

- DockerとGNU make（Pico SDK、picotool、arm-none-eabi-gcc、CMake、Ninjaはイメージに入っています）
- 「使い方」でホストから基板を操作するにはPython 3.10以上（追加ライブラリなし。macOS/Linuxのみ）
- ビルド・テストをDockerなしで行う場合は「Dockerを使わない場合」を参照

## テスト

```sh
make test    # tests/ のctest（ASan/UBSan）と host/ のunittest
```

## ビルド

```sh
make         # test と firmware
make firmware
```

初回は `make image` でイメージ（Arm GNU Toolchain 13.3.Rel1、Pico SDK 2.3.1、picotool 2.3.1）をビルドします。ネットワークを使うのはこのときだけで、`make test` と `make firmware` は `--network none` で動きます。成果物は `build/docker/firmware/ddt_can_bridge.uf2` です。

BOOTボタンを押しながらUSB接続し、出てきたドライブに `.uf2` をコピーします。

### Dockerを使わない場合

Pico SDK 2.3.1（`PICO_SDK_PATH`）、CMake、Ninja、arm-none-eabi-gcc（13.3.rel1）、Python 3.10以上が必要です。

```sh
cmake -S tests -B build/tests -G Ninja
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
(cd host && python3 -m unittest)

cmake -S firmware -B build/firmware -G Ninja
cmake --build build/firmware    # build/firmware/ddt_can_bridge.uf2
```

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
| `joy_enable_button` | 7 | `/joy` でこのボタンを押すと有効化（Start を仮定）。押した瞬間だけ反応し、デッドマンかスティックが中立でないと拒否。-1 でパッドからの有効化だけ無効（無効化は生きる） |
| `joy_disable_button` | 6 | このボタンを押すと無効化（Back を仮定）。常に効く。`joy_enable_button` が 0 以上なら -1 不可 |
| `joy_deadman_button` | 0 | `teleop_twist_joy.yaml` の `enable_button` と同じ番号にする。押している間は有効化しない |
| `joy_neutral_axes` | `[0, 1]` | 有効化時に中立（絶対値 0.1 以下）を要求する軸。`teleop_twist_joy.yaml` の `axis_linear.x`・`axis_angular.yaw` と同じ番号にする |
| `auto_reenable_wait_s` | 10.0 | USB 切断から復帰した後、デッドマンが離されスティックが中立になるのを待つ最大秒数。超えたら無効のまま。`joy_node` の `autorepeat_rate` が 0 より大きいことが前提（0 だと操作していない間 `/joy` が来ず、必ず諦める） |
| `publish_tf` | true | TF `odom` → `base_footprint` を出す。実行中に変更可（bool 以外は拒否）。`/wheel/odom` は常に出す |

3つのボタン番号は互いに異なる必要があります。最初に届いた `/joy`、および 0.5 秒以上途絶えた後の最初の `/joy` は、有効化の「押した瞬間」とみなしません（Start を押したまま `joy` を再起動したり、パッドを挿し直したりしても有効化されません。無効化は途絶後でも効きます）。この判定は `joy_node` の `autorepeat_rate` が 0 より大きいことが前提です（初期値は 20 Hz）。`joy_deadman_button: -1` や `joy_neutral_axes: []` にすると、その確認は黙って外れます。無効化の後は、基板が `DISABLED` になるまで `/cmd_vel` を捨て、`DISABLE` を送り直し、1.5 秒たっても `DISABLED` にならなければ `disable not confirmed` を ERROR で出します。

ノードは実行中の変更を受け付けますが、ファイルには保存しません。確定した値は `ros2/ddt_diff_drive/config/diff_drive.yaml` に書いてください。`pixi run drive` はこのファイルを直接読みます。

## 車輪オドメトリ

モーターのフィードバック rpm（指令値ではない）から `/wheel/odom`（`nav_msgs/Odometry`）と TF `odom` → `base_footprint` を 50 Hz で出します。IMU は無いので yaw はドリフトします。LiDAR・GPS との融合は lightweight-nav-stack 側の担当です。

- フレーム: `header.frame_id = odom`、`child_frame_id = base_footprint`。`base_footprint` → `base_link` は `tsukuchalle_description` の `robot_state_publisher` が出します。twist は `base_footprint` 上の `linear.x`（前進が正）と `angular.z`（左回りが正）です。
- 積分はフィードバックを受け取るたびに行い（`poll` の時刻。通常は制御タイマの 50 Hz でまとめて、`~/enable` のブロック中は `poll(0.02)` でほぼフレームごと）、左右それぞれの最新 rpm を中点法で積分します。出版は制御タイマ（50 Hz）で、前回の出版以降に積分が 1 回以上あれば出します。無効化（DISABLED）中でも 24V が入っていれば出ます（手で押した移動も積分します）。静止中も出ます。
- 原点はノード起動時の位置です。USB 再接続や再有効化ではリセットしません。
- 途絶: 左右どちらかのフィードバックが 0.1 秒来ないと積分と出版を止め（`wheel odometry paused`）、再開すると `wheel odometry resumed` を出します。止まっていた間の移動は失われます。nav-stack は 0.2 秒で `INPUT_STALE` にします。
- 姿勢は実際の車体より 20〜30 ms 程度古い（USB 1 段 ＋ poll 間隔 20 ms ＋ フレーム間隔 10 ms）。
- 共分散: twist は 1 rpm の量子化（片輪 q = 2π·r/60）から `σ_v = q/√2 + 0.02|v|`、`σ_w = √2·q/W + 0.10|w|`。pose は積分で増える名目値です。**nav-stack の車輪因子はこの共分散を読みません**（因子のノイズは nav-stack 側のパラメータ）。

既知の制限:

- `~/enable` は最大 1.5 秒ブロックします（走行中の `false` はファームの STOPPING 0.5 秒を待つので約 0.6 秒）。その間は姿勢の積分は続きますが出版が止まります。パッド（Start / Back）はブロックしません。
- 制御タイマが 0.1 秒を超えて遅れると、その間の移動は失われます（`wheel odometry dropped <秒> s of motion` を WARN で出します）。
- システム時刻が逆行すると、stamp が追いつくまで出版が止まります（`stamp did not advance` の WARN は起動後 1 回だけ）。
- 1 rpm 刻みなので、0.1 m/s 未満では速度の相対誤差が 10 % 級です。

`track_width_m` の校正: その場で車体を実角度 θ_true（例: 床のマークで 1 回転 = 2π）だけ回し、`/wheel/odom` の yaw を積算した値 θ_odom を読みます（回転の途中で ±π を跨ぐので、差分を足し合わせる）。補正後のトレッドは `track_width_m × θ_odom / θ_true` です。値は `tsukuchalle_description` の `config/dimensions.yaml` を先に直し、本リポの `diff_drive.yaml` を合わせます。

## ロボット PC で動かす（Docker）

ロボット PC には pixi を入れず、Docker イメージで `drive`（ノード）、`joy`（ジョイスティック）、`teleop`（`/joy` から `/cmd_vel_teleop`）と `mux` を動かします。Mac では実機を運転できません（Mac の Docker は arm64 で、ビルドと試験のみ）。

### 配布（ビルド機で実行）

Mac からはビルド機へ `ssh <ビルド機>` で入って実行します。送りたいコミットをビルド機の clone で checkout し、作業ツリーを clean にしてから実行します（未 commit の変更があると中止します）。ビルド機からロボット PC へ鍵認証の ssh が通り、両方で `docker` が sudo なしで使えることが前提です。

```sh
scripts/deploy.sh <ロボットのSSH先> <ddt|velodyne|all>
```

送るイメージは必須で、`ddt`（`ddt-can-bridge-ros2`）、`velodyne`（`tsukuchalle-velodyne`）、`all`（両方）のどれかを指定します。省略や未知の値は usage を出して終了 2 です。Velodyne のイメージは別なので、`ddt_diff_drive` の修正だけなら `ddt` を使います。`compose.yaml`・`config/`・`udev/` はどれを選んでも送ります。

ビルド、`docker save | gzip | ssh ... docker load`、`latest` タグ付け、`~/ddt-can-bridge/` への `compose.yaml`・`config/`・`udev/` の送付までを行い、最後に起動コマンドを表示します。`ddt` の圧縮後のイメージはおよそ370MBで、LAN内の1回転送です（`ddt`・`velodyne` それぞれの所要時間は実測して追記してください）。ビルド機とロボット PC の `uname -m` が違うと中止します。経路だけ試すときは、ssh で届くローカルの宛先（ビルド機自身など）を送り先にします。

### 起動（ロボット PC の `~/ddt-can-bridge/`）

```sh
# 初回だけ。.env は compose が自動で読むので、別のシェルや再ログインの後でも ps・down・logs・exec が通る
# TSUKUCHALLE_UID/GID=0 は他リポが uid 1000 になるまでの暫定（省略不可。下の説明）
printf 'DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...\nTSUKUCHALLE_UID=0\nTSUKUCHALLE_GID=0\n' > ~/ddt-can-bridge/.env
docker compose up -d drive joy teleop mux                # この時点ではモーターは無効
```

**deploy（`scripts/deploy.sh` とその後の `up`）の前に `.env` を書いてください。** 既定の uid は 1000 ですが、全リポ（datagnss・description/foxglove・pointcloud・nav-stack）を uid 1000 に切り替えるまでは、X13 でも `TSUKUCHALLE_UID=0` と `TSUKUCHALLE_GID=0` を書きます（**省略不可**。省略すると本リポだけ uid 1000 になり、root のままの他リポのコンテナとの間で点群などが黙って 0 受信になります）。切り替えの手順は下の「ROS 2 の通信範囲と実行 uid」です。

```sh
# 切り替え前の X13（他リポがまだ root）
printf 'DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...\nTSUKUCHALLE_UID=0\nTSUKUCHALLE_GID=0\n' > ~/ddt-can-bridge/.env
```

`.env` の変数（`DDT_SERIAL_DEVICE` と、切り替えまでの `TSUKUCHALLE_UID`/`TSUKUCHALLE_GID` 以外は省略可。既定は X13 の値）:

| 変数 | 既定 | 内容 |
|---|---|---|
| `TSUKUCHALLE_UID` / `TSUKUCHALLE_GID` | 1000 / 1000 | 全サービスの実行 uid / gid。切り替え後はロボット PC のログインユーザー（`id -u`、`id -g`）に合わせる。**切り替えまでは 0 / 0 を書く（省略不可）** |
| `TSUKUCHALLE_DIALOUT_GID` | 20 | `drive` が `/dev/ttyACM*`（`root:dialout 0660`）を開くための数値 GID（`getent group dialout`） |
| `TSUKUCHALLE_INPUT_GID` | 995 | `joy` が `/dev/input/event*`（`root:input 0660`）を開くための数値 GID（`getent group input`。X13 は 995 で、PC ごとに違う） |
| `TSUKUCHALLE_DDS_MODE` | 空 | `lan` にすると Mac などから見られる（下の「ROS 2 の通信範囲」） |

`DDT_SERIAL_DEVICE` は必須で、未設定だと **`down`・`ps`・`logs` を含む compose のすべてのコマンド**が `required variable DDT_SERIAL_DEVICE is missing` で止まります（`.env` か `export` のどちらかで設定してください）。`scripts/deploy.sh` が送るのは `compose.yaml`・`config/`・`udev/` だけで、`.env` は上書きも削除もしません。`.env` はリポジトリには置きません（`.gitignore` 済み）。

動かすには車輪を浮かせて、パッドで有効化します（コンテナの起動では有効化しません）。**Start で有効化、Back で無効化**します（ボタン番号は下の `/joy` で確認）。デッドマン（A）を離し、左スティックを中立に戻してから Start を押してください。そうでないと `enable ignored: ...` の警告が出て有効化しません。

パッドが使えないときの代替として、サービスでも有効化できます（中立条件は見ません）。`exec` は entrypoint を通らないため、`docker-entrypoint.sh` 経由で呼びます。

```sh
docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
```

`enable_button`（初期値は0番、Xbox系のA）を**押している間だけ**、左スティックの上下で前後、左右で旋回します。離すと teleop はゼロを一度送り、約 0.5 秒後には nav の指令があれば走行を再開します。確実に停止するには Back で無効化してください。無効化は Back、`data: false`、またはコンテナの停止です。

```sh
docker compose logs -f drive teleop     # 状態の確認
docker compose stop                     # SIGTERMで停止処理（DISABLE）を実行（Ctrl-Cや down でも同じ）
```

ナビゲーションと同時に手動走行できます。nav-stack は変更せず /cmd_vel に発行し、ジョイスティックとキーボードは /cmd_vel_teleop に発行します。mux は teleop（優先度 100）を nav（優先度 10）より優先し、各入力のタイムアウトは 0.5 秒です。出力は /cmd_vel_mux で、compose の drive はこれを購読します。ほかの /cmd_vel 発行元も nav 入力として mux を経由しなければ drive に届きません。デッドマンを押している間は nav のゼロ指令も手動操縦を妨げません。解除後に nav が動作中なら、タイムアウト後に自律走行が再開します。両入力が途絶えると mux の発行が止まり、drive の指令タイムアウトで停止します。

mux の構成と優先動作の試験は、イメージをビルドした Linux マシンのリポジトリ root で実行します。構成試験には Python の PyYAML と Docker Compose が必要です。

```sh
python3 ros2/tests/test_mux_config.py
docker run --rm --network none --ipc private --shm-size 256m -e ROS_DOMAIN_ID=83 \
  -v "$PWD:/test:ro" ddt-can-bridge-ros2:mux python3 /test/ros2/tests/test_mux_behavior.py
```

キーボードで動かすときは、ジョイスティックの代わりに次を使います。

```sh
docker compose run --rm teleop-keyboard
```

- ボタン・軸の番号が違うパッドは、`docker compose exec joy /ws/ros2/docker-entrypoint.sh ros2 topic echo /joy` で確認して `config/teleop_twist_joy.yaml` を直し、`docker compose up -d --force-recreate teleop` で反映します。Start・Back・デッドマンの番号と中立を見る軸は `ros2/ddt_diff_drive/config/diff_drive.yaml`（`joy_*`）にもあるので、**両方を同じ番号に直してください**（片方だけ直すと、有効化の誤操作防止が黙って効かなくなります）。`diff_drive.yaml` は `drive` のイメージに焼き込まれているので、`teleop` の `--force-recreate` だけでは反映されません。ビルド機で `scripts/deploy.sh <ロボットのSSH先> ddt` を実行してイメージを作り直して送り、ロボット PC で `docker compose up -d drive` します。反映後は `docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 param get /diff_drive_node joy_deadman_button` で値を確かめてください。初期値は 0.6 m/s、1.5 rad/s です（2026-11-06 に車輪を浮かせた試験と地面での走行で確認し、0.3 m/s、1.0 rad/s から引き上げ）。**初めて走らせるロボットや床では、スティックを浅く倒して低速で確かめてください。** A を離してから止まるまで、最高速から約 0.8 秒（約 0.3 m）かかります。
- `joy` は `/dev/input` を丸ごと読めるため、コンテナ内からホストのキーボード入力も読めます。専用のロボット PC を前提にしています。
- USB を抜き差ししたり、基板がリセットしたりしても、`drive` は終了せず、ノードが基板を開き直します（`lost the board` は ERROR、`board reconnected` は WARN で、累計回数つきで毎回ログに出ます。回数の上限はありません）。有効だったときは、デッドマン（A）が離されスティックが中立になってから自動で再有効化します（`auto re-enabled after board reconnect`）。`auto_reenable_wait_s`（10 秒）以内に中立にならないと `auto re-enable abandoned` を出して無効のままになるので、Start で有効化してください。切断中に Back（または `data: false`）を押すと、再有効化は取り消されます。`joy_node` の `autorepeat_rate` を 0 にすると中立を判定できず、必ず諦めます。ファーム側の停止（`feedback lost` など）からは自動復帰しません。なお起動時に基板が無いと、`drive` は従来どおり終了します。
- `DDT_SERIAL_DEVICE` は `/dev/serial/by-id/usb-...` か `/dev/ddt_board`（下の udev ルールの symlink）を指定してください。`/dev/ttyACM0` を固定で指定すると、再列挙で `ttyACM1` になったときに永久に再接続できません。`drive` は `/dev` を丸ごと見る（`devices:` は作成時に固定され、再列挙に追従できないため）ので、`joy` と同様に専用のロボット PC を前提にしています。`/dev` は再帰的に見えるため、ホストの `/dev/pts`・`/dev/ptmx` にもコンテナの root から届きます。
- ModemManager が動いていると、挿した直後の `ttyACM` に AT コマンドを送ることがあります。ノードの排他ロック（TIOCEXCL）は open 後にしか効きません。`mmcli -L` と `journalctl -u ModemManager` で確認し、症状が出たら `sudo systemctl mask --now ModemManager` してください。再列挙のたびにプローブされると、再接続が `EBUSY` になったり、AT コマンドが基板に流れたりします。`reconnect failed` が続くときも mask してください。

### udev ルール（シリアル機器の symlink）

`scripts/udev/70-tsukuchalle.rules` が、RP2350-CAN を `/dev/ddt_board`、DATAGNSS MA-10P を `/dev/ma10p_nmea`（if00）・`/dev/ma10p_info`（if02）にし、`dialout` 0660 と ModemManager 除外（`ID_MM_DEVICE_IGNORE=1`）を付けます。シリアル番号で 1 台ずつ指定しています。

- 設置: deploy の後、ロボット PC で `sudo bash ~/ddt-can-bridge/udev/install.sh`。`udevadm verify` の後に設置済みとの `diff -u` を表示します。同一なら `already installed` で何もせず終了します。差分があれば設置して reload し、tty だけに change を飛ばします（`joy` の input や USB LAN の net には影響しません）。deploy は sudo をしません。
- 基板・受信機を替えたとき: `udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_USB_INTERFACE_NUM'` で値を取り、`70-tsukuchalle.rules` を直して PR（CI の `udevadm verify` が通ること）、deploy、`install.sh` の順です。
- ルール番号 70 は、`60-serial.rules`（`ID_SERIAL_SHORT`・`ID_USB_INTERFACE_NUM` を付ける）より後、ModemManager の `77-mm-*.rules` より前です。リネームするときはこの範囲を守ってください。
- 2 台目のロボットは同じファイルに足さず `71-<名前>.rules` を別に作ります（シリアル番号は秘密ではないので直書きでよい）。

### Velodyne HDL-32E（LiDAR）

`velodyne-driver`（UDP パケットを受ける）と `velodyne-transform`（`/velodyne_points` にする。本リポの `ros2/velodyne/tsukuchalle_velodyne` の `velodyne_pointcloud_node`。PCL を使わない自作で、式は ros-drivers の `velodyne_transform_node` と同じ）を、別イメージ `tsukuchalle-velodyne` で動かします。このイメージは ubuntu:noble に 2 つの実行ファイルと必要な .so だけを置いた最小構成で、`ros2` CLI は入っていません。ENTRYPOINT は uid と DDS の設定を確かめるだけの `/dds_check.sh`（POSIX `sh` 15 行ほど。確かめたら `exec` で消えます）だけです（`ros2 topic hz`・bag は `ros2/tools/Dockerfile` の `tsukuchalle-ros-tools` を `docker run` で使います）。送るには `scripts/deploy.sh <ロボットのSSH先> velodyne`（または `all`）を実行します。`all` はイメージを 1 つずつ送るので、2 つ目のビルドが失敗すると、1 つ目は送られて `latest` になっていますが `compose.yaml`・`udev/` は送られません。

```sh
# 仮設に手で起動した 3 つ（コンテナとイメージ）を先に消す。残っていても bind は失敗せず（SO_REUSEADDR）、点群が片方にしか届かないだけでログにも出ない。
# tf-velodyne は description の静的 TF と二重に出て点群の向きが揺れ、foxglove-bridge は description 側の bridge が 8765 で落ちる
docker ps -a --format '{{.Names}}\t{{.Image}}' | grep -E 'velodyne-test|tf-velodyne|foxglove-bridge'    # 消す前にイメージ名を控える
docker rm -f velodyne-test tf-velodyne foxglove-bridge 2>/dev/null || true
docker rmi velodyne-test:jazzy                                                                           # 控えた他のイメージも同様に消す
cd ~/ddt-can-bridge                       # .env の DDT_SERIAL_DEVICE は Velodyne だけの up でも必要（compose が全体を評価する）
docker compose --profile velodyne up -d velodyne-driver velodyne-transform
```

Velodyne の 2 サービスは `profiles: [velodyne]` なので、`--profile velodyne` を付けない `docker compose up -d` では起動しません（`ddt` だけ送ったロボットで velodyne のイメージを pull しに行って止まるのを避けるため）。起動済みのものへの `exec`・`logs`・`ps`・`stop` は profile 無しで通りますが、**`down` は `--profile velodyne` を付けないと Velodyne のコンテナが残ります**。

確認は `ros2 topic hz`（jazzy では best-effort 固定）で足ります。tools イメージはロボット PC でビルドし、**ホストのログインユーザーの uid で**起動します（`--user` を忘れると root になり、`/dds_check.sh` が止めます。`-e TSUKUCHALLE_UID` を付けないと uid の検査は飛ぶので、例には必ず付けます）。

```sh
# deploy.sh の `tools` 選択肢は別 PR。リポ root で:
docker build -f ros2/tools/Dockerfile -t tsukuchalle-ros-tools .
docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" -e TSUKUCHALLE_UID="$(id -u)" \
  tsukuchalle-ros-tools ros2 topic hz /velodyne_points                 # 10 Hz 前後（欠けると 10 を大きく下回る）
docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" -e TSUKUCHALLE_UID="$(id -u)" \
  tsukuchalle-ros-tools bash -c \
  "timeout 10 ros2 topic echo --qos-reliability reliable --qos-durability volatile --field header.frame_id /velodyne_points | grep -c velodyne"   # reliable は 10 秒で 100 前後
```

- ネットワーク前提（deploy はしない。`sudo` が要る一回きりの作業）: USB 有線 LAN `enx6c1ff772d439` に NetworkManager 接続 `velodyne`（固定 192.168.100.100/24）が済んでいること。`nmcli con add type ethernet ifname enx6c1ff772d439 con-name velodyne ipv4.method manual ipv4.addresses 192.168.100.100/24` 相当です。確認は `ip addr show enx6c1ff772d439` と `ping 192.168.100.200`。センサー（192.168.100.200、UDP 2368）の送信先が 192.168.100.100 かブロードキャストであることを HDL-32E の Web UI で確かめます。
- センサーの IP を変えたら、`compose.yaml` の `velodyne-driver` の `device_ip` を直します（送信元が違うパケットは黙って捨てられます。`.env` の変数にはしていません）。
- センサーが無い・LAN が抜けているときは、`velodyne_driver_node` が `Velodyne poll() timeout` を WARN で出し続けますが終了せず、復旧は自動です。`velodyne_pointcloud_node` は入力が無いだけです。どちらも `restart: on-failure` です。
- 出力は `/velodyne_points`（`sensor_msgs/PointCloud2`、10 Hz、organized で width 32（列 = ring）× height 2172、`is_dense: false`、1 フレーム約 1.5 MB）。範囲外・返りなしの点は x,y,z,intensity が NaN なので、使う側は非有限点を捨てます。副産物として `/velodyne_packets`（約 2.2 MB/s）と `/diagnostics` も出ます。
- best-effort（SensorDataQoS）でも欠けません（[#19](https://github.com/RoboCupper-tsukuchallebu/tsukuchalle_drivers/issues/19)。原因は下の「ROS 2 の通信範囲」）。CI は best-effort 20/20 を確かめています。
- Foxglove の注意: 生の `/velodyne_points`（約 15 MB/s）を Wi-Fi 越しの Foxglove で購読すると foxglove_bridge の送信バッファが埋まり、他のトピックも落ちます。Foxglove では `tsukuchalle_pointcloud` の間引きトピック（別 Issue）を見てください。それまでは 3D パネルで `/velodyne_points` を購読せず、一覧に出る `/velodyne_packets` も購読しません。

### ロボット PC の電池（`host_monitor`）

ロボット PC の電池の状態を `/diagnostics` に出します。`ddt` のイメージ（`ddt-can-bridge-ros2`）に入っているパッケージ `tsukuchalle_host_monitor`（`ros2/tsukuchalle_host_monitor`）で、`drive` と同じイメージを使うので追加の deploy は要りません（`scripts/deploy.sh <ロボットのSSH先> ddt`）。`/sys/class/power_supply` を読むだけで、モーター・駆動系の topic は購読も発行もしません。

```sh
docker compose up -d host_monitor
docker compose exec host_monitor /ws/ros2/docker-entrypoint.sh ros2 topic echo --once --qos-reliability reliable /diagnostics
```

| 項目 | 内容 |
|---|---|
| サービス | `host_monitor`（`/sys/class/power_supply` を read-only でマウント。`user`・`ipc: host`・`network_mode: host` は他のサービスと同じ） |
| 出すもの | `/diagnostics`（`diagnostic_msgs/DiagnosticArray`、1 Hz）。DiagnosticStatus は 1 つで `name` は `host: battery`、`hardware_id` は電池名（`BAT0` など） |
| 購読するもの | なし |

`values` のキー（読めない値は `unknown`）:

| キー | 内容 |
|---|---|
| `capacity_percent` | 残量 %（`capacity`） |
| `status` | `Charging` / `Discharging` / `Full` / `Not charging`（`status` のまま） |
| `ac` | `online` / `offline`（`type` が `Mains` か名前が `AC` で始まる電源の `online`。どれかが 1 なら `online`） |
| `energy_wh` / `energy_full_wh` / `energy_full_design_wh` | 今・満充電・設計のエネルギー Wh（`energy_now` / `energy_full` / `energy_full_design`）。無ければ `charge_*`（µAh）に `voltage_now` を掛けた概算 |
| `power_w` | 消費電力 W（`power_now`）。無ければ `current_now` × `voltage_now`。符号は捨てる |
| `time_remaining_min` | 残り時間の見込み（分）。放電中だけ、`energy_wh` / `power_w`。今の消費電力が続く前提で、負荷が変わると大きく動く |
| `health_percent` | 劣化の度合い。`energy_full_wh` / `energy_full_design_wh` の % |
| `cycle_count` | 充放電回数（`cycle_count`） |
| `voltage_v` | 電圧 V（`voltage_now`） |

level: 放電中（`status` が `Discharging`）で残量が `battery_error_percent` 以下なら ERROR、`battery_warn_percent` 以下なら WARN、それ以外は OK（充電中・満充電では残量が低くても OK）。電池が見つからなければ STALE（メッセージ `no battery found`）。周辺機器の電池（`scope` が `Device`）と `present` が 0 の電池は無視し、名前順で最初の電池を使います。

| パラメータ | 初期値 | 内容 |
|---|---|---|
| `power_supply_root` | `/sys/class/power_supply` | 読む場所。試験で偽装した sysfs に差し替える |
| `battery_warn_percent` | 20.0 | 放電中、この残量以下で WARN |
| `battery_error_percent` | 10.0 | 放電中、この残量以下で ERROR（`0 <= error <= warn <= 100`。外れると起動しない） |

値を変えるときは `ros2/tsukuchalle_host_monitor/config/host_monitor.yaml` を直し、`scripts/deploy.sh <ロボットのSSH先> ddt` で配ります（イメージに焼き込まれます）。試験は `make` では走りません。`pixi run test`、またはイメージの中で `docker run --rm ddt-can-bridge-ros2:<タグ> python3 -m unittest discover -s /ws/ros2/tsukuchalle_host_monitor/test` を実行します（CI の `ddt-image` も同じ）。

### ROS 2 の通信範囲と実行 uid

#### 設定

- 全イメージ（`ros2/Dockerfile`、`ros2/velodyne/Dockerfile` の `runtime`、`ros2/tools/Dockerfile`）に `ros2/dds/fastdds_profiles.xml` を `/fastdds_profiles.xml` として焼き込み、ENV は `ROS_DOMAIN_ID=42`・`ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT`・`FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml`・`ROS_LOG_DIR=/tmp/ros_log`・`ROS_HOME=/tmp/ros_home` です。`FASTDDS_BUILTIN_TRANSPORTS` と `LOCALHOST` は使いません。
- XML の中身: 共有メモリ（SHM）のセグメントを **16 MiB / プロセス**、UDPv4 は `127.0.0.1` だけ（`maxInitialPeersRange 32`）、多播なし、`initialPeersList` は `127.0.0.1`。同じホストの別コンテナと SHM でつながり、**同じ LAN の別 PC には見えません**。`LOCALHOST` のままのコンテナ（他リポで未更新のもの）とも混在できます。
- 以前の `LOCALHOST` は、rmw_fastrtps が XML を読んだ**後に** 512 KiB の既定 SHM を足すため、1.5 MB の `/velodyne_points`（約 24 断片）が best-effort で欠けました（断片 1 つで 1 フレームを失う）。reliable は再送されるので欠けません。LOCALHOST の UDP は全インターフェースで開きますが、LAN に出ないのは多播を作らず peer が `127.0.0.1` だけのためです。
- `ipc: host` と `network_mode: host` は全コンテナの前提です（compose が付けます。`docker run` は `--ipc host --network host`）。`ipc: host` が無いと SHM が使えません。
- XML と `ros2/dds/dds_check.sh` は他リポ（datagnss・description・pointcloud）と nav-stack に複製します。正は本リポで、複製は sha256 で照合します（`fastdds_profiles.xml` の先頭コメントに「変えるときは tsukuchalle_drivers を先に」）。

#### 起動時の検査（`/dds_check.sh`）

全イメージの ENTRYPOINT の先頭で走り、次のどれかなら理由を出して終了 1 にします。`compose exec` や `--entrypoint` の上書きは通らないので、`exec` で `ros2` を使うときは ENV の設定だけが効きます。

- `TSUKUCHALLE_UID` が設定されていて `id -u` と違う（`0` なら root で通る。未設定なら uid は見ない）。
- `FASTRTPS_DEFAULT_PROFILES_FILE` が読めない、または `is_default_profile="true"` の participant profile が無い。
- `ROS_AUTOMATIC_DISCOVERY_RANGE` が `SYSTEM_DEFAULT` でない。XML が読まれないと SUBNET の既定に落ち、LAN に多播で出て 512 KiB の SHM に戻るのを防ぎます。

試験: `ros2/dds/test_dds_check.sh`（この PC で）、`ros2/dds/test_dds_check.sh <イメージ>`（イメージの中のスクリプトを uid 1000・0 で）。

#### 実行 uid は全 ROS プロセスで 1 つ（X13 は 1000）

- SHM のファイル（`/dev/shm/fastrtps_*`）は**作った側の uid で `0644`** です。`umask` を変えても直りません。別 uid のプロセスは相手を発見（match）できても書き込めず、**エラーなしで 0 受信**になります（`ros2 topic list` や購読者数では見つからず、実際に受信を数えるしかありません）。
- そのため、compose は全サービスに `user: "${TSUKUCHALLE_UID:-1000}:${TSUKUCHALLE_GID:-1000}"` を付け、nav-stack（pixi）と `ros2` CLI は同じログインユーザーで動かします。**`sudo ros2 ...` は root になり 0 受信になるので使わないでください。**
- 書き込み先は `/tmp` の下（`ROS_LOG_DIR`・`ROS_HOME`）です。イメージに `useradd` はしません（root でも 1000 でも動く）。`drive`・`joy` は数値の GID を `group_add` で足します（上の `.env` の表）。
- **暫定（他リポのコンテナがまだ root のとき）**: `.env` に `TSUKUCHALLE_UID=0` と `TSUKUCHALLE_GID=0` を書いて本リポも root で動かします（`/dds_check.sh` は 0 でも通る）。DDS の XML は混在できるので配る順は任意ですが、**uid の切り替えは X13 の全 ROS コンテナ（本リポ・datagnss・description/foxglove・pointcloud）が新イメージになってから同時に**行います。

#### 切り替え手順（root から uid 1000 へ。全コンテナ同時）

root のコンテナが作った `fastrtps_*` は `/dev/shm`（1777、sticky）では uid 1000 から消せず、同じ名前のポートファイルを開けないため、先に掃除します。

```sh
# 1. 全 compose プロジェクト（本リポ・datagnss・description・pointcloud）を止める。velodyne は --profile velodyne も付ける
docker compose --profile velodyne --profile keyboard down
# 2. 残骸を消し、root 所有が 0 であることを確かめる（ホストに fastdds CLI は無い。tools イメージの `fastdds shm clean` でも可）
ls -la /dev/shm | grep fastrtps
sudo rm -f /dev/shm/fastrtps_*
ls -la /dev/shm | grep -c fastrtps            # 0
# 3. 各 .env の TSUKUCHALLE_UID=0 / TSUKUCHALLE_GID=0 を消す（または 1000 にする）
# 4. 全部起動し、所有者が全部ログインユーザーであることを確かめる
docker compose up -d drive joy teleop mux && docker compose --profile velodyne up -d velodyne-driver velodyne-transform
ls -la /dev/shm | grep fastrtps               # 所有者は全部ログインユーザー（1000）
docker compose exec drive id -u               # 1000
```

#### RAM と `/dev/shm`

- SHM のセグメントは**プロセス（Context）ごと**に 1 つで、確保時に `memset` するので RAM を実際に使います。1 プロセスあたり約 17,564,192 B（16 MiB ＋ 約 0.75 MiB）です。`ros2` の daemon や CLI を 1 回起動するたびにも 1 つ増えます。
- X13 の常時プロセス ≈ 15 で約 250 MiB（`/dev/shm` は 3.5 GiB の tmpfs）。`df /dev/shm` と `ls -la /dev/shm` で合計を照合します。コンテナを kill すると `fastrtps_*` が残ることがあるので、増え続けるときは全部止めて `sudo rm -f /dev/shm/fastrtps_*` です。
- `--ipc host` の無いコンテナ（1 コンテナに participant が 4 つ以上）は `--shm-size=256m` 以上を必須にします。`/dev/shm` が小さい（Docker の既定 64 MiB や 32m）と SHM の作成に失敗して ERROR（`Failed to create segment` / `Unable to Register SHM Transport`）を出し、**SHM なしで続行します**。プロセスは落ちず、点群は UDP の `lo` に回って黙って劣化します（欠けが戻る）。ログに上の ERROR が出たら `--shm-size` を疑ってください。

#### Mac から見る（`TSUKUCHALLE_DDS_MODE=lan`）

普段は Foxglove で見ます。Mac の ROS 2 から直接見るときだけ、`.env` に `TSUKUCHALLE_DDS_MODE=lan` を書いて `docker compose up -d` し直します。`/dds_check.sh` が XML と `SYSTEM_DEFAULT` を外して SUBNET に戻します。**この間は 512 KiB の SHM と多播に戻るので、best-effort の欠けが再発し、LAN から見えます。** 終わったら `.env` から消して `up -d` し直します。Mac 側は XML なしの既定のままで、`export ROS_DOMAIN_ID=42` だけで見えます。

## 安全動作

- **基板はUSB給電のため、USBが抜けると停止処理を実行できません。24Vをすぐ遮断できる非常停止を用意してください。**
- 基板はDISABLED中、0指令と無効化を100ms周期で送り続け、明示的なENABLEまで動かしません。
- 指令200ms途絶、フィードバック100ms途絶、モーター故障、速度モード離脱、CAN送信失敗で、速度0を500ms送ってから無効化します。
- ファームが止めた後に自動復帰はしません。USB 切断からの復帰はノードが行い、スティックとデッドマンが離されてから再有効化します。回数上限は無く、ログで確認してください。
- ファームが停止した場合は100msのウォッチドッグでリセットします。
- 速度上限はモーターの指令範囲いっぱいの210rpmです（ファームと`host/ddt_bridge.py`の両方で定義）。定格は115rpmです。
- 上限速度などの調整値は `firmware/src/bridge_controller.hpp` にあります。
