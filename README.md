# ROX ロボット制御ライブラリ

メカナム、DualSense、ATシリアル通信、エンコーダー付き位置サーボをPythonから扱うライブラリです。プログラミング時はこのREADMEを関数・ボタン名の早見表として使ってください。

## 本番の起動

メカナム、catch/lift、ソレノイド、状態表示サイトをまとめて起動します。

```bash
cd ~/Desktop/honban/ROX_matsue2026_honban
python3 run_all.py
```

起動時にcatch/liftを機械的な原点へ合わせ、Enterを押します。状態サイトは次で開けます。

```text
http://ロボットのIPアドレス:8000
```

個別の `mecanum.py`、`servos.py`、`sorenoido.py` は動作確認用です。同じ `/dev/ttyUSB0` を使うため、`run_all.py` と同時に起動しません。

## よく使う import

```python
from rox_mecanum import (
    Button, PygameDualSense,                 # コントローラー
    MotionCommand, MecanumMixer, MecanumRobot, # メカナム
    ATMotor, PySerialTransport,              # モーター通信
    ATEncoderReader, EncoderPositionServo, PositionServoConfig, # PIDサーボ
    at_address_from_can_id,
)
```

## DualSense のボタン名

`Button.名前` を使います。`state.button()` は押している間ずっと `True`、`state.was_pressed()` は押した瞬間だけ `True` です。

| 実際のボタン | コード |
|---|---|
| × | `Button.CROSS` |
| ○ | `Button.CIRCLE` |
| □ | `Button.SQUARE` |
| △ | `Button.TRIANGLE` |
| L1 / R1 | `Button.L1` / `Button.R1` |
| L2 / R2 | `Button.L2` / `Button.R2` |
| CREATE / OPTIONS | `Button.CREATE` / `Button.OPTIONS` |
| L3 / R3 | `Button.L3` / `Button.R3` |
| PS / タッチパッド / MUTE | `Button.PS` / `Button.TOUCHPAD` / `Button.MUTE` |
| 十字キー | `Button.DPAD_UP`, `DPAD_DOWN`, `DPAD_LEFT`, `DPAD_RIGHT` |

```python
controller = PygameDualSense.open()
state = controller.read()

if state.was_pressed(Button.CIRCLE):  # ○を押した瞬間に1回だけ実行
    print("catchを閉じる")

if state.button(Button.L2):           # L2を押している間ずっと実行
    print("ソレノイドON")

if state.button(Button.OPTIONS):
    print("停止")
```

スティック値は `-1.0〜1.0` です。

```python
state.left_stick.x       # 左右。右がプラス
state.left_stick.y       # 前後。前がプラス
state.right_stick.x      # 右スティック左右
state.left_stick.magnitude      # 倒し量 0.0〜1.0
state.left_stick.angle_degrees  # 右=0°、前=90°
state.l2, state.r2       # トリガーの倒し量 0.0〜1.0
state.active_buttons     # 押されている全ボタン
```

## run_all.py の操作割当て

| 操作 | 機能 |
|---|---|
| 左スティック | 前後・左右移動 |
| R2 + 右スティック左右 | 旋回 |
| L2 | ソレノイドを指定時間だけON |
| ○ / × | catchを最大角度 / 原点へ |
| △ / □ | liftを最大角度 / 原点へ |
| OPTIONS | 全モーターとソレノイドを停止して終了 |

## メカナムをライブラリから使う

速度は `-1.0〜1.0` です。

```python
from rox_mecanum import MotionCommand, MecanumMixer

mixer = MecanumMixer(rotation_gain=0.22)
speeds = mixer.mix(MotionCommand(forward=0.5, strafe=0.2, rotate=0.0))
print(speeds.front_left, speeds.front_right, speeds.rear_left, speeds.rear_right)
```

短縮関数もあります。

```python
from rox_mecanum import forward, backward, strafe_left, strafe_right, turn_left, turn_right, stop

forward(0.5)
strafe_right(0.3)
turn_left(0.4)
stop()
```

実機へ送る場合は `MecanumRobot.drive()` を定期的に呼びます。

```python
robot.drive(MotionCommand(forward=0.5))
robot.stop()
```

## ATモーター通信

この機体の成功した通信方式は **pyserial + `/dev/ttyUSB0` + 921600 baud** です。`python-can` や `can0` をメカナム制御には使いません。

```python
from rox_mecanum import ATMotor, PySerialTransport, at_address_from_can_id

transport = PySerialTransport.open("/dev/ttyUSB0", baudrate=921600)
motor = ATMotor(transport, at_address_from_can_id(5))  # CAN ID 5 → AT宛先 0x2C

try:
    motor.enable()
    motor.set_velocity(0.10)  # 正方向10%
finally:
    motor.stop()
    transport.close()
```

### CAN ID とAT宛先

`at_address_from_can_id(can_id)` を必ず使います。自分で `0x2C` などを計算する必要はありません。

| CAN ID | AT宛先 |
|---:|---:|
| 1 | `0x0C` |
| 2 | `0x14` |
| 3 | `0x1C` |
| 4 | `0x24` |
| 5（catch） | `0x2C` |
| 6（lift） | `0x34` |

## エンコーダー付きPID位置サーボ

`EncoderPositionServo` は、目標位置とエンコーダー値の差から速度を出します。外力で位置がずれたときも、次のフィードバックで目標位置へ戻ろうとします。

```python
from rox_mecanum import ATMotor, EncoderPositionServo, PositionServoConfig

servo = EncoderPositionServo(
    ATMotor(transport, at_address_from_can_id(5)),
    PositionServoConfig(
        min_angle=-30,
        max_angle=90,
        counts_per_degree=65536 / 360,
        kp=0.015,
        ki=0.002,
        max_speed=0.20,
        tolerance_deg=0.5,
        direction=1,
    ),
)

servo.enable(retries=3)
servo.set_home(raw_count=12345)  # 原点に合わせた時のエンコーダー値
servo.write(45)                  # 目標角度を45°にする
servo.read()                      # 現在角度を読む
servo.is_at_target()              # 目標角度へ到達したか
servo.hold_current()              # 今いる位置を新しい目標として保持
servo.release()                   # 保持解除して停止

# エンコーダーを受信するたびに必ず呼ぶ。これが位置保持を行う。
servo.update(raw_count=12500, now=time.monotonic())

servo.stop()
```

### catch・liftをもっと短く書く

この機体で使うCAN ID 5（catch）とID 6（lift）は、`hensuu.py` に設定済みです。普段は `PositionServoConfig(...)` を毎回書かず、`open_servos()` を使います。

```python
import time
from servos import open_servos

servos = open_servos()
try:
    servos.attach()
    input("catch/liftを0度に合わせてEnter: ")
    servos.home_from_feedback()
    servos.start_pid()  # 以後、内部で50Hzのエンコーダー読取りとPID保持を行う

    servos.catch.write(45)
    servos.lift.write(90)
    time.sleep(5)
finally:
    servos.close()
```

`servos.catch` と `servos.lift` はどちらも同じ関数を使えます。

```python
servos.catch.write(30)
servos.lift.hold_current()
servos.catch.release()
```

PIDをモーターごとに切り替えることもできます。

```python
servos.catch.pid_off()   # catchだけ重力に任せる
servos.lift.pid_on()     # liftだけPID保持をオン

# 同じことを名前指定でも書ける
servos.pid_off("lift")
servos.pid_on("catch")
```

エンコーダー読取りには `ATEncoderReader` を使います。

```python
reader = ATEncoderReader(transport, {"catch": at_address_from_can_id(5)})
reader.request_all()

for feedback in reader.poll():
    print(feedback.name, feedback.count)
    servo.update(feedback.count, feedback.received_at)
```

## hensuu.py の主な調整値

| 変数 | 意味 |
|---|---|
| `mecanum_speed_percent` | メカナム最高速度 |
| `catch_min_angle`, `catch_max_angle` | catchの可動範囲（度） |
| `lift_min_angle`, `lift_max_angle` | liftの可動範囲（度） |
| `catch_counts_per_degree`, `lift_counts_per_degree` | 1°あたりのエンコーダーカウント。ギヤ比があれば調整する |
| `catch_pid_kp`, `lift_pid_kp` | 大きいほど復帰が強い。振動したら下げる |
| `catch_pid_ki`, `lift_pid_ki` | 重力でゆっくりずれ続ける力を打ち消す。振動したら下げる |
| `servo_max_speed_percent` | PID補正の最高速度 |
| `servo_tolerance_deg` | この誤差以内なら停止する範囲 |
| `catch_direction`, `lift_direction` | 目標と逆へ動く場合は `-1` |
| `dashboard_port` | 状態サイトのポート番号 |

## RobStride 05の角度取得と安全なサーボ動作

RobStrideの正式な位置値は`0x7019`の`mechPos`です。これは負荷側の多回転機械角度で、単位は`rad`（float）です。ライブラリはこの正式応答を受信できた場合だけ、内部で度へ変換して位置制御します。

```bash
python3 angle_monitor.py
```

このプログラムは**モーターを有効化・回転させず、角度を読むだけ**です。表示が`mechPos=... rad (...°)`になれば正式な角度読取りができています。`旧AT生値=...`の場合は、USB-AT変換器が別形式を返しているため、PIDは出力せず安全に停止したままです。

`旧AT生値`になる場合は、次を実行して生フレームを確認します。これはモーターの有効化・速度指令を一切送りません。

```bash
python3 at_mechpos_probe.py
```

AT変換器が返す16bit位置値の対応を測る場合は、こちらを使います。モーターを動かす命令は送りません。

```bash
python3 at_angle_calibrate.py
```

表示を記録してから機構を正確に90°動かし、`delta`の増減を確認します。ここで得る仮角度は検証用であり、変換式が確定するまでPIDには使いません。

AT形式で正式なfloat応答を受け取れない場合の次の確認は、SLCANとしてRobStrideへ直接type 17を送る方法です。

```bash
python3 direct_mechpos_probe.py
```

成功時は`mechPos=... rad (...°)`と表示されます。このプログラムもモーターの有効化・速度指令を送りません。`SLCANを開けませんでした`または`応答なし`なら、変換器のファームウェア・CAN配線・終端抵抗・CAN 1Mbps設定を確認します。

サーボは原点を読んだだけでは動きません。`write(角度)`、`hold_current()`、または`pid_on()`を明示的に呼んだ場合だけ位置補正を開始します。

## PIDを実機で調整する

`pid_tuner.py` は、`hensuu.py`を編集・再起動せずにPID値を試すための調整専用プログラムです。
起動時の物理位置を目標位置にして保持を始めます。`OPTIONS`は非常停止して終了です。

```bash
python3 pid_tuner.py
```

ターミナルへ次の形式で入力します。変更値は**実行中だけ**有効です。安定した値を見つけたら、同じ値を`hensuu.py`へ写してください。

```text
show                    # 現在のPID設定・角度・出力状態を表示
catch kp 0.003          # catchのPを変更
lift ki 0               # liftのIを0にする
both max 5              # 両方の最大PID速度を5%にする
both deadband 3         # 両方の停止範囲を±3°にする
catch hold              # catchの今の位置を保持
lift off                # liftのPIDを解除
quit                    # 安全停止して終了
```

調整は `I=0`、`D=0`、`max=5` から開始し、まずPだけを少しずつ上げてください。

liftだけをDualSenseで調整したい場合は、こちらを実行します。

```bash
python3 lift_pid_tuner.py
```

- 十字キー上下: `P → I → D → 最大速度 → 停止範囲` を選択
- 十字キー左右: 選択値を増減
- L1を押しながら十字キー左右: 10倍ずつ増減
- ○: 現在位置でliftをPID保持
- ×: liftのPIDを解除
- OPTIONS: 非常停止して終了

PIDが期待通りに動かない場合は、先にPIDを使わない単体確認をします。

```bash
python3 lift_check.py
```

左スティック上・下でliftを正逆転し、エンコーダー生値を表示します。この確認で両方向へ回ることを確かめてからPID調整へ進みます。

catchも同時に確認する場合はこちらです。

```bash
python3 servo_check.py
```

- 左スティック上下: lift
- R1 + ○: catch正方向、R1 + ×: catch逆方向（ボタンを離すと停止）
- OPTIONS: 両方を停止して終了

## 安全上の注意

- PIDの初回確認は必ず機構を浮かせ、`servo_max_speed_percent` を低くして行います。
- 角度が逆へ動いたら、すぐ止めて `catch_direction` または `lift_direction` を `-1` にします。
- `counts_per_degree` が実機のギヤ比と合わないと、表示角度と目標角度がずれます。
- `/dev/ttyUSB0` は1つのプロセスだけが開きます。`run_all.py` を使うときは個別プログラムを終了します。
