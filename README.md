# rox-mecanum

ROX のメカナム移動と DualSense 入力を、実機接続とは独立して使える Python ライブラリです。元の `mecanum_rc.py` の運動学、モーター取付方向、AT フレーム形式をライブラリ化しています。

## できること

- 前進・後退・左右平行移動・左右旋回のコマンド作成
- 前後、ストレーフ、旋回をメカナム4輪速度へ変換
- DualSense の全ボタン、左右スティック、L2/R2、十字キーを意味のある名前で取得
- スティックの X/Y、倒し量、角度（右=0°、上=90°）を取得
- OS 側の割当てが違う場合でも、全軸・全ボタンの生値を取得
- RobStride EDULITE05 向け AT シリアルフレーム生成・実機送信

## インストール

計算・テストだけなら追加ライブラリは不要です。

```bash
cd ROX_matsue2026_honban
python -m pip install -e .
```

DualSense を実際に読む場合は pygame、モーターにシリアル送信する場合は pyserial を追加します。

```bash
python -m pip install -e '.[hardware]'
```

## 移動コマンドと4輪速度

```python
from rox_mecanum import MecanumMixer, forward, strafe_left, turn_right

mixer = MecanumMixer(rotation_gain=0.22)  # 元の機体寸法 L + W

print(mixer.mix(forward(0.6)).as_dict())
# {'FL': 0.6, 'FR': 0.6, 'RL': 0.6, 'RR': 0.6}

print(mixer.mix(strafe_left(0.6)).as_dict())
print(mixer.mix(turn_right(0.6)).as_dict())
```

`MotionCommand(forward=..., strafe=..., rotate=...)` を直接使っても構いません。正の向きは、前進・右ストレーフ・右旋回です。混合後の速度は自動で `-1.0〜+1.0` に収まるよう正規化されます。

## コントローラーの全入力を読む

```python
from rox_mecanum import Button, PygameDualSense

controller = PygameDualSense.open()
try:
    while True:
        state = controller.read()

        # 名前で全ボタンを読む
        if state.button(Button.CROSS):
            print("× を押している")
        if state.was_pressed(Button.OPTIONS):
            break

        # 左スティック: 上を正に揃えた値、倒し量、角度
        print(state.left_stick.x, state.left_stick.y)
        print(state.left_stick.magnitude, state.left_stick.angle_degrees)

        # 全アナログ入力と、対応付け前の全物理入力
        print(state.axes)        # left_x, left_y, right_x, right_y, l2, r2
        print(state.raw_axes)    # OS が返した全軸を番号順に取得
        print(state.raw_buttons) # OS が返した全ボタンを番号順に取得
finally:
    controller.close()
```

ボタンは `CROSS`, `CIRCLE`, `SQUARE`, `TRIANGLE`, `L1`, `R1`, `L2`, `R2`, `CREATE`, `OPTIONS`, `L3`, `R3`, `PS`, `TOUCHPAD`, `MUTE`, `DPAD_UP/DOWN/LEFT/RIGHT` を扱います。

接続方法や OS によって pygame の番号が異なる場合があります。その場合でも `examples/controller_monitor.py` を実行すれば `raw_axes` と `raw_buttons` を確認できます。`ControllerProfile` を作り、正しい番号を明示的に設定できます。

```python
from rox_mecanum import Axis, Button, ControllerProfile, PygameDualSense

profile = ControllerProfile(
    axes={Axis.LEFT_X: 0, Axis.LEFT_Y: 1, Axis.RIGHT_X: 2, Axis.RIGHT_Y: 3, Axis.L2: 4, Axis.R2: 5},
    buttons={Button.CROSS: 0, Button.OPTIONS: 9, Button.L2: 6, Button.R2: 7},
)
controller = PygameDualSense.open(profile=profile)
```

## DualSense で移動する

```python
from rox_mecanum import DualSenseMotionMapping, MecanumMixer, PygameDualSense

controller = PygameDualSense.open()
mapping = DualSenseMotionMapping()  # 左スティック: 移動 / 右X: 旋回
mixer = MecanumMixer()

state = controller.read()
command = mapping.command(state)
wheel_speeds = mixer.mix(command)
```

既定の `DualSenseMotionMapping` は左スティックの移動を常に有効にし、R2 押下中のみ旋回を許可します。L2 は移動許可には使わず、ボタン入力として他の操作に自由に割り当てられます。旋回も常に有効にするには `DualSenseMotionMapping(rotation_enable=None)` を指定します。

## 実機への送信

```python
from rox_mecanum import MecanumRobot, PySerialTransport, forward

transport = PySerialTransport.open("/dev/ttyUSB1", baudrate=921600)
robot = MecanumRobot(transport)
try:
    robot.enable_all()
    robot.drive(forward(0.3))
finally:
    robot.stop()
    transport.close()
```

既定のモーターIDは `FL=0x0C`, `FR=0x14`, `RL=0x1C`, `RR=0x24` で、右側（FR/RR）の取付方向反転も元プログラムと同じ設定です。

## 実機をコントローラーで動かす

リポジトリ直下の `mecanum.py` が実行用プログラムです。速度上限・スティックの操作感・シリアルポートなどは `hensuu.py` にまとめています。L2 は移動の条件にしていないため、ソレノイドなど別の操作へ使えます。

```bash
python mecanum.py
```

初回は `hensuu.py` の `mecanum_speed_percent = 30.0` のまま、タイヤが浮いた状態で回転方向を確認してください。

## エンコーダー付き新機構をサーボのように扱う

`EncoderServo` は単体モーターを角度指定で動かすPID付きAPIです。原点を設定してから、`move_to()` に目標角度を渡します。可動範囲・最高速度・PIDゲイン・原点から見た正方向は `hensuu.py` の `mechanism_*` で設定します。

```python
from rox_mecanum import ATMotor, EncoderServo, ServoConfig

motor = ATMotor(transport, motor_id=0x05, speed_span=10000)
servo = EncoderServo(
    motor,
    ServoConfig(min_position_deg=-30, max_position_deg=90, max_command=0.15),
)
servo.enable()
servo.set_home(raw_encoder_deg=42.3)  # 物理原点に合わせた時のエンコーダー値
servo.move_to(45)                     # 原点から45°へ移動

# CAN受信部で得た角度を20〜100 Hzで渡す
state = servo.update(raw_encoder_deg=87.3)
if state.at_target:
    print("到達")
```

ArduinoのServoライブラリに近い書き方もできます。`loop()` は必ず20〜100 Hz程度で繰り返し呼ぶ。

```python
servo.attach()
servo.write(45)                   # move_to(45) と同じ
state = servo.loop(raw_encoder_deg)  # update() と同じ
print(servo.read())                # 最後に指定した目標角度
print(servo.read_position())       # エンコーダーから得た実角度
servo.detach()                     # 安全停止
```

まず `hensuu.py` の `mechanism_speed_percent` を10〜15程度にし、`mechanism_pid_ki` と `mechanism_pid_kd` は0のまま調整を始める。Kpを少しずつ上げ、振動や行き過ぎがあればKpを下げる。定常的に目標からずれる場合だけKiを微量に上げ、急停止時の行き過ぎが大きい場合だけKdを微量に上げる。

現在のATシリアル用コードは速度指令だけを実装しているため、`raw_encoder_deg` のCAN受信処理は別途必要です。この部分はUSB-CANアダプターの種類が分かれば追加できます。

### MKS CANable V2.0でエンコーダーを受信する

MKS CANableをUbuntuへ接続し、CANH/CANLをモーターCANバスへ接続する。CANバスの終端抵抗は両端だけに120Ωを有効にする。CANableが `can0` として見える場合、EDULITE05の既定1 Mbpsで起動する。

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
python3 -m pip install -e '.[can]'
```

CANableが `can0` ではなく `/dev/ttyACM0` として見える場合は、先にslcanから `can0` を作る。

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

CAN ID 5のエンコーダー値を受け、サーボへ渡す例:

```python
from rox_mecanum import CanEncoderReceiver

receiver = CanEncoderReceiver.open_socketcan("can0")
feedback = receiver.read(0x05)
if feedback is not None:
    state = servo.update(feedback.position_deg)
```

モーターは通常、指令への応答としてフィードバックを送信する。位置を保持する制御ループ中は継続して受信すること。

## 動作確認

```bash
python -m pytest
```

実機なしで、入力値・スティック角度・運動学・AT フレームをテストできます。
