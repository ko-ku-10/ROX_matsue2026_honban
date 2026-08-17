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

既定の `DualSenseMotionMapping` は元プログラムと同じく L2 押下中のみ移動、R2 押下中のみ旋回を許可します。常に有効にするには `DualSenseMotionMapping(translation_enable=None, rotation_enable=None)` を指定します。

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

## 動作確認

```bash
python -m pytest
```

実機なしで、入力値・スティック角度・運動学・AT フレームをテストできます。
