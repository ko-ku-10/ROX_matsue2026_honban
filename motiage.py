"""○ボタンで catch と lift の一連動作を行う実機プログラム。"""

import time

import hensuu
from rox_mecanum import Button, PygameDualSense, ServoPair


# (catchの目標角度, liftの目標角度, 次の動作までの秒数)
SEQUENCE = (
    (0.0, 0.0, 0.5),    # 初期位置
    (30.0, 0.0, 0.5),   # つかむ
    (30.0, 60.0, 0.8),  # 持ち上げる
    (0.0, 60.0, 0.5),   # 放す
    (0.0, 0.0, 0.8),    # 初期位置へ戻る
)


def set_step(robot: ServoPair, index: int) -> float:
    """指定した一連動作を開始し、次の切替時刻を返す。"""
    catch_deg, lift_deg, duration_sec = SEQUENCE[index]
    robot.catch.write(catch_deg)
    robot.lift.write(lift_deg)
    print(f"step {index + 1}: catch={catch_deg:g}°, lift={lift_deg:g}°")
    return time.monotonic() + duration_sec


robot = ServoPair.open_from_hensuu(hensuu)
controller = PygameDualSense.open()

last_circle_state = False
sequence_index: int | None = None
next_step_at = 0.0

try:
    # CAN ID 5/6を有効化し、受信した現在位置を原点として登録する。
    # 必ず機構を決めた物理原点位置に置いてから起動すること。
    robot.begin()
    print("準備完了。○ボタンで一連動作を開始します。")

    while True:
        # 常にCAN受信・PID計算・CAN速度送信を進める。ここを止めないこと。
        robot.update(0.0)

        state = controller.read()
        circle_pressed = state.button(Button.CIRCLE)

        # ○を押した瞬間に一連動作を開始する。実行中の押し直しは最初からやり直す。
        if circle_pressed and not last_circle_state:
            print("一連の動作を開始します")
            sequence_index = 0
            next_step_at = set_step(robot, sequence_index)

        # sleepでPIDを止めず、時刻で次の目標角度へ切り替える。
        if sequence_index is not None and time.monotonic() >= next_step_at:
            sequence_index += 1
            if sequence_index >= len(SEQUENCE):
                sequence_index = None
                print("動作完了")
            else:
                next_step_at = set_step(robot, sequence_index)

        last_circle_state = circle_pressed
        time.sleep(0.005)  # 約200Hz。CPUを使い切らずにPIDを継続する。

except KeyboardInterrupt:
    print("停止します")

finally:
    robot.close()
