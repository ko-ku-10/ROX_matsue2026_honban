"""liftだけを低速で確認する実験プログラム。

実行: python3 -m experiments.lift_test
△: lift_test_up_angle へ動かす / ×: 地面高さへ戻す / OPTIONS: 停止して終了
"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import Button, PygameDualSense
from servos import open_servos


def main() -> None:
    servos = None
    controller = None
    try:
        servos = open_servos()
        controller = PygameDualSense.open()

        servos.attach()
        print("liftを現在の地面高さ（0度）に合わせてから Enter を押してください")
        input()
        servos.home_from_feedback()
        # 移動時だけ、hensuu.pyで決めた低速上限を使う。
        servos.set_pid("lift", max_speed_percent=hensuu.mechanism_move_speed_percent)
        servos.start_pid()
        servos.lift.write(hensuu.lift_ball_ground_angle)

        print("△: 持上げテスト / ×: 地面高さへ戻す / OPTIONS: 停止して終了")
        while True:
            state = controller.read()
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 停止")
                break
            if state.was_pressed(Button.TRIANGLE):
                servos.lift.write(hensuu.lift_test_up_angle)
                print(f"lift -> {hensuu.lift_test_up_angle}度")
            if state.was_pressed(Button.CROSS):
                servos.lift.write(hensuu.lift_ball_ground_angle)
                print(f"lift -> 地面高さ {hensuu.lift_ball_ground_angle}度")
            time.sleep(0.02)
    finally:
        if servos is not None:
            servos.close()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
