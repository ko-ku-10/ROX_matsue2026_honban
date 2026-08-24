"""GAME3と同じcatch/lift姿勢を単体確認する実験プログラム。

角度は game3.py を直接使う。ここだけを編集してもGAME3には反映されない。

実行: python3 -m experiments.mechanism_manual
CREATE: 地面保持 / ○: 掴む / □: 排出 / △: 発射姿勢 / ×: 地面保持 / OPTIONS: 終了
"""

from __future__ import annotations

import time

import game3
import hensuu
from rox_mecanum import Button, open_configured_dualsense
from servos import open_servos


def main() -> None:
    servos = None
    controller = None
    try:
        servos = open_servos()
        controller = open_configured_dualsense()
        servos.attach()
        print("catch/liftを現在の0度位置へ合わせてから Enter を押してください")
        input()
        servos.home_from_feedback()
        servos.set_pid("catch", max_speed_percent=hensuu.mechanism_move_speed_percent)
        servos.set_pid("lift", max_speed_percent=hensuu.mechanism_move_speed_percent)
        servos.hold_all_current()
        servos.start_pid()
        print("CREATE/×: 地面保持  ○: 掴む  □: 排出  △: 発射姿勢  OPTIONS: 停止")
        while True:
            state = controller.read()
            if state.was_pressed(Button.OPTIONS):
                break
            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                servos.catch.write(game3.GROUND_CATCH_ANGLE)
                servos.lift.write(game3.GROUND_LIFT_ANGLE)
                print("地面保持姿勢")
            elif state.was_pressed(Button.CIRCLE):
                servos.catch.write(game3.GRAB_CATCH_ANGLE)
                print("掴む姿勢")
            elif state.was_pressed(Button.SQUARE):
                servos.catch.write(game3.RELEASE_CATCH_ANGLE)
                print("排出姿勢")
            elif state.was_pressed(Button.TRIANGLE):
                servos.catch.write(game3.RELEASE_CATCH_ANGLE)
                servos.lift.write(game3.LIFT_FIRE_ANGLE)
                print(f"発射姿勢: lift={game3.LIFT_FIRE_ANGLE}度")
            time.sleep(0.02)
    finally:
        if servos is not None:
            servos.close()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
