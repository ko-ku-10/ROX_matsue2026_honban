"""GAME3と同じcatch/lift姿勢を単体確認する実験プログラム。

実行: python3 -m experiments.mechanism_manual
CREATE: 地面保持 / ○: 掴む / □: 排出 / △: 発射姿勢 / ×: 地面保持 / OPTIONS: 終了
"""

from __future__ import annotations

import time

import game3_hensuu as cfg
import hensuu
from rox_mecanum import BallMechanism, Button, open_configured_dualsense
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
        mechanism = BallMechanism(servos)

        print("CREATE/×: 地面保持  ○: 掴む  □: 排出  △: 発射姿勢  OPTIONS: 停止")
        while True:
            state = controller.read()
            if state.was_pressed(Button.OPTIONS):
                break
            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                mechanism.ground()
                print("地面保持姿勢")
            elif state.was_pressed(Button.CIRCLE):
                mechanism.grab()
                print("掴む姿勢")
            elif state.was_pressed(Button.SQUARE):
                mechanism.release()
                print("排出姿勢")
            elif state.was_pressed(Button.TRIANGLE):
                if cfg.lift_fire_angle is None:
                    print("game3_hensuu.py の lift_fire_angle を設定してください")
                else:
                    mechanism.fire_pose(cfg.lift_fire_angle)
                    print(f"発射姿勢: lift={cfg.lift_fire_angle}度")
            time.sleep(0.02)
    finally:
        if servos is not None:
            servos.close()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
