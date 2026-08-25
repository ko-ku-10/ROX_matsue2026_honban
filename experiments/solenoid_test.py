"""robot_actions.pyに書いたエアシリンダー動作を単体確認する。

L2: 伸ばす / R2: 戻す / OPTIONS: 両方OFFにして終了
実行: python3 -m experiments.solenoid_test
"""

import time

import robot_actions
from rox_mecanum import Button, RobotRuntime


def main() -> None:
    runtime = None
    try:
        runtime = RobotRuntime.open()
        robot_actions.setup_gpio()
        print("L2: 伸ばす / R2: 戻す / OPTIONS: 終了")

        while True:
            state = runtime.controller.read()
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break
            if state.was_pressed(Button.L2):
                robot_actions.game3_cylinder_extend(runtime)
            if state.was_pressed(Button.R2):
                robot_actions.game3_cylinder_retract(runtime)
            time.sleep(0.02)
    finally:
        robot_actions.all_off()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
