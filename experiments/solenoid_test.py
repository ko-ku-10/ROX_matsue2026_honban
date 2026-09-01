"""ソレノイド（エアシリンダー）だけをDualSenseで確認する。

メカナム、CAN、lift、catchは一切開かない。

実行:
    python3 -m experiments.solenoid_test

操作:
    R1      発射して、そのまま戻す
    OPTIONS 両GPIOをOFFにして終了
"""

import time

import robot_actions
from rox_mecanum import Button, open_configured_dualsense


def main() -> None:
    controller = None

    try:
        controller = open_configured_dualsense()
        robot_actions.setup_gpio()
        print("ソレノイド単体テスト")
        print("R1: 発射して戻す / OPTIONS: 終了")

        while True:
            state = controller.read()
            robot_actions.update_lights()

            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 両方OFFにして終了")
                break

            if state.was_pressed(Button.R1):
                print("発射して戻します")
                robot_actions.ball_fire(None)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nCtrl+C: 両方OFFにして終了")
    finally:
        robot_actions.all_off()
        robot_actions.close_gpio()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
