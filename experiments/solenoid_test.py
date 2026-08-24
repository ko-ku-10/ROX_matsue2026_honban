"""L2を押すと、本番と同じソレノイド発射処理を単体確認する。"""

import time

import hensuu
from rox_mecanum import Button, open_configured_dualsense
from rox_mecanum.solenoid import RDKSolenoid


def main() -> None:
    solenoid = RDKSolenoid(hensuu.solenoid_pin)
    controller = open_configured_dualsense()
    was_pressed = False
    print("L2: ソレノイド  /  OPTIONS: 終了")
    try:
        while True:
            state = controller.read()
            if state.button(Button.OPTIONS):
                break
            pressed = state.button(Button.L2)
            if pressed and not was_pressed:
                # GAME2/GAME3の RobotRuntime.fire() と同じ共通処理。
                solenoid.pulse(hensuu.solenoid_time_sec)
                print(f"ソレノイド ON ({hensuu.solenoid_time_sec}秒)")
            was_pressed = pressed
            solenoid.update()
            time.sleep(0.02)
    finally:
        solenoid.close()
        controller.close()


if __name__ == "__main__":
    main()
