"""L2を押すと、本番と同じソレノイド発射処理を単体確認する。"""

import time

import hensuu
import solenoid
from rox_mecanum import Button, open_configured_dualsense
from rox_mecanum.solenoid import RDKSolenoid


def main() -> None:
    output = RDKSolenoid(hensuu.solenoid_pin)
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
                # GAME2/GAME3と同じ solenoid.py の処理。
                solenoid.fire(output)
                print(f"ソレノイド ON ({solenoid.on_time_sec}秒)")
            was_pressed = pressed
            output.update()
            time.sleep(0.02)
    finally:
        output.close()
        controller.close()


if __name__ == "__main__":
    main()
