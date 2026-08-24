"""L2を押すとソレノイドを指定時間だけONにする。"""

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
                solenoid.on()
                time.sleep(hensuu.solenoid_time_sec)
                solenoid.off()
            was_pressed = pressed
            time.sleep(0.02)
    finally:
        solenoid.close()
        controller.close()


if __name__ == "__main__":
    main()
