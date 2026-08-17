"""DualSense からメカナム実機を操作する最小例。"""

from __future__ import annotations

from time import sleep

from rox_mecanum import (
    Button,
    DualSenseMotionMapping,
    MecanumRobot,
    PySerialTransport,
    PygameDualSense,
)


def main() -> None:
    controller = PygameDualSense.open()
    transport = PySerialTransport.open("/dev/ttyUSB1")
    robot = MecanumRobot(transport)
    mapping = DualSenseMotionMapping()
    robot.enable_all()
    try:
        while True:
            state = controller.read()
            if state.button(Button.OPTIONS):
                break
            robot.drive(mapping.command(state))
            sleep(0.02)
    finally:
        robot.stop()
        transport.close()
        controller.close()


if __name__ == "__main__":
    main()
