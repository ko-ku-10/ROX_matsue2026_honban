"""DualSense の全入力を表示する。実行には pygame が必要。"""

from __future__ import annotations

from time import sleep

from rox_mecanum import PygameDualSense


def main() -> None:
    controller = PygameDualSense.open()
    print(f"connected: {controller.name}")
    print("OPTIONS または Ctrl+C で終了")
    try:
        while True:
            state = controller.read()
            print(
                f"left=({state.left_stick.x:+.2f}, {state.left_stick.y:+.2f}) "
                f"angle={state.left_stick.angle_degrees!s:>6}° "
                f"right=({state.right_stick.x:+.2f}, {state.right_stick.y:+.2f}) "
                f"angle={state.right_stick.angle_degrees!s:>6}° "
                f"L2={state.l2:.2f} R2={state.r2:.2f} "
                f"buttons={[button.value for button in state.active_buttons]} "
                f"raw_axes={state.raw_axes} raw_buttons={state.raw_buttons}",
                end="\r",
                flush=True,
            )
            if state.button("options"):
                break
            sleep(0.02)
    finally:
        controller.close()
        print()


if __name__ == "__main__":
    main()
