"""6個のLEDテープだけを順番に確認する。

実行:
    python3 -m experiments.led_test

先に robot_lights.py の LED_OUTPUT_ENABLED を True にすること。
メカナム、CAN、サーボ、ソレノイド、コントローラーは使用しない。
"""

from __future__ import annotations

import time

from robot_lights import LED_OUTPUT_ENABLED, lights


def run_animation(name: str, start, seconds: float) -> None:
    print(f"{name}: {seconds:.1f}秒")
    start()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        lights.update()
        time.sleep(0.02)


def main() -> None:
    if not LED_OUTPUT_ENABLED:
        raise RuntimeError(
            "robot_lights.py の LED_OUTPUT_ENABLED = True にしてから実行してください"
        )

    print("LEDテープ単体テスト。Ctrl+Cまたは終了時に消灯します。")
    try:
        run_animation("待機", lights.standby, 2.0)
        run_animation("地面走行", lights.ground, 2.0)
        run_animation("持上げ", lights.lifting, 2.0)
        run_animation("発射準備", lights.ready_to_fire, 2.0)
        run_animation("発射", lights.firing, 2.0)
    finally:
        lights.off()
        lights.close()
        print("LEDを消灯しました")


if __name__ == "__main__":
    main()
