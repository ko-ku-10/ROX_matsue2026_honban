#!/usr/bin/env python3
"""持ち上げ機構だけを、DualSenseで練習するプログラム。

メカナムとソレノイドは一切操作しない。
本番と同じ robot_actions.py の動作を呼ぶため、ここで確認した持上げ動作は
GAME2・GAME3にもそのまま使われる。

実行:
    python3 -m experiments.lift_practice
"""

from __future__ import annotations

from types import SimpleNamespace

import robot_actions
from rox_mecanum import Button, open_configured_dualsense
from servos import open_servos


def main() -> None:
    print("=" * 54)
    print("持ち上げ機構の操作練習（メカナム・ソレノイドは動きません）")
    print("CREATE または ×: 地面走行姿勢")
    print("○: 掴む姿勢 / □: 発射台へ載せる前の姿勢")
    print("△: 持上げて発射台へ載せる")
    print("OPTIONS: 機構を停止して終了")
    print("=" * 54)

    controller = None
    servos = None

    try:
        controller = open_configured_dualsense()
        servos = open_servos()
        servos.attach()
        servos.home_from_feedback()
        servos.hold_all_current()
        servos.start_pid()

        # robot_actions.py の関数は runtime.servos だけを使う。
        # この練習プログラムでは、必要な部分だけを渡す。
        runtime = SimpleNamespace(servos=servos)

        while True:
            state = controller.read()

            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 停止します")
                break

            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                print("地面走行姿勢")
                robot_actions.game3_ground_pose(runtime)

            elif state.was_pressed(Button.CIRCLE):
                print("掴む姿勢")
                robot_actions.game3_grab(runtime)

            elif state.was_pressed(Button.SQUARE):
                print("発射台へ載せる前の姿勢")
                robot_actions.game3_release(runtime)

            elif state.was_pressed(Button.TRIANGLE):
                print("持上げて発射台へ載せます")
                # robot_actions.py 内の time.sleep 中はOPTIONSを読めない。
                robot_actions.ball_lift_for_shot(runtime)
                print("持上げ動作が終わりました")

    except KeyboardInterrupt:
        print("\nCtrl+C: 停止します")
    finally:
        if servos is not None:
            servos.release()
            servos.close()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
