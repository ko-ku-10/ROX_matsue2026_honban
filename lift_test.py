#!/usr/bin/env python3
"""持上げ機構だけを確認するテスト。

実行: python3 lift_test.py

メカナム、DualSense、カメラ、ソレノイドは起動しない。
robot_actions.py の ball_lift_for_shot() と同じ動きを確認する。
終了は Ctrl+C。
"""

import time
from types import SimpleNamespace

import robot_actions
from servos import open_servos


servos = open_servos()
runtime = SimpleNamespace(servos=servos)

try:
    # catch/liftだけを有効化し、今の実測角度を0度としてPIDを開始する。
    servos.attach()
    servos.home_from_feedback()
    servos.hold_all_current()
    servos.start_pid()

    print("持上げテストを開始します")
    print("メカナム・ソレノイドは動きません")

    # GAME2・GAME3と同じ持上げ動作。
    robot_actions.ball_lift_for_shot(runtime)

    print("動作指令が終わりました。位置を確認して Ctrl+C で停止してください")
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("持上げテストを停止しました")

finally:
    # PIDを解除し、catch/liftを停止してシリアル接続を閉じる。
    servos.close()
