"""catch/liftを、実行した瞬間の位置で保持し続ける確認プログラム。"""

import time

from rox_mecanum import Button, open_configured_dualsense
from servos import open_servos

servos = open_servos()
controller = open_configured_dualsense()

try:
    # 停止状態で有効化し、RobStrideから読んだ現在角度を0度として登録する。
    servos.attach()
    servos.home_from_feedback()

    # 「今の位置」を目標にする。この命令時点では誤差が0なので動かない。
    servos.hold_all_current()
    servos.start_pid()

    print("catch/liftを現在位置で保持中。OPTIONSで安全停止して終了")
    while not controller.read().button(Button.OPTIONS):
        time.sleep(0.02)

finally:
    # PID解除、停止指令、シリアル切断の順に必ず安全停止する。
    servos.close()
    controller.close()
