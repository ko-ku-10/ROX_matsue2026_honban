# キャッチ(catch)がID5で持ち上げ(lift)がID6

import time
import hensuu

from rox_mecanum import (
    Button,
    PygameDualSense,
    ServoPair,
)
from rox_mecanum.servo import EncoderServo, ServoConfig

robot = ServoPair.open_from_hensuu(hensuu)
controller = PygameDualSense.open()

# ボタンの前回の状態を記憶（1回だけ押す動作のため）
last_circle_state = False

try:
    while True:
        state = controller.read()
        current_circle = state.button(Button.CIRCLE)

        # 〇ボタンが「押された瞬間」だけ実行（押しっぱなし対策）
        if current_circle and not last_circle_state:
            print("一連の動作を開始します...")

            # 1. 位置を初期化
            robot.catch.write(0)
            robot.lift.write(0)
            time.sleep(0.5)  # モーターが動く時間を確保

            # 2. 掴む
            robot.catch.write(30)
            time.sleep(0.5)

            # 3. 持ち上げる
            robot.lift.write(60)
            time.sleep(0.8)

            # 4. 放す
            robot.catch.write(0)
            time.sleep(0.5)

            # 5. 位置を初期化
            robot.catch.write(0)
            robot.lift.write(0)
            time.sleep(0.8)

            print("動作完了")

        # ボタンの状態を更新
        last_circle_state = current_circle

        # コントローラーの読み取り周期調整
        time.sleep(0.02)

except KeyboardInterrupt:
    pass

finally:
    robot.close()