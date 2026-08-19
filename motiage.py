import time

from rox_mecanum import Button, PygameDualSense
from servos import open_servos

# 機構の角度
hiraki = 20  
tozi = 10    
age = -10    
sage = -100  

servos = open_servos()
controller = PygameDualSense.open()

try:
    # 起動時に1回だけ原点を登録する
    servos.attach()
    servos.home_from_feedback()
    servos.start_pid()  # PID保持を自動で開始する

    print("○: 持ち上げ  /  OPTIONS: 終了")

    while True:
        state = controller.read()

        if state.button(Button.OPTIONS):
            break

        if state.was_pressed(Button.CIRCLE):
            # 初期位置へ
            servos.catch.write(hiraki)
            servos.lift.write(sage)
            time.sleep(1.0)

            # ボールを掴む
            servos.catch.write(tozi)
            time.sleep(1.0)

            # ボールを装填
            servos.lift.write(age)
            time.sleep(1.0)

            # 初期位置へ戻す
            servos.catch.write(hiraki)
            servos.lift.write(sage)

        time.sleep(0.02)

finally:
    servos.close()
    controller.close()
