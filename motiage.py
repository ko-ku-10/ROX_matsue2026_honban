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

            # 初期位置へ
            servos.lift.write(0)
            
            time.sleep(2.0)


        time.sleep(0.02)

finally:
    servos.close()
    controller.close()
