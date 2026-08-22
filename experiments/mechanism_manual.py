import time

import hensuu
from rox_mecanum import Button, PygameDualSense
from servos import open_servos

# 機構の角度
hiraki = 20  
tozi = 10    
age = -10    
sage = -100  
basyo = "原点"

servos = open_servos()
controller = PygameDualSense.open()

try:
    # 起動時に1回だけ原点を登録する
    servos.attach()
    servos.home_from_feedback()
    # PID保持用の低速設定とは別に、目標角度へ動かす時だけ速度を上げる。
    servos.set_pid("catch", max_speed_percent=hensuu.mechanism_move_speed_percent)
    servos.set_pid("lift", max_speed_percent=hensuu.mechanism_move_speed_percent)
    servos.start_pid()  # PID保持を自動で開始する

    while True:
        state = controller.read()

        if state.buttons[Button.CREATE]:
            if basyo == "原点":
                servos.catch.write(0)
                servos.lift.write(110)
                print("下に移動")
                time.sleep(3.0)
                basyo = "下"
            
            elif basyo == "下":
                servos.catch.write(0)
                servos.lift.write(0)
                print("原点に移動")
                time.sleep(3.0)
                basyo = "原点"
                
        if state.buttons[Button.TRIANGLE]:
            print("掴みます")
            servos.lift.write(110)
            time.sleep(3.0)
            
            servos.catch.write(-70)
            time.sleep(2.0)

            servos.lift.write(20)
            time.sleep(3.0)

            servos.catch.write(0)
            time.sleep(2.0)



        time.sleep(0.02)

finally:
    servos.close()
    controller.close()
