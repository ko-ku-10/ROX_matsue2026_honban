import time

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
    servos.start_pid()  # PID保持を自動で開始する

    while True:
        state = controller.read()

        if state.buttons[Button.CREATE]:
            if basyo == "原点":
                servos.catch.write(0)
                servos.lift.write(105)
                print("下に移動")
                while servos.lift.read() >= 105
                basyo = "下"
            
            elif basyo == "下":
                servos.catch.write(0)
                servos.lift.write(0)
                print("原点に移動")
                while servos.lift.read() >= 0
                basyo = "原点"
                
        if state.buttons[Button.TRIANGLE]:
            print("掴みます")
            servos.lift.write(105)
            while servos.lift.read() >= 105

            servos.catch.write(-70)
            while servos.catch.read() <= -30

            servos.lift.write(0)
            while servos.lift.read() >= 0

            servos.catch.write(0)
            while servos.catch.read() <= 0



        time.sleep(0.02)

finally:
    servos.close()
    controller.close()
