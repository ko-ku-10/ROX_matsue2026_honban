#キャッチ(catch)がID5で持ち上げ(lift)がID6

import hensuu
import time
from rox_mecanum import ServoPair, Button
from rox_mecanum import Button, pygameDualSense

from rox_mecanum.servo import EncoderServo, ServoConfig

robot = ServoPair.open_from_hensuu(hensuu)
controller = PygameDualSense.open()


try:
    while True:
        state = controller.read()

        if state.button(Button.CIRCLE):
            #位置を初期化
            robot.catch.write(0)
            robot.lift.write(0)
            robot.update(0.02)

            #掴む
            robot.catch.write(30)
            robot.update(0.02)

            #持ち上げる
            robot.lift.write(60)
            robot.update(0.02)

            #放す
            robot.catch.write(0)
            robot.update(0.02)

            #位置を初期化
            robot.catch.write(0)
            robot.lift.write(0)
            robot.update(0.02)
                
except KeyboardInterrupt:
    pass

finally:
    robot.close()

