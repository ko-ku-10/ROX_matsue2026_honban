from rox_mecanum import Button, PygameDualSense
from gpiozero import LED
import hensuu
import time

sore = LED(hensuu.sore_pin)



Controller = PygameDualSense.open()
try:
    while True:
        state = comtroller.read()

        if state.button(Button.L2):
            sore.on()
            sleep(hensuu.sore_time)
            sore.off()

finally:
    controller.close()



