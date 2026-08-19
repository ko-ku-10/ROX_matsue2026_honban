from servos import open_servos
import time

servos = open_servos()

try:
    servos.attach()

    # 実機を両方とも0度位置に合わせてから実行
    servos.home()

    servos.lift.write(45)   # ID 5: catchを45度へ
    time.sleep(1.0)
    servos.lift.write(90)  # ID 6: liftを90度へ


finally:
    servos.close()