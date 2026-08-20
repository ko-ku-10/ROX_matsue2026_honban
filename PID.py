import time

from rox_mecanum import Button, PygameDualSense
from servos import open_servos

servos = open_servos()
controller = PygameDualSense.open()

# ボタンを押した瞬間だけ判定するための記録
before = {
    Button.CIRCLE: False,
    Button.CROSS: False,
    Button.TRIANGLE: False,
    Button.SQUARE: False,
}

try:
    servos.attach()

    # エンコーダーを読んで、起動時の物理位置を両方とも目標位置にする
    # → 起動した瞬間に動かず、その位置を保持する
    servos.home_from_feedback()

    # PIDの自動更新を開始（約50Hz）
    servos.start_pid()

    print("起動時の位置で保持中")
    print("○: catchを今の位置で保持")
    print("×: catchのPIDをOFF")
    print("△: liftを今の位置で保持")
    print("□: liftのPIDをOFF")
    print("OPTIONS: 終了")

    while True:
        state = controller.read()

        if state.button(Button.OPTIONS):
            break

        circle = state.button(Button.CIRCLE)
        cross = state.button(Button.CROSS)
        triangle = state.button(Button.TRIANGLE)
        square = state.button(Button.SQUARE)

        # ○を押した瞬間: catchの「今の位置」を目標にしてPID保持
        if circle and not before[Button.CIRCLE]:
            servos.catch.hold_current()
            print("catch: PID ON（現在位置を保持）")

        # ×を押した瞬間: catchのPID保持を解除
        if cross and not before[Button.CROSS]:
            servos.catch.pid_off()
            print("catch: PID OFF")

        # △を押した瞬間: liftの「今の位置」を目標にしてPID保持
        if triangle and not before[Button.TRIANGLE]:
            servos.lift.hold_current()
            print("lift: PID ON（現在位置を保持）")

        # □を押した瞬間: liftのPID保持を解除
        if square and not before[Button.SQUARE]:
            servos.lift.pid_off()
            print("lift: PID OFF")

        before[Button.CIRCLE] = circle
        before[Button.CROSS] = cross
        before[Button.TRIANGLE] = triangle
        before[Button.SQUARE] = square

        time.sleep(0.02)

finally:
    # PID停止、モーター停止、通信終了
    servos.close()
    controller.close()