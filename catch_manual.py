"""エンコーダーを使わず、CAN ID 5のcatchを手動速度制御する確認用プログラム。"""

import time

import can
import hensuu
from rox_mecanum import Button, PygameDualSense, RobStrideCanMotor


def main() -> None:
    # MKS CANableを直接開く。エンコーダー受信・PID・原点登録は一切使わない。
    bus = can.Bus(
        interface=hensuu.mechanism_can_interface,
        channel=hensuu.mechanism_can_channel,
        bitrate=hensuu.mechanism_can_bitrate,
    )
    motor = RobStrideCanMotor(bus, hensuu.catch_motor_id, host_id=hensuu.mechanism_host_id)
    controller = PygameDualSense.open()
    max_speed = hensuu.catch_speed_percent / 100.0

    print("catch 手動制御を開始します")
    print("  ○を押している間: 正方向")
    print("  ×を押している間: 逆方向")
    print("  OPTIONS: 停止して終了")

    try:
        motor.enable()
        motor.stop()

        while True:
            state = controller.read()
            if state.button(Button.OPTIONS):
                break

            # 同時押し・未押下は必ず停止。意図しない連続回転を防ぐ。
            forward = state.button(Button.CIRCLE)
            backward = state.button(Button.CROSS)
            if forward and not backward:
                speed = max_speed
            elif backward and not forward:
                speed = -max_speed
            else:
                speed = 0.0

            motor.set_velocity(speed)
            time.sleep(0.02)

    finally:
        motor.stop()
        motor.disable()
        bus.shutdown()
        print("catch を停止しました")


if __name__ == "__main__":
    main()
