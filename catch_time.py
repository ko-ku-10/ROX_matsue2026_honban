"""エンコーダーなしでCAN ID 5のcatchを時間制御する実機プログラム。"""

import time

import hensuu
from rox_mecanum import ATMotor, Button, PygameDualSense, PySerialTransport


def main() -> None:
    # メカナムと同じATシリアル制御を使う。CANable・CAN受信・PIDは使わない。
    transport = PySerialTransport.open(
        hensuu.catch_serial_port,
        baudrate=hensuu.catch_serial_baud,
        minimum_interval=hensuu.mecanum_serial_write_interval_sec,
    )
    motor = ATMotor(transport, hensuu.catch_motor_id)
    controller = PygameDualSense.open()

    last_circle = False
    last_cross = False
    active_speed = 0.0
    stop_at = 0.0

    print("catch 時間制御を開始します")
    print("  ○: 正方向へ指定時間だけ回す")
    print("  ×: 逆方向へ指定時間だけ回す")
    print("  OPTIONS: 停止して終了")

    try:
        motor.enable()
        motor.stop()

        while True:
            state = controller.read()
            if state.button(Button.OPTIONS):
                break

            circle = state.button(Button.CIRCLE)
            cross = state.button(Button.CROSS)

            # 押した瞬間だけタイマーを開始する。押しっぱなしでも1回だけ動く。
            if circle and not last_circle:
                active_speed = hensuu.catch_forward_speed_percent / 100.0
                stop_at = time.monotonic() + hensuu.catch_forward_time_sec
            elif cross and not last_cross:
                active_speed = -(hensuu.catch_reverse_speed_percent / 100.0)
                stop_at = time.monotonic() + hensuu.catch_reverse_time_sec

            if time.monotonic() < stop_at:
                motor.set_velocity(active_speed)
            else:
                motor.stop()

            last_circle = circle
            last_cross = cross
            time.sleep(0.02)

    finally:
        motor.stop()
        transport.close()
        print("catch を停止しました")


if __name__ == "__main__":
    main()
