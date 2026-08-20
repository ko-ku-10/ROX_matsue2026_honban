"""PIDを使わず、lift(ID 6)の正逆転とエンコーダーを確認する。"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import ATEncoderReader, ATMotor, Button, PygameDualSense, PySerialTransport, at_address_from_can_id


# 最初の動作確認用。危険なら必ずこの値をさらに小さくする。
TEST_MAX_SPEED = 0.10  # 10%
STICK_DEADBAND = 0.15

address = at_address_from_can_id(hensuu.lift_can_id)
transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
# PID用は小さな出力も確認できるよう、メカナム用の6%停止帯は使わない。
lift = ATMotor(transport, address, zero_hold_band=0.0)
reader = ATEncoderReader(transport, {"lift": address})
controller = PygameDualSense.open()

try:
    for _ in range(3):
        lift.enable()
        time.sleep(0.05)
    lift.stop()

    print("lift単体確認: 左スティック上=正方向、下=逆方向、OPTIONS=停止終了")
    print("エンコーダーのcountも表示します。PIDは一切動きません。")
    next_read_at = 0.0

    while True:
        state = controller.read()
        if state.button(Button.OPTIONS):
            print("OPTIONS: 停止")
            lift.stop()
            break

        stick = state.left_stick.y
        speed = 0.0 if abs(stick) < STICK_DEADBAND else stick * TEST_MAX_SPEED
        lift.set_velocity(speed)

        now = time.monotonic()
        if now >= next_read_at:
            reader.request_all()
            next_read_at = now + 0.1
        for feedback in reader.poll(now):
            print(f"速度指令={speed:+.3f}  encoder={feedback.count:5d}")

        time.sleep(0.01)
finally:
    lift.stop()
    transport.close()
    controller.close()
