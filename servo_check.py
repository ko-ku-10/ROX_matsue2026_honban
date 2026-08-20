"""PIDを使わず、lift(ID 6)とcatch(ID 5)を同時に確認する。"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import ATEncoderReader, ATMotor, Button, PygameDualSense, PySerialTransport, at_address_from_can_id


TEST_MAX_SPEED = 0.10  # 10%。最初の確認用。
STICK_DEADBAND = 0.15


def stick_speed(value: float) -> float:
    return 0.0 if abs(value) < STICK_DEADBAND else value * TEST_MAX_SPEED


catch_address = at_address_from_can_id(hensuu.catch_can_id)
lift_address = at_address_from_can_id(hensuu.lift_can_id)
transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
# 小さな速度も確かめるため、メカナム用の6%停止帯は使わない。
catch = ATMotor(transport, catch_address, zero_hold_band=0.0)
lift = ATMotor(transport, lift_address, zero_hold_band=0.0)
reader = ATEncoderReader(transport, {"catch": catch_address, "lift": lift_address})
controller = PygameDualSense.open()

try:
    catch.stop()  # 有効化前に前回の速度指令を消す。
    lift.stop()
    for _ in range(3):
        catch.enable()
        lift.enable()
        catch.stop()  # 有効化直後の急発進を防ぐ。
        lift.stop()
        time.sleep(0.05)

    print("サーボ単体確認（PIDなし）")
    print("左スティック上下=lift / R1 + 右スティック上下=catch / OPTIONS=停止終了")
    next_read_at = 0.0

    while True:
        state = controller.read()
        if state.button(Button.OPTIONS):
            print("OPTIONS: 停止")
            catch.stop()
            lift.stop()
            break

        lift_speed = stick_speed(state.left_stick.y)
        # catchはR1を押している間だけ動かす。右スティックの微小なずれで
        # 意図せず回り続けることを防ぐ。
        catch_speed = stick_speed(state.right_stick.y) if state.button(Button.R1) else 0.0
        lift.set_velocity(lift_speed)
        catch.set_velocity(catch_speed)

        now = time.monotonic()
        if now >= next_read_at:
            reader.request_all()
            next_read_at = now + 0.1
        for feedback in reader.poll(now):
            print(
                f"{feedback.name}: 速度指令="
                f"{lift_speed if feedback.name == 'lift' else catch_speed:+.3f} "
                f"encoder={feedback.count:5d}"
            )

        time.sleep(0.01)
finally:
    catch.stop()
    lift.stop()
    transport.close()
    controller.close()
