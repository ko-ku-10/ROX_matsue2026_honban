"""PIDを使わず、lift(ID 6)とcatch(ID 5)を同時に確認する。"""

from __future__ import annotations

import time
from math import degrees

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
    print("左スティック上下=lift / R1+○=catch正方向 / R1+×=catch逆方向 / OPTIONS=停止終了")
    next_read_at = 0.0
    last_catch_speed: float | None = None

    while True:
        state = controller.read()
        if state.button(Button.OPTIONS):
            print("OPTIONS: 停止")
            catch.stop()
            lift.stop()
            break

        lift_speed = stick_speed(state.left_stick.y)
        # catchは右スティックを使わない。R1+○とR1+×で正逆転を明確に分ける。
        # 右スティックの中立ずれ・軸割当の違いによる誤動作を避けるため。
        catch_armed = state.button(Button.R1)
        if catch_armed and state.button(Button.CIRCLE) and not state.button(Button.CROSS):
            catch_speed = TEST_MAX_SPEED
        elif catch_armed and state.button(Button.CROSS) and not state.button(Button.CIRCLE):
            catch_speed = -TEST_MAX_SPEED
        else:
            catch_speed = 0.0
        if catch_speed != last_catch_speed:
            print(f"catch 指令: {catch_speed:+.3f}")
            last_catch_speed = catch_speed
        lift.set_velocity(lift_speed)
        if catch_speed:
            catch.set_velocity(catch_speed)
        else:
            catch.stop()

        now = time.monotonic()
        if now >= next_read_at:
            reader.request_all()
            next_read_at = now + 0.1
        for feedback in reader.poll(now):
            position = (
                f"mechPos={degrees(feedback.position_rad):+.2f}°"
                if feedback.position_rad is not None
                else f"旧AT生値={feedback.count}"
            )
            print(
                f"{feedback.name}: 速度指令="
                f"{lift_speed if feedback.name == 'lift' else catch_speed:+.3f} "
                f"{position}"
            )

        time.sleep(0.01)
finally:
    catch.stop()
    lift.stop()
    transport.close()
    controller.close()
