"""catch（ID 5）とlift（ID 6）を時間式サーボとして扱う。"""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time

import hensuu
from rox_mecanum import ATMotor, PySerialTransport, TimedServo, TimedServoConfig


@dataclass
class ServoMotors:
    """2台の時間式サーボと、共通のUSBシリアル接続。"""

    catch: TimedServo
    lift: TimedServo
    transport: PySerialTransport

    def attach(self) -> None:
        # メカナムと同じく3回再送する。1回だけだと有効化を取りこぼすことがある。
        self.catch.attach(retries=3, interval_sec=0.05)
        self.lift.attach(retries=3, interval_sec=0.05)

    def home(self, catch_angle: float = 0.0, lift_angle: float = 0.0) -> None:
        """実機を各原点に合わせた直後に、両方の角度を登録する。"""
        self.catch.home(catch_angle)
        self.lift.home(lift_angle)

    def close(self) -> None:
        self.catch.detach()
        self.lift.detach()
        self.transport.close()


def _at_address(can_id: int) -> int:
    """CAN IDを、メカナムと同じATシリアル用宛先へ変換する。"""
    return (int(can_id) << 3) + 4


def _config(
    name: str,
    min_angle: float,
    max_angle: float,
    calibration_speed_percent: float,
    move_speed_percent: float,
    time_90deg: float,
    direction: int,
    brake_time_sec: float,
) -> TimedServoConfig:
    if time_90deg <= 0.0:
        raise RuntimeError(f"{name}_90deg_time_sec を90度の実測値（秒）に設定してください")
    return TimedServoConfig(
        min_angle=min_angle,
        max_angle=max_angle,
        degrees_per_second=90.0 / time_90deg,
        calibration_speed=calibration_speed_percent / 100.0,
        default_speed=move_speed_percent / 100.0,
        direction=direction,
        brake_time_sec=brake_time_sec,
    )


def open_servos() -> ServoMotors:
    """catchとliftを開く。使用前に ``attach()`` と ``home()`` を呼ぶ。"""
    transport = PySerialTransport.open(hensuu.serial_port, baudrate=hensuu.serial_baud, minimum_interval=0.0008)
    try:
        return ServoMotors(
            catch=TimedServo(
                ATMotor(transport, _at_address(hensuu.catch_can_id)),
                _config(
                    "catch", hensuu.catch_min_angle, hensuu.catch_max_angle,
                    hensuu.catch_calibration_speed_percent, hensuu.catch_move_speed_percent,
                    hensuu.catch_90deg_time_sec, hensuu.catch_direction, hensuu.catch_brake_time_sec,
                ),
            ),
            lift=TimedServo(
                ATMotor(transport, _at_address(hensuu.lift_can_id)),
                _config(
                    "lift", hensuu.lift_min_angle, hensuu.lift_max_angle,
                    hensuu.lift_calibration_speed_percent, hensuu.lift_move_speed_percent,
                    hensuu.lift_90deg_time_sec, hensuu.lift_direction, hensuu.lift_brake_time_sec,
                ),
            ),
            transport=transport,
        )
    except Exception:
        transport.close()
        raise


def measure_lift_90() -> None:
    """liftを90度動かす時間を、Enterキーで実測する。"""
    transport = PySerialTransport.open(hensuu.serial_port, baudrate=hensuu.serial_baud, minimum_interval=0.0008)
    motor = ATMotor(transport, _at_address(hensuu.lift_can_id))
    try:
        print("liftを安全な0度位置に手で合わせてください。")
        input("準備できたら Enter。liftが回り始めます: ")
        for _ in range(3):
            motor.enable()
            time.sleep(0.05)
        started = time.monotonic()
        motor.set_velocity(hensuu.lift_calibration_speed_percent / 100.0, force=True)
        input("ちょうど90度になった瞬間に Enter: ")
        elapsed = time.monotonic() - started
        print(f"hensuu.py に設定: lift_90deg_time_sec = {elapsed:.3f}")
    finally:
        motor.stop()
        transport.close()


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "measure-lift":
        measure_lift_90()
    else:
        print("ライブラリとして import して使います。liftの90度測定: python3 servos.py measure-lift")
