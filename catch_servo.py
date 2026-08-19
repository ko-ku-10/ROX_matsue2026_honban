"""時間式サーボとして catch を使う最小例。"""

import hensuu
from rox_mecanum import ATMotor, PySerialTransport, TimedServo, TimedServoConfig


def open_catch() -> tuple[TimedServo, PySerialTransport]:
    transport = PySerialTransport.open(
        hensuu.serial_port,
        baudrate=hensuu.serial_baud,
        minimum_interval=0.0008,
    )
    servo = TimedServo(
        ATMotor(transport, (hensuu.catch_can_id << 3) + 4),
        TimedServoConfig(
            min_angle=hensuu.catch_min_angle,
            max_angle=hensuu.catch_max_angle,
            degrees_per_second=90.0 / hensuu.catch_90deg_time_sec,
            calibration_speed=hensuu.catch_speed_percent / 100.0,
            direction=hensuu.catch_direction,
        ),
    )
    return servo, transport


def main() -> None:
    servo, transport = open_catch()
    try:
        servo.attach()
        input("実機を0度に合わせて Enter: ")
        servo.home(0)
        servo.write(90)  # 90度へ移動
        servo.write(0)   # 0度へ戻る
    finally:
        servo.detach()
        transport.close()


if __name__ == "__main__":
    main()
