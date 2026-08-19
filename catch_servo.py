"""時間式サーボとして catch を使う最小例。"""

import hensuu
from rox_mecanum import ATMotor, PySerialTransport, TimedServo, TimedServoConfig


def open_catch() -> tuple[TimedServo, PySerialTransport]:
    if hensuu.catch_90deg_time_sec <= 0.0:
        raise RuntimeError("先に python3 catch_calibrate.py を実行して catch_90deg_time_sec を設定してください")
    transport = PySerialTransport.open(
        hensuu.catch_serial_port,
        baudrate=hensuu.catch_serial_baud,
        minimum_interval=hensuu.mecanum_serial_write_interval_sec,
    )
    servo = TimedServo(
        ATMotor(transport, hensuu.catch_motor_id),
        TimedServoConfig(
            min_angle=hensuu.catch_min_position_deg,
            max_angle=hensuu.catch_max_position_deg,
            degrees_per_second=90.0 / hensuu.catch_90deg_time_sec,
            calibration_speed=hensuu.catch_calibration_speed_percent / 100.0,
            direction=hensuu.catch_time_servo_direction,
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
