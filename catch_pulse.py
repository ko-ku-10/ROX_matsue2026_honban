"""ID 5のcatchを、エンコーダー・CANable・コントローラーなしで短時間だけ回す。"""

import sys
import time

import hensuu
from rox_mecanum import ATMotor, PySerialTransport


def main() -> None:
    # `python3 catch_pulse.py` は正方向、`python3 catch_pulse.py reverse` は逆方向。
    reverse = len(sys.argv) >= 2 and sys.argv[1].lower() == "reverse"
    speed = hensuu.catch_forward_speed_percent / 100.0
    duration = hensuu.catch_forward_time_sec
    if reverse:
        speed = -(hensuu.catch_reverse_speed_percent / 100.0)
        duration = hensuu.catch_reverse_time_sec

    transport = PySerialTransport.open(
        hensuu.catch_serial_port,
        baudrate=hensuu.catch_serial_baud,
        minimum_interval=hensuu.mecanum_serial_write_interval_sec,
    )
    motor = ATMotor(transport, hensuu.catch_motor_id)

    try:
        print(
            f"CAN ID {hensuu.catch_can_id} "
            f"(AT宛先 0x{hensuu.catch_motor_id:02X}): "
            f"{speed * 100:+.0f}% を {duration:.2f}秒だけ送信します"
        )
        for _ in range(hensuu.catch_enable_retries):
            motor.enable()
            time.sleep(hensuu.catch_enable_interval_sec)

        motor.set_velocity(speed, force=True)
        time.sleep(duration)
    finally:
        motor.stop()
        transport.close()
        print("停止しました")


if __name__ == "__main__":
    main()
