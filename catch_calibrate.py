"""catch を90度動かす時間を実測するためのプログラム。"""

import time

import hensuu
from rox_mecanum import ATMotor, PySerialTransport


def main() -> None:
    speed = hensuu.catch_calibration_speed_percent / 100.0
    transport = PySerialTransport.open(
        hensuu.catch_serial_port,
        baudrate=hensuu.catch_serial_baud,
        minimum_interval=hensuu.mecanum_serial_write_interval_sec,
    )
    motor = ATMotor(transport, hensuu.catch_motor_id)

    try:
        print("catch 90度測定")
        print("機構を安全な原点（0度）に手で合わせてください。")
        input("準備できたら Enter。以後、モーターが回り始めます: ")
        motor.enable()
        started = time.monotonic()
        motor.set_velocity(speed, force=True)
        input("ちょうど90度になった瞬間に Enter: ")
        elapsed = time.monotonic() - started
        print(f"90度の時間: {elapsed:.3f} 秒")
        print(f"角速度: {90.0 / elapsed:.3f} 度/秒")
        print("hensuu.py へ次を設定してください:")
        print(f"catch_90deg_time_sec = {elapsed:.3f}")
    finally:
        motor.stop()
        transport.close()
        print("catch を停止しました")


if __name__ == "__main__":
    main()
