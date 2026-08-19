"""ID 5のRobStrideモーターがCANへ応答するかだけを確認する安全な診断プログラム。"""

import time

import can
import hensuu
from rox_mecanum.can_motor import build_get_device_id_command


def main() -> None:
    motor_id = hensuu.catch_motor_id
    host_id = hensuu.mechanism_host_id
    command = build_get_device_id_command(motor_id, host_id)

    bus = can.Bus(interface="socketcan", channel=hensuu.mechanism_can_channel)
    try:
        bus.send(can.Message(
            arbitration_id=command.arbitration_id,
            data=command.data,
            is_extended_id=True,
        ))
        print(f"ID {motor_id} へ問い合わせを送信しました: {command.arbitration_id:08X}")

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            message = bus.recv(timeout=0.1)
            if message is not None:
                print(f"返信: {message.arbitration_id:08X}  {bytes(message.data).hex(' ')}")
                return

        print("返信なし: 配線・終端抵抗・電源・CAN ID・プロトコルを確認してください")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
