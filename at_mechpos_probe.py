"""AT変換器経由の0x7019(mechPos)応答を、生フレームのまま確認する。

モーターの有効化・速度指令は一切送らない。
"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import PySerialTransport, at_address_from_can_id
from rox_mecanum.feedback_servo import build_encoder_read_command


addresses = {
    "catch": at_address_from_can_id(hensuu.catch_can_id),
    "lift": at_address_from_can_id(hensuu.lift_can_id),
}
transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)

try:
    print("AT mechPos(0x7019) 生フレーム確認。Ctrl+Cで終了。モーターは動きません。")
    while True:
        for name, address in addresses.items():
            request = build_encoder_read_command(address)
            transport.write(request)
            time.sleep(0.05)
            response = transport.read_available()
            print(f"{name} 要求: {request.hex(' ')}")
            print(f"{name} 応答: {response.hex(' ') if response else '(なし)'}")
        time.sleep(0.5)
finally:
    transport.close()
