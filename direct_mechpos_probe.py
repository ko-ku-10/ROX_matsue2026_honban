"""SLCAN経由でRobStride標準type 17 / mechPos(0x7019)を直接読む。

速度・有効化・PIDの命令は送らない。MKS CANable系SLCANファームウェア用。
"""

from __future__ import annotations

from math import degrees, isfinite
from struct import unpack
import time

import hensuu


COMM_READ_ONE = 0x11
MASTER_ID = 0xFD
MECH_POS_INDEX = 0x7019


def extended_id(mode: int, data16: int, node_id: int) -> int:
    return ((mode & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (node_id & 0xFF)


def request_mech_pos(bus, motor_id: int):
    import can

    message = can.Message(
        arbitration_id=extended_id(COMM_READ_ONE, MASTER_ID << 8, motor_id),
        is_extended_id=True,
        data=bytes((0x19, 0x70, 0, 0, 0, 0, 0, 0)),
    )
    bus.send(message)

    deadline = time.monotonic() + 0.10
    while time.monotonic() < deadline:
        response = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if response is None or not response.is_extended_id or len(response.data) != 8:
            continue
        mode = (response.arbitration_id >> 24) & 0x1F
        index = int.from_bytes(response.data[0:2], "little")
        if mode != COMM_READ_ONE or index != MECH_POS_INDEX:
            continue
        source_id = (response.arbitration_id >> 16) & 0xFF
        position_rad = unpack("<f", bytes(response.data[4:8]))[0]
        if isfinite(position_rad):
            return source_id, position_rad
    return None


try:
    import can
except ImportError as error:
    raise SystemExit("python-canが必要です: python3 -m pip install --user python-can") from error


try:
    bus = can.Bus(
        interface="slcan",
        channel=hensuu.serial_port,
        bitrate=1_000_000,
        tty_baudrate=115200,
    )
except Exception as error:
    raise SystemExit(f"SLCANを開けませんでした: {error}") from error

try:
    print("RobStride直接mechPos読取り。Ctrl+Cで終了。モーターは動きません。")
    print(f"SLCAN: {hensuu.serial_port} / CAN 1Mbps")
    while True:
        for name, motor_id in (("catch", hensuu.catch_can_id), ("lift", hensuu.lift_can_id)):
            result = request_mech_pos(bus, motor_id)
            if result is None:
                print(f"{name}: 応答なし")
            else:
                source_id, position_rad = result
                print(f"{name}: id={source_id} mechPos={position_rad:+.6f} rad ({degrees(position_rad):+.2f}°)")
        time.sleep(0.1)
finally:
    bus.shutdown()
