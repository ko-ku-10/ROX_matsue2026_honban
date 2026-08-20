"""SLCAN経由でRobStrideの標準GET_IDを全IDに送る。モーターは動かさない。"""

from __future__ import annotations

import time

import hensuu


COMM_GET_ID = 0x00
MASTER_ID = 0xFD
RESPONSE_MARKER = 0xFE


def extended_id(mode: int, data16: int, node_id: int) -> int:
    return ((mode & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (node_id & 0xFF)


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
    print("RobStride標準GET_IDスキャン。モーターの有効化・速度指令は送りません。")
    for target_id in range(128):
        bus.send(
            can.Message(
                arbitration_id=extended_id(COMM_GET_ID, MASTER_ID << 8, target_id),
                is_extended_id=True,
                data=bytes(8),
            )
        )
        time.sleep(0.003)

    found: set[int] = set()
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        message = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if message is None or not message.is_extended_id or len(message.data) != 8:
            continue
        mode = (message.arbitration_id >> 24) & 0x1F
        response_marker = message.arbitration_id & 0xFF
        if mode == COMM_GET_ID and response_marker == RESPONSE_MARKER:
            motor_id = (message.arbitration_id >> 8) & 0xFF
            found.add(motor_id)
            print(f"発見: RobStride CAN ID {motor_id}")

    if not found:
        print("応答なし: SLCAN設定、CAN-H/L、終端抵抗、電源、CAN 1Mbps、変換器モードを確認")
    else:
        print("検出ID:", ", ".join(map(str, sorted(found))))
finally:
    bus.shutdown()
