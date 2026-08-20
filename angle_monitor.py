"""RobStrideの位置を読むだけの安全な確認プログラム。モーターは有効化も操作もしない。"""

from __future__ import annotations

from math import degrees
import time

import hensuu
from rox_mecanum import ATEncoderReader, PySerialTransport, at_address_from_can_id


transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
reader = ATEncoderReader(
    transport,
    {
        "catch": at_address_from_can_id(hensuu.catch_can_id),
        "lift": at_address_from_can_id(hensuu.lift_can_id),
    },
)

try:
    print("角度監視中。Ctrl+Cで終了。モーターへ速度・有効化指令は送られません。")
    while True:
        # USB-AT変換器は同時要求で応答を落とすことがあるため、1台ずつ読む。
        for name in ("catch", "lift"):
            reader.request(name)
            time.sleep(0.05)
            for feedback in reader.poll():
                if feedback.position_rad is not None:
                    print(
                        f"{feedback.name}: mechPos={feedback.position_rad:+.6f} rad "
                        f"({degrees(feedback.position_rad):+.2f}°)"
                    )
                else:
                    print(f"{feedback.name}: 旧AT生値={feedback.count}（mechPos float形式ではありません）")
        time.sleep(0.10)
finally:
    transport.close()
