#!/usr/bin/env python3
"""RobStrideのCAN IDを、mechPos読取りだけで探索する。

速度・有効化・PID・GPIOは一切送らない。CAN IDが想定と違うかを確認するための
読み取り専用プログラム。

実行:
    python3 -m experiments.scan_mechpos_ids
"""

from __future__ import annotations

import time
from math import degrees

import hensuu
from rox_mecanum import ATEncoderReader, PySerialTransport, at_address_from_can_id


FIRST_CAN_ID = 1
LAST_CAN_ID = 31
PASSES = 2
RESPONSE_WAIT_SEC = 0.03


def main() -> None:
    names = {f"ID {can_id}": at_address_from_can_id(can_id) for can_id in range(FIRST_CAN_ID, LAST_CAN_ID + 1)}
    transport = PySerialTransport.open(
        hensuu.serial_port,
        hensuu.serial_baud,
        minimum_interval=0.0008,
    )
    reader = ATEncoderReader(transport, names)
    found: dict[str, float] = {}

    print("=" * 60)
    print(" RobStride CAN ID スキャン（mechPos読取りのみ・モーターは動かない）")
    print(f" ID {FIRST_CAN_ID}〜{LAST_CAN_ID} を {PASSES}回確認します")
    print("=" * 60)

    try:
        for scan_pass in range(PASSES):
            for name in names:
                reader.request(name)
                time.sleep(RESPONSE_WAIT_SEC)
                for feedback in reader.poll():
                    if feedback.position_rad is not None:
                        found[feedback.name] = feedback.position_rad

            print(f"{scan_pass + 1}/{PASSES} 回目完了")

        # 最後の要求に遅れて届いた応答も読む。
        time.sleep(RESPONSE_WAIT_SEC)
        for feedback in reader.poll():
            if feedback.position_rad is not None:
                found[feedback.name] = feedback.position_rad

        if not found:
            print("応答したIDはありませんでした")
        else:
            print("\n応答したCAN ID:")
            for name, position_rad in found.items():
                print(f"  {name}: {position_rad:+.5f} rad ({degrees(position_rad):+.1f}度)")
        print("\n想定: catch=ID 5 / lift=ID 6")
    except KeyboardInterrupt:
        print("\nCtrl+C: 読取りを終了しました（速度指令は送っていません）")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
