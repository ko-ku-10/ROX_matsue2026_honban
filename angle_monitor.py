#!/usr/bin/env python3
"""catch / lift のmechPos応答を生データで表示する一時プログラム。

モーターを回す命令、PID保持、原点登録は行いません。
RobStride 05 に mechPos (0x7019) を問い合わせ、返ってきた通信データを
ラジアン・度へ変換せず、10進数のバイト値としてターミナルに表示します。

実行中は CAN-USB 変換器をこのプログラムだけで使ってください。
ゲームプログラム、mecanum.py、game3.py などを同時に起動してはいけません。
終了は Ctrl+C です。
"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import ATEncoderReader, PySerialTransport, at_address_from_can_id


# 表示を更新する間隔。小さくすると数字が速く流れます。
DISPLAY_INTERVAL_SEC = 0.10


def main() -> None:
    transport = PySerialTransport.open(
        hensuu.serial_port,
        hensuu.serial_baud,
        minimum_interval=0.0008,
    )
    reader = ATEncoderReader(
        transport,
        {
            "catch": at_address_from_can_id(hensuu.catch_can_id),
            "lift": at_address_from_can_id(hensuu.lift_can_id),
        },
    )

    # 最新のAT応答フレームを機構ごとに控える。
    raw_frames: dict[str, bytes] = {}
    next_display_at = 0.0

    print("=" * 56)
    print("  catch / lift のmechPos生データを表示します（読み取り専用）")
    print("  Ctrl+C で終了")
    print("  表示値はすべて変換前の10進数バイト値です")
    print("  最初の行の『モーター位置（変換なし・10進数）』を確認してください")
    print("=" * 56)

    try:
        while True:
            now = time.monotonic()

            # USB-AT変換器が応答を落とさないよう、catch/liftを交互に要求する。
            reader.request_next()
            time.sleep(0.015)

            for feedback in reader.poll():
                if feedback.position_rad is None:
                    # 正式mechPosではない旧ステータス応答は角度として使わない。
                    continue
                raw_frames[feedback.name] = feedback.raw_at_frame

            if now >= next_display_at:
                lines = []
                for name in ("catch", "lift"):
                    if name not in raw_frames:
                        lines.append(f"{name}: 応答待ち")
                        continue
                    frame = raw_frames[name]
                    payload = frame[7:15]
                    position_bytes = payload[4:8]
                    position_bits = int.from_bytes(position_bytes, "little")
                    lines.append(
                        f"{name}: モーター位置（変換なし・10進数）= {position_bits}\n"
                        f"  AT応答（10進数）: {' '.join(str(value) for value in frame)}\n"
                        f"  データ8バイト（10進数）: {' '.join(str(value) for value in payload)}\n"
                        f"  mechPosの4バイト（10進数）: {' '.join(str(value) for value in position_bytes)}\n"
                        f"  mechPos生bit列（uint32 little-endian）: {position_bits}"
                    )
                print("\n".join(lines))
                next_display_at = now + DISPLAY_INTERVAL_SEC

            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n角度表示を終了しました。モーターへの駆動命令は送っていません。")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
