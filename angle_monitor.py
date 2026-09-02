#!/usr/bin/env python3
"""catch / lift の実測角度を表示するだけの一時プログラム。

モーターを回す命令、PID保持、原点登録は行いません。
RobStride 05 に mechPos (0x7019) を問い合わせ、返ってきた実測角度だけを
ターミナルに表示します。

実行中は CAN-USB 変換器をこのプログラムだけで使ってください。
ゲームプログラム、mecanum.py、game3.py などを同時に起動してはいけません。
終了は Ctrl+C です。
"""

from __future__ import annotations

import time
from math import degrees

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

    # 起動時の実測角度を控える。relative_deg はここからの変化量です。
    start_rad: dict[str, float] = {}
    current_rad: dict[str, float] = {}
    raw_frames: dict[str, bytes] = {}
    next_display_at = 0.0

    print("=" * 56)
    print("  catch / lift の実測角度を表示します（読み取り専用）")
    print("  Ctrl+C で終了")
    print("  mechPos(10進数): RobStride内部の実測位置。radとdegで表示します")
    print("  phase_deg: 1回転内へ丸めた角度（電源を入れ直しても原点保存に使える値）")
    print("  relative_deg: このプログラムを起動した位置からの角度差")
    print("  AT応答: CAN-USB変換器から届いた生の17バイトフレーム（16進数）")
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
                current_rad[feedback.name] = feedback.position_rad
                start_rad.setdefault(feedback.name, feedback.position_rad)
                raw_frames[feedback.name] = feedback.raw_at_frame

            if now >= next_display_at:
                lines = []
                for name in ("catch", "lift"):
                    if name not in current_rad:
                        lines.append(f"{name}: 応答待ち")
                        continue
                    raw_deg = degrees(current_rad[name])
                    phase_deg = (raw_deg + 180.0) % 360.0 - 180.0
                    relative_deg = degrees(current_rad[name] - start_rad[name])
                    lines.append(
                        f"{name}: mechPos(10進数)={current_rad[name]:+10.5f} rad "
                        f"/ {raw_deg:+9.2f} deg\n"
                        f"  phase={phase_deg:+8.2f} deg  "
                        f"開始位置から {relative_deg:+8.2f} deg\n"
                        f"  AT応答: {raw_frames[name].hex(' ')}\n"
                        f"  mechPosデータ: {raw_frames[name][7:15].hex(' ')}"
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
