#!/usr/bin/env python3
"""EDULITE05の現在位置を、モーター本体へ機械0度として保存する一回用ツール。

使い方:
    python3 save_motor_zero.py lift
    python3 save_motor_zero.py catch

Type 6 (機械0度設定) の直後に Type 22 (本体フラッシュ保存) を1回だけ送る。
速度指令・PID・GPIOは一切送らない。通常のGAME起動では実行しないこと。
"""

from __future__ import annotations

import sys
import time
from math import degrees

import hensuu
from rox_mecanum import (
    ATEncoderReader,
    PySerialTransport,
    at_address_from_can_id,
    build_save_motor_data_command,
    build_set_mechanical_zero_command,
)


# 引数なしで実行した場合の対象。最初はliftだけで確認する。
DEFAULT_MOTOR = "lift"
MOTOR_IDS = {
    "catch": hensuu.catch_can_id,
    "lift": hensuu.lift_can_id,
}


def read_mech_pos(reader: ATEncoderReader, name: str, timeout_sec: float = 1.0) -> float:
    """指定モーターの正式mechPosを1つだけ安全に読む。"""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        reader.request(name)
        time.sleep(hensuu.encoder_response_wait_sec)
        for feedback in reader.poll():
            if feedback.name == name and feedback.position_rad is not None:
                return feedback.position_rad
        time.sleep(0.02)
    raise TimeoutError(f"{name} のmechPos応答がありません。保存は行いません。")


def main() -> None:
    name = sys.argv[1].lower() if len(sys.argv) >= 2 else DEFAULT_MOTOR
    if name not in MOTOR_IDS:
        names = ", ".join(MOTOR_IDS)
        raise SystemExit(f"対象は {names} のどちらかです。例: python3 save_motor_zero.py lift")

    address = at_address_from_can_id(MOTOR_IDS[name])
    transport = PySerialTransport.open(
        hensuu.serial_port,
        hensuu.serial_baud,
        minimum_interval=0.0008,
    )
    reader = ATEncoderReader(transport, {name: address})

    try:
        print("=" * 60)
        print(f"  {name} (CAN ID {MOTOR_IDS[name]}) の機械0度を本体へ保存します")
        print("  Type 6 → Type 22 だけを送信します。モーターは動かしません。")
        print("=" * 60)
        print("GAME・PID・MotorStudioを終了し、機構を0度にしたい物理位置へ合わせてください。")

        before = read_mech_pos(reader, name)
        print(f"保存前 mechPos: {before:+.5f} rad ({degrees(before):+.2f} deg)")
        answer = input("位置と周囲が安全なら、半角で SAVE と入力してEnter: ").strip()
        if answer != "SAVE":
            print("中止しました。本体には何も保存していません。")
            return

        # 公式private protocol: Type 6=機械0度設定、Type 22=本体フラッシュ保存。
        transport.write(build_set_mechanical_zero_command(address))
        time.sleep(0.10)
        transport.write(build_save_motor_data_command(address))
        time.sleep(0.25)

        reader.discard_pending()
        after = read_mech_pos(reader, name)
        print(f"保存直後 mechPos: {after:+.5f} rad ({degrees(after):+.2f} deg)")
        print("完了。次はモーター電源をOFF→ONして、angle_monitor.pyで0度付近か確認してください。")
        print("電源再投入後も0度なら、本体保存に成功です。")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
