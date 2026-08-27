#!/usr/bin/env python3
"""liftだけのCAN通信を段階的に確認する。

メカナム、catch、PID、GPIO、DualSenseは一切開かない。
CAN通信が他のプログラムから使われていない状態で実行する。

実行:
    python3 -m experiments.lift_can_diagnosis
"""

from __future__ import annotations

import time
from math import degrees

import hensuu
from rox_mecanum import ATEncoderReader, ATMotor, PySerialTransport, at_address_from_can_id


# 安全のため、診断時の速度と回す時間は小さくしている。
# 逆向きに回したい時だけ TEST_SPEED を -0.05 に変える。
TEST_SPEED = 0.05
TEST_MOVE_SEC = 0.7
READ_SEC = 1.0


def watch_mech_pos(reader: ATEncoderReader, seconds: float, title: str) -> tuple[float | None, float | None]:
    """指定時間liftのmechPosを読み、最初と最後の値を返す。"""
    print(f"\n--- {title} ({seconds:.1f}秒) ---")
    first: float | None = None
    last: float | None = None
    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        # angle_monitor.py と同じ: 要求して15ms待ってから受信する。
        reader.request("lift")
        time.sleep(0.015)
        for feedback in reader.poll():
            if feedback.name != "lift" or feedback.position_rad is None:
                continue
            first = feedback.position_rad if first is None else first
            last = feedback.position_rad
            moved = degrees(last - first)
            print(f"lift mechPos={last:+.5f} rad  この区間の変化={moved:+.2f}度")
        time.sleep(0.015)

    if last is None:
        print("結果: mechPos応答なし")
    else:
        print(f"結果: mechPos応答あり / 区間の変化={degrees(last - first):+.2f}度")
    return first, last


def main() -> None:
    transport = PySerialTransport.open(
        hensuu.serial_port,
        hensuu.serial_baud,
        minimum_interval=0.0008,
    )
    lift_address = at_address_from_can_id(hensuu.lift_can_id)
    reader = ATEncoderReader(transport, {"lift": lift_address})
    # PID用の停止帯を使わず、ここで指定した5%をそのまま送る。
    lift = ATMotor(transport, lift_address, zero_hold_band=0.0)

    print("=" * 56)
    print(" lift CAN診断: 読取り → 有効化/停止 → 低速回転 を確認します")
    print(" メカナム・catch・PID・GPIOは動かしません。Ctrl+Cなら直ちに停止します。")
    print("=" * 56)

    try:
        # 1. モーターへ駆動命令を送らない状態で、正式mechPosを確認する。
        watch_mech_pos(reader, READ_SEC, "1) 読取りのみ")

        input("\n機構の周囲が安全なら Enter: liftを有効化して停止指令を送ります: ")
        # 2. 通常起動と同じ有効化・停止をした後でも、値が読めるか確認する。
        for _ in range(3):
            lift.enable()
            time.sleep(0.05)
        lift.stop()
        watch_mech_pos(reader, READ_SEC, "2) 有効化して停止した後の読取り")

        input(
            f"\n安全なら Enter: liftを {TEST_SPEED * 100:+.1f}% で "
            f"{TEST_MOVE_SEC:.1f}秒だけ動かします: "
        )
        # 3. 低速の速度指令を1回だけ送り、その間に角度が変わるかを見る。
        lift.set_velocity(TEST_SPEED, force=True)
        watch_mech_pos(reader, TEST_MOVE_SEC, "3) 低速回転中の読取り")
        lift.stop()
        watch_mech_pos(reader, 0.4, "4) 停止後の読取り")
        print("\n診断完了: 表示結果をそのまま共有してください。")
    except KeyboardInterrupt:
        print("\nCtrl+C: liftを停止します")
    finally:
        # 例外・Ctrl+C・通常終了の全てで、liftの速度を必ず0にする。
        try:
            lift.stop()
        finally:
            transport.close()


if __name__ == "__main__":
    main()
