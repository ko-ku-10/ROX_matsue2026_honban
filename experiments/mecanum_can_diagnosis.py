#!/usr/bin/env python3
"""足回りだけのCAN送信確認。

DualSense・lift/catch・カメラ・GPIOは一切使いません。
公式サンプルと同じ AT Type 3 (enable) と Type 1 (motion control) を、
hensuu.py の ``serial_port`` へ直接送ります。

必ずタイヤを浮かせてから実行してください。
"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum.serial_at import (
    ATMotor,
    AT_NEUTRAL_VALUE,
    PySerialTransport,
    build_mecanum_motion_control_value_frame,
)
from rox_mecanum.feedback_servo import ATEncoderReader


# ここだけ書き換えると、診断時の回転速度を変えられます。
TEST_SPEED_PERCENT = 10.0
# 1輪ずつ回す時間です。短く安全な値にしています。
TEST_SECONDS = 0.5

# 現在の足回りのATアドレス。CAN ID 1〜4ではなく、既存の実機設定値です。
WHEELS = {
    "FL（前左）": 0x0C,
    "FR（前右）": 0x14,
    "RL（後左）": 0x1C,
    "RR（後右）": 0x24,
}


def main() -> None:
    speed_span = int(AT_NEUTRAL_VALUE * TEST_SPEED_PERCENT / 100.0)
    print("=" * 62)
    print("足回り CAN 診断: DualSense・機構・GPIOは動かしません")
    print(f"送信先: {hensuu.serial_port} @ {hensuu.serial_baud}")
    print(f"速度: {TEST_SPEED_PERCENT:.1f}% / 1輪ずつ {TEST_SECONDS:.1f}秒")
    print("必ずタイヤを浮かせ、周囲に人・物がないことを確認してください。")
    print("=" * 62)

    input("安全を確認したら Enter: ")
    transport = PySerialTransport.open(
        hensuu.serial_port,
        hensuu.serial_baud,
        minimum_interval=0.015,  # 公式サンプルと同じ全送信間隔
    )
    motors = {
        name: ATMotor(
            transport,
            address,
            speed_span=speed_span,
            zero_hold_band=0.0,
            velocity_value_frame_builder=build_mecanum_motion_control_value_frame,
        )
        for name, address in WHEELS.items()
    }
    reader = ATEncoderReader(transport, WHEELS)

    try:
        # まず動かさず、4輪のmechPos応答だけを確認する。ここで応答がない
        # モーターへ速度指令を送っても動かないため、ID・配線を先に切り分ける。
        print("\n1) 4輪のmechPos応答を確認します（この段階では回りません）")
        received: dict[str, float] = {}
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and len(received) < len(WHEELS):
            reader.request_next()
            time.sleep(0.015)
            for feedback in reader.poll():
                if feedback.position_rad is not None:
                    received[feedback.name] = feedback.position_rad
            time.sleep(0.015)

        for name in WHEELS:
            if name in received:
                print(f"  {name}: mechPos={received[name]:+.5f} rad")
            else:
                print(f"  {name}: 応答なし")
        if len(received) != len(WHEELS):
            print("\n停止: 応答なしの車輪があります。CAN ID・電源・CAN配線を確認してください。")
            print("この診断は速度指令を送らずに終了します。")
            return

        # 公式サンプルと同じenableを3回送る。ここではまだ回りません。
        print("\n2) enableフレームを全輪へ送信中...")
        for _ in range(3):
            for motor in motors.values():
                motor.enable()
            time.sleep(0.05)
        for motor in motors.values():
            motor.stop()

        for name, motor in motors.items():
            input(f"\n{name}だけを +{TEST_SPEED_PERCENT:.1f}% で回します。Enter: ")
            motor.set_velocity(1.0, force=True)
            time.sleep(TEST_SECONDS)
            motor.stop()
            print(f"{name}: 停止指令を送信しました")

        print("\n完了: 4輪すべてへ公式形式のenable/速度/停止を送信しました。")
    finally:
        # Ctrl+Cやエラー時も、可能な限り停止フレームを送る。
        for motor in motors.values():
            try:
                motor.stop()
            except Exception:
                pass
        transport.close()


if __name__ == "__main__":
    main()
