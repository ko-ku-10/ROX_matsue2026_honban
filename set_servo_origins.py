#!/usr/bin/env python3
"""catch/liftの0度を一度だけ作り、次回以降のGAME起動で使う。

通常のGAME1〜3では実行しない。物理的な0度を作り直す時だけ実行する。
"""

import hensuu
from servos import open_servos


servos = open_servos()

try:
    print("catch/liftの周囲に手・ボール・工具が無いことを確認してください。")
    input("Enterでストッパー原点合わせを開始します: ")

    servos.attach()
    servos.home_to_stop(
        "catch",
        speed_percent=hensuu.catch_homing_speed_percent,
        direction=hensuu.catch_homing_direction,
        stillness_deg=hensuu.catch_homing_stillness_deg,
        stillness_sec=hensuu.catch_homing_stillness_sec,
        timeout_sec=hensuu.catch_homing_timeout_sec,
    )
    servos.home_to_stop(
        "lift",
        speed_percent=hensuu.lift_homing_speed_percent,
        direction=hensuu.lift_homing_direction,
        stillness_deg=hensuu.lift_homing_stillness_deg,
        stillness_sec=hensuu.lift_homing_stillness_sec,
        timeout_sec=hensuu.lift_homing_timeout_sec,
    )
    servos.save_origins(hensuu.servo_origin_file)
    print(f"原点を保存しました: {hensuu.servo_origin_file}")
    print("次回からGAME1〜3は、ストッパー原点合わせをせず保存済み0度を使います。")
finally:
    servos.close()
