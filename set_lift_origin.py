#!/usr/bin/env python3
"""liftだけを下側ストッパーへ合わせ、0度として保存する。

実行: ``python3 set_lift_origin.py``

catchは動かさず、保存済みのcatch原点をそのまま残す。
liftの動く向き・速度・停止判定は hensuu.py の ``lift_homing_*`` を使う。
"""

from __future__ import annotations

import hensuu
from servos import open_servos


def main() -> None:
    servos = open_servos()
    try:
        # save_origins() はcatch/liftを一緒に保存するため、先にcatchの保存済み
        # 原点を読み込む。ここでcatchを動かすことはない。
        servos.load_origins(hensuu.servo_origin_file)
        print("=" * 56)
        print("liftだけを下側ストッパーへ動かして、0度として保存します")
        print("catchは動かしません。周囲が安全なことを確認してください。")
        print(
            f"速度: {hensuu.lift_homing_speed_percent:.1f}% / "
            f"原点合わせ方向: {hensuu.lift_homing_direction}"
        )
        print("=" * 56)
        input("安全を確認したら Enter: ")

        # catchへはenable/停止/速度を一切送らず、liftだけを有効化する。
        servos.lift.motor.stop()
        servos.lift.enable(retries=3)
        servos.lift.motor.stop()
        servos.home_to_stop(
            "lift",
            speed_percent=hensuu.lift_homing_speed_percent,
            direction=hensuu.lift_homing_direction,
            stillness_deg=hensuu.lift_homing_stillness_deg,
            stillness_sec=hensuu.lift_homing_stillness_sec,
            timeout_sec=hensuu.lift_homing_timeout_sec,
        )
        servos.save_origins(hensuu.servo_origin_file)
        print(f"保存完了: {hensuu.servo_origin_file}")
        print("以後、lift.write(0) が下側ストッパー位置になります。")
    finally:
        # ``servos.close()`` はcatchにも停止を送るため、ここでは使わない。
        try:
            servos.lift.release()
        finally:
            servos.transport.close()


if __name__ == "__main__":
    main()
