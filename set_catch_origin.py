#!/usr/bin/env python3
"""catchだけを機械ストッパーへ合わせ、0度として保存する。

実行: ``python3 set_catch_origin.py``

liftは動かさず、既に保存されているlift原点をそのまま残す。
catchの動く向き・速度・停止判定は hensuu.py の ``catch_homing_*`` を使う。
"""

from __future__ import annotations

import hensuu
from servos import open_servos


def main() -> None:
    servos = open_servos()
    try:
        # save_origins() はcatch/lift両方の原点を保存するため、先に既存の
        # lift原点を読み込む。ファイルが無い場合にliftの原点を勝手に作らない。
        servos.load_origins(hensuu.servo_origin_file)
        print("=" * 56)
        print("catchだけをストッパーへ動かして、0度として保存します")
        print("liftは動かしません。機構の周囲が安全なことを確認してください。")
        print("=" * 56)
        input("安全を確認したら Enter: ")

        servos.attach()
        servos.home_to_stop(
            "catch",
            speed_percent=hensuu.catch_homing_speed_percent,
            direction=hensuu.catch_homing_direction,
            stillness_deg=hensuu.catch_homing_stillness_deg,
            stillness_sec=hensuu.catch_homing_stillness_sec,
            timeout_sec=hensuu.catch_homing_timeout_sec,
        )
        servos.save_origins(hensuu.servo_origin_file)
        print(f"保存完了: {hensuu.servo_origin_file}")
        print("以後、catch.write(0) がこのストッパー位置になります。")
    finally:
        servos.close()


if __name__ == "__main__":
    main()
