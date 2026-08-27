#!/usr/bin/env python3
"""catch/liftの物理0度を一度だけ登録するプログラム。"""

import hensuu
from servos import open_servos


print("=== catch/lift 原点登録 ===")
print("1. catch と lift を、決めた物理0度位置へ手で安全に合わせてください。")
print("   通常は各機構のストッパー位置を0度にします。")
print("2. 機構に手や物を挟まないことを確認してから Enter を押してください。")
input("準備できたら Enter: ")

servos = open_servos()
try:
    # 原点登録はmechPosを読むだけなので、enable/stopのCANフレームを送らない。
    # これにより、角度監視では読めるのにattach()直後だけ応答が消える状態を避ける。
    servos.save_origins(hensuu.servo_origin_file)
    print(f"原点を保存しました: {hensuu.servo_origin_file}")
    print("次回から GAME1 / GAME2 / GAME3 はストッパー原点合わせをせずに起動します。")
finally:
    servos.close()
