# ROX 本番プログラム

実戦で使うファイルは次の4つです。

- `mecanum.py` — メカナム操作
- `catch_servo.py` — catch の時間式サーボ例
- `sorenoido.py` — L2でソレノイドを動作
- `hensuu.py` — 実機で調整する値

## 起動

```bash
python3 mecanum.py
python3 catch_servo.py
python3 sorenoido.py
```

`mecanum.py` と `catch_servo.py` は同じUSBシリアルを使うため、同時には起動しません。

## catch の使い方

```python
from catch_servo import open_catch

servo, transport = open_catch()
try:
    servo.attach()
    servo.home(0)      # 実機を0度に合わせてから呼ぶ
    servo.write(90)    # 90度へ動かす
    servo.write(0)     # 0度へ戻す
finally:
    servo.detach()
    transport.close()
```

時間式のため、電源を入れ直した後は必ず `home(0)` を呼びます。
