# ROX 本番プログラム

実戦で使うファイルは次の4つです。

- `mecanum.py` — メカナム操作
- `servos.py` — catch・lift の時間式サーボ
- `sorenoido.py` — L2でソレノイドを動作
- `hensuu.py` — 実機で調整する値

## 起動

```bash
python3 mecanum.py
python3 sorenoido.py
```

`mecanum.py` と `servos.py` は同じUSBシリアルを使うため、同時には起動しません。

## catch の使い方

```python
from servos import open_servos

servos = open_servos()
try:
    servos.attach()
    servos.home()          # 実機を両方とも0度に合わせてから呼ぶ
    servos.catch.write(90) # catchを90度へ
    servos.lift.write(45)  # liftを45度へ
finally:
    servos.close()
```

時間式のため、電源を入れ直した後は必ず `home(0)` を呼びます。
