"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# メカナム
mecanum_speed_percent = 30.0       # 最高速度（0〜100）
mecanum_rotation_requires_r2 = True

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = -30.0
catch_max_angle = 90.0
catch_speed_percent = 15.0         # 90度を測定したときと同じ速度
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1                # 角度を増やして逆へ動く場合は -1

# ソレノイド
solenoid_pin = 17
solenoid_time_sec = 0.3
