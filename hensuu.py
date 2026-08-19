"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# メカナム
mecanum_speed_percent = 100       # 最高速度（0〜100）
mecanum_rotation_requires_r2 = True

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = -30.0
catch_max_angle = 90.0
# 90度を15%で測定した値を基準に、実際は7.5%でゆっくり動かす。
catch_calibration_speed_percent = 15.0
catch_move_speed_percent = 7.5
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1                # 角度を増やして逆へ動く場合は -1
catch_brake_time_sec = 0.08        # 停止前に減速する時間。反動が残るなら増やす

# lift: CAN ID 6、時間式サーボ
# 90度の時間はliftで測定してから入力する。0.0のままでは安全のため動かない。
lift_can_id = 6
lift_min_angle = 0.0
lift_max_angle = 90.0
lift_calibration_speed_percent = 15.0
lift_move_speed_percent = 7.5
lift_90deg_time_sec = 0.124
lift_direction = 1
lift_brake_time_sec = 0.08

# ソレノイド
solenoid_pin = 17
solenoid_time_sec = 0.3
