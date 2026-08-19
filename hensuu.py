"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# メカナム
mecanum_speed_percent = 100       # 最高速度（0〜100）
mecanum_rotation_requires_r2 = True

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = 0
catch_max_angle = 30
# 90度を15%で測定した値を基準に、実際は7.5%でゆっくり動かす。
catch_calibration_speed_percent = 7.5
catch_move_speed_percent = 7.5
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1                # 角度を増やして逆へ動く場合は -1
catch_brake_time_sec = 0.08        # 停止前に減速する時間。反動が残るなら増やす

# lift: CAN ID 6、時間式サーボ
# 90度の時間はliftで測定してから入力する。0.0のままでは安全のため動かない。
lift_can_id = 6
lift_min_angle = -100.0
lift_max_angle = 0
lift_calibration_speed_percent = 15.0
lift_move_speed_percent = 7.5
lift_90deg_time_sec = 0.124
lift_direction = 1
lift_brake_time_sec = 0.08

# エンコーダーPID位置サーボ（外力に対して位置を保持する設定）
# 直結なら 65536 / 360。ギヤがある場合は実機角度に合わせて増減する。
catch_counts_per_degree = 65536.0 / 360.0
lift_counts_per_degree = 65536.0 / 360.0
catch_pid_kp = 0.015
lift_pid_kp = 0.015
# 重力などで同じ方向へずれ続ける場合に、少しずつ保持力を増やす。
catch_pid_ki = 0.002
lift_pid_ki = 0.002
servo_pid_kd = 0.000
servo_pid_integral_limit = 30.0
servo_max_speed_percent = 20.0
servo_tolerance_deg = 0.1
encoder_poll_hz = 50.0

# 状態表示サイト: http://ロボットのIPアドレス:8000
dashboard_port = 8000

# ソレノイド
solenoid_pin = 17
solenoid_time_sec = 0.3
