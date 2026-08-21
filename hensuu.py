"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# メカナム
mecanum_speed_percent = 100       # 最高速度（0〜100）
mecanum_rotation_requires_r2 = True

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = -360
catch_max_angle = 360
# 90度を15%で測定した値を基準に、実際は7.5%でゆっくり動かす。
catch_calibration_speed_percent = 7.5
catch_move_speed_percent = 7.5
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1                # 角度を増やして逆へ動く場合は -1
catch_brake_time_sec = 0.08        # 停止前に減速する時間。反動が残るなら増やす

# lift: CAN ID 6、時間式サーボ
# 90度の時間はliftで測定してから入力する。0.0のままでは安全のため動かない。
lift_can_id = 6
lift_min_angle = -360
lift_max_angle = 360
lift_calibration_speed_percent = 15.0
lift_move_speed_percent = 7.5
lift_90deg_time_sec = 0.124
lift_direction = 1
lift_brake_time_sec = 0.08

# エンコーダーPID位置サーボ（外力に対して位置を保持する設定）
# 直結なら 65536 / 360。ギヤがある場合は実機角度に合わせて増減する。
catch_counts_per_degree = 65536.0 / 360.0
lift_counts_per_degree = 65536.0 / 360.0
# 保持の反応。停止範囲を小さくしたため、弱すぎるP値では戻りが遅くなる。
catch_pid_kp = 0.015
lift_pid_kp = 0.003
# 重力などで同じ方向へずれ続ける場合に、少しずつ保持力を増やす。
catch_pid_ki = 0.002
lift_pid_ki = 0.000
servo_pid_kd = 0.000
servo_pid_integral_limit = 30.0
# 保持の初回確認は低速から。安定後に必要なら少しずつ上げる。
# 1周期で1台ずつ読むため、200Hzなら各モーターは100Hzで実測角度を更新する。
# ±0.2°を超えた直後から低速で戻す。力を受けた直後の位置ずれを小さくする。
servo_max_speed_percent = 3.0
servo_tolerance_deg = 0.2
encoder_poll_hz = 200.0
# この時間mechPos応答が来なければ保持出力を停止する（角度の推定はしない）。
servo_feedback_timeout_sec = 0.25

# 状態表示サイト: http://ロボットのIPアドレス:8000
dashboard_port = 8000

# ソレノイド
solenoid_pin = 17
solenoid_time_sec = 0.3
