#後から変更する可能性のあるものをここでまとめています

#シリンダ
sore_pin = 17 #ソレノイドのピン番号
sore_time = 0.3 #ソレノイドのオンにする時間


# ============================================================
# メカナム（ここだけを調整すればOK）
# ============================================================

# モーターとのUSBシリアル接続
mecanum_serial_port = "/dev/ttyUSB1"
mecanum_serial_baud = 921600

# 最高速度。0〜100 の範囲で設定する（最初は低めの30がおすすめ）
mecanum_speed_percent = 30.0

# 操作感
mecanum_deadzone = 0.08             # スティック中心の無視範囲（0.00〜0.99）
mecanum_translation_gain = 1.00     # 前後・平行移動の効き（0.0〜1.0）
mecanum_rotation_gain = 1.00        # 旋回の効き（0.0〜1.0）
mecanum_response_exponent = 1.00    # 1.0: リニア、2.0: 中心付近をゆっくり

# 旋回はR2を押している間だけにするか。Falseなら右スティックだけで旋回する。
mecanum_rotation_requires_r2 = True

# 起動直後の安全停止時間と制御周期
mecanum_startup_stop_sec = 0.8
mecanum_control_hz = 20

# モーター有効化の再送設定（通常は変更不要）
mecanum_enable_retries = 3
mecanum_enable_interval_sec = 0.05
mecanum_serial_write_interval_sec = 0.0008

# メカナムの寸法。旋回の出力比に使う（元プログラムと同じ値）
mecanum_wheel_base_half_l = 0.12
mecanum_wheel_base_half_w = 0.10

# 実機の配線・取付方向。通常は変更不要。
mecanum_motor_ids = {"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24}
mecanum_motor_directions = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}


# ============================================================
# 新機構用サーボ（CAN ID 5）
# ============================================================
# 位置制御を使う前に、実機の可動範囲を必ず測定して設定する。
mechanism_motor_id = 0x05
mechanism_min_position_deg = -30.0
mechanism_max_position_deg = 90.0
# 速度上限。初回は10〜15%程度で、機構を浮かせて確認する。
mechanism_speed_percent = 15.0
mechanism_max_command = mechanism_speed_percent / 100.0

# 位置PID。最初はKi=Kd=0で始め、必要な場合だけ少しずつ上げる。
mechanism_pid_kp = 0.015
mechanism_pid_ki = 0.000
mechanism_pid_kd = 0.000
mechanism_pid_integral_limit = 100.0
mechanism_position_kp = mechanism_pid_kp  # 互換用の別名
mechanism_command_accel_per_sec = 0.8
mechanism_tolerance_deg = 1.0
mechanism_encoder_direction = 1     # 目標角度と逆に動く場合は -1
mechanism_can_channel = "can0"      # MKS CANableのSocketCAN名
mechanism_can_id = 0x05
