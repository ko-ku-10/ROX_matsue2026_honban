#後から変更する可能性のあるものをここでまとめています

#シリンダ
sore_pin = 17 #ソレノイドのピン番号
sore_time = 0.3 #ソレノイドのオンにする時間


# ============================================================
# メカナム（ここだけを調整すればOK）
# ============================================================

# モーターとのUSBシリアル接続
mecanum_serial_port = "/dev/ttyUSB0"
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
# 新機構用サーボ 1：catch（CAN ID 5）
# ============================================================
# 原点はプログラム中で set_home() を呼んだ位置。可動範囲は原点からの角度で設定する。
catch_motor_id = 0x05
catch_can_id = catch_motor_id
catch_min_position_deg = -30.0
catch_max_position_deg = 90.0

# 速度上限。初回は10〜15%程度で、機構を浮かせて確認する。
catch_speed_percent = 15.0
catch_max_command = catch_speed_percent / 100.0

# 位置PID。最初は Ki=Kd=0 のまま Kp だけを少しずつ調整する。
catch_pid_kp = 0.015
catch_pid_ki = 0.000
catch_pid_kd = 0.000
catch_pid_integral_limit = 100.0
catch_command_accel_per_sec = 0.8
catch_tolerance_deg = 1.0
catch_encoder_direction = 1     # 目標と逆に動く場合は -1


# ============================================================
# 新機構用サーボ 2：昇降など（CAN ID 6）
# ============================================================
# ID 5とは独立して、可動範囲・速度・PIDを設定できる。
lift_motor_id = 0x06
lift_can_id = lift_motor_id
lift_min_position_deg = 0.0
lift_max_position_deg = 200.0

lift_speed_percent = 15.0
lift_max_command = lift_speed_percent / 100.0

lift_pid_kp = 0.015
lift_pid_ki = 0.000
lift_pid_kd = 0.000
lift_pid_integral_limit = 100.0
lift_command_accel_per_sec = 0.8
lift_tolerance_deg = 1.0
lift_encoder_direction = 1      # 目標と逆に動く場合は -1


# MKS CANableのSocketCAN名。CAN ID 5/6の両方で共通に使う。
mechanism_can_channel = "can1"      # MKS CANableをslcandで作成したSocketCAN名
mechanism_host_id = 0xFF       # PC側のCANホストID。モーターIDより大きい0xFFを使う。

