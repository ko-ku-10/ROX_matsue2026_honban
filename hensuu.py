"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# DualSenseはプログラム起動時だけBluetooth接続し、終了時に切断する。
# ペアリング済みDualSenseのMACアドレス。違うコントローラーを使う時だけ変更する。
dualsense_mac_address = "0C:27:56:31:25:90"
dualsense_connect_timeout_sec = 30.0
dualsense_disconnect_on_close = True

# メカナム
mecanum_speed_percent = 100       # 最高速度（0〜100）
# 急発進を抑える加速の速さ。300なら停止→100%まで約0.33秒。
# 小さいほどゆっくり、大きいほどキビキビ加速する。
mecanum_acceleration_percent_per_sec = 300.0
# 左スティックを上へ倒した時に前進するよう、前後入力だけを反転する。
# 前後が再び逆なら True / False を切り替える。
mecanum_invert_forward_input = True
# Falseなら右スティック左右だけで旋回する。L2/R2を押す必要はない。
mecanum_rotation_requires_r2 = False

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = -360
catch_max_angle = 360
# 90度を15%で測定した値を基準に、実際は7.5%でゆっくり動かす。
catch_calibration_speed_percent = 7.5
catch_move_speed_percent = 10
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1                # 角度を増やして逆へ動く場合は -1
catch_brake_time_sec = 0.08        # 停止前に減速する時間。反動が残るなら増やす

# lift: CAN ID 6、時間式サーボ
# 90度の時間はliftで測定してから入力する。0.0のままでは安全のため動かない。
lift_can_id = 6
lift_min_angle = -360
lift_max_angle = 360
lift_calibration_speed_percent = 15.0
lift_move_speed_percent = 10
lift_90deg_time_sec = 0.124
lift_direction = 1
lift_brake_time_sec = 0.08

# 起動時のcatch原点合わせ。開く側の機械ストッパーまで動かし、
# mechPosがほぼ変わらなくなった位置を0度にする。
# 閉じる向きへ動いてしまった場合だけ、catch_homing_direction を -1 に変える。
catch_home_to_stop_enabled = True
catch_homing_speed_percent = 15.0
catch_homing_direction = 1
catch_homing_stillness_deg = 0.2
catch_homing_stillness_sec = 0.10
catch_homing_timeout_sec = 3.0

# 起動時のlift原点合わせ。地面ドリブル位置の機械ストッパーまで低速で下ろし、
# mechPosがほぼ変わらなくなった位置を0度にする。
# 上へ動いてしまった場合だけ、lift_homing_direction を -1 に変える。
lift_home_to_stop_enabled = True
lift_homing_speed_percent = 12.0
lift_homing_direction = 1
lift_homing_stillness_deg = 0.2
lift_homing_stillness_sec = 0.10
lift_homing_timeout_sec = 3.0

# ボールを扱う時の共通角度（全GAME共通）
# ボールを持って走る間は、必ず hold と ground の姿勢にする。
# 3つのcatch角度は実機でゆっくり動かして測定してから入力する。
# hold: 地面に付けたまま、ボールを落とさず保持する角度
# grab: ボールを掴み始める角度
# release: ボールをRobotの外へ出す（発射前など）角度
catch_ball_hold_angle = 0.0
catch_ball_grab_angle = 0.0
catch_ball_release_angle = 0.0

# ボールを持って移動する時のlift角度。地面に付く高さに設定する。
lift_ball_ground_angle = 0.0

# experiments.lift_test 専用。△でこの角度へ動かす。
# 初回は小さい角度で確認する。逆へ動いたら符号を反転する。
lift_test_up_angle = -10.0

# エンコーダーPID位置サーボ（外力に対して位置を保持する設定）
# 直結なら 65536 / 360。ギヤがある場合は実機角度に合わせて増減する。
catch_counts_per_degree = 65536.0 / 360.0
lift_counts_per_degree = 65536.0 / 360.0
# 保持の反応。停止範囲を小さくしたため、弱すぎるP値では戻りが遅くなる。
catch_pid_kp = 0.020
lift_pid_kp = 0.003
# 重力などで同じ方向へずれ続ける場合に、少しずつ保持力を増やす。
catch_pid_ki = 0.002
lift_pid_ki = 0.000
servo_pid_kd = 0.000
servo_pid_integral_limit = 30.0
# 保持の初回確認は低速から。安定後に必要なら少しずつ上げる。
# 1周期で1台ずつ読むため、200Hzなら各モーターは100Hzで実測角度を更新する。
# ±0.2°を超えた直後から低速で戻す。力を受けた直後の位置ずれを小さくする。
servo_max_speed_percent = 8.0
# mechanism_manual実験で目標角度へ動かす時だけ使う上限。保持用の3%とは分ける。
mechanism_move_speed_percent = 10.0
servo_tolerance_deg = 0.2
encoder_poll_hz = 200.0
# この時間mechPos応答が来なければ保持出力を停止する（角度の推定はしない）。
servo_feedback_timeout_sec = 0.25

# 状態表示サイト: http://ロボットのIPアドレス:8000
dashboard_port = 8000
