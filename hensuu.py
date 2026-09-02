"""実機で調整する値だけをここにまとめる。"""

# メカナム・catch モーター共通のUSBシリアル接続
# 現在RDK X5で確認できたUSB-CANの接続先。
# USBを抜き差しして番号が変わった時だけ、``ls -l /dev/ttyUSB*`` で確認して直す。
serial_port = "/dev/ttyUSB0"
serial_baud = 921600

# catch/liftの物理0度を一度だけ保存するファイル。
# EDULITE05は電源投入でmechPosが360度単位にずれるため、保存・比較時には
# 1回転内の角度へ自動的に丸める。通常のGAME起動ではこの値を使うため、
# ストッパーへ毎回押し付けない。
# 原点を作り直す時だけ ``python3 set_servo_origins.py`` を実行する。
servo_origin_file = "servo_origins.json"

# DualSenseの接続モード。
# "connect_each_run": プログラム起動時に接続し、終了時にBluetoothを切断する（電池節約）。
# "keep_connected": 終了時に切断しない。続けて別のGAME/実験を実行する時に使う。
# USBケーブルで使う本番設定。Bluetoothを使う場合だけ "connect_each_run" または
# "keep_connected" に変更する。
dualsense_connection_mode = "wired"
# ペアリング済みDualSenseのMACアドレス。違うコントローラーを使う時だけ変更する。
dualsense_mac_address = "0C:27:56:31:25:90"
dualsense_connect_timeout_sec = 30.0

# メカナム
mecanum_speed_percent = 100       # 最高速度（0〜100）
# スティックを少し倒した時に勝手に動かない範囲。大きいほどブレに強い。
mecanum_deadzone = 0.08
# 1.0ならスティック量と速度が直線的。1.6なら中央付近を細かく操作でき、
# 最後まで倒した時の最高速度は100%のまま。
mecanum_response_exponent = 1.6
# 急発進を抑える加速の速さ。300なら停止→100%まで約0.33秒。
# 小さいほどゆっくり、大きいほどキビキビ加速する。
mecanum_acceleration_percent_per_sec = 300.0
# スティックを戻した時の減速の速さ。加速より大きくし、止まりたい時は早く止まる。
# 800なら100%→停止まで約0.13秒。急すぎるなら小さくする。
mecanum_deceleration_percent_per_sec = 800.0
# 足回りのガタガタ対策。公式サンプルと同じ値。
# スティック値が細かく変わっても、各輪はこの秒数より短い間隔では再送しない。
mecanum_command_minimum_interval_sec = 0.05
# この値以上の大きな指令変化だけは、上の間隔を待たず即時送信する。
mecanum_command_force_delta = 0.20
# AT速度値の細かい揺れを無視する幅。大きくすると静かになるが、微速操作は鈍る。
mecanum_command_hysteresis_counts = 220
# 前後反転する時、これ以下の逆向き値はいったん停止へ吸着する。
mecanum_command_reverse_guard_counts = 420
# 左スティックを上へ倒した時に前進するよう、前後入力だけを反転する。
# 前後が再び逆なら True / False を切り替える。
mecanum_invert_forward_input = True
# Falseなら右スティック左右だけで旋回する。L2/R2を押す必要はない。
mecanum_rotation_requires_r2 = False
# 横移動だけの最高速度（0〜100%）。前後・旋回の速度には影響しない。
# 後ろが重くて横移動中に曲がる時は、まず50%程度から試す。
mecanum_strafe_speed_percent = 50.0

# catch: CAN ID 5、時間式サーボ
catch_can_id = 5
catch_min_angle = -360
catch_max_angle = 360
# 90度を15%で測定した値を基準に、実際は7.5%でゆっくり動かす。
catch_calibration_speed_percent = 7.5
catch_move_speed_percent = 10
catch_90deg_time_sec = 0.124       # 実測値。90度に掛かった時間
catch_direction = 1         # 角度を増やして逆へ動く場合は -1
catch_brake_time_sec = 0.08        # 停止前に減速する時間。反動が残るなら増やす

# lift: CAN ID 6、時間式サーボ
# 90度の時間はliftで測定してから入力する。0.0のままでは安全のため動かない。
lift_can_id = 6
lift_min_angle = -360
lift_max_angle = 360
lift_calibration_speed_percent = 15.0
lift_move_speed_percent = 10
lift_90deg_time_sec = 0.124
lift_direction = -1
lift_brake_time_sec = 0.08

# 起動時のcatch原点合わせ。開く側の機械ストッパーまで動かし、
# mechPosがほぼ変わらなくなった位置を0度にする。
# 閉じる向きへ動いてしまった場合だけ、catch_homing_direction を -1 に変える。
# 起動時は衝撃を避けるため低速でストッパーへ当てる。
catch_homing_speed_percent = 10
catch_homing_direction = -1
catch_homing_stillness_deg = 0.2
catch_homing_stillness_sec = 0.10
catch_homing_timeout_sec = 8.0

# 起動時のlift原点合わせ。地面ドリブル位置の機械ストッパーまで低速で下ろし、
# mechPosがほぼ変わらなくなった位置を0度にする。
# 上へ動いてしまった場合だけ、lift_homing_direction を -1 に変える。
lift_homing_speed_percent = 3
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
catch_pid_kp = 0.010
lift_pid_kp = 0.003
# 重力などで同じ方向へずれ続ける場合に、少しずつ保持力を増やす。
catch_pid_ki = 0.002
lift_pid_ki = 0.000
servo_pid_kd = 0.000
servo_pid_integral_limit = 30.0
# 保持の初回確認は低速から。安定後に必要なら少しずつ上げる。
# 1周期で1台ずつ読む。USB-AT変換器の応答を安定させるため、100Hzなら
# catch/liftはそれぞれ50Hzで実測角度を更新する。
# ±0.2°を超えた直後から低速で戻す。力を受けた直後の位置ずれを小さくする。
servo_max_speed_percent = 8.0
# mechanism_manual実験で目標角度へ動かす時だけ使う上限。保持用の3%とは分ける。
mechanism_move_speed_percent = 10.0
servo_tolerance_deg = 0.2
encoder_poll_hz = 100.0
# mechPos要求を送ってから受信するまでの待機時間。angle_monitor.pyと同じ15ms。
# 短くすると応答を取りこぼしやすく、長くするとPIDの更新回数が減る。
encoder_response_wait_sec = 0.015
# この時間mechPos応答が来なければ保持出力を停止する（角度の推定はしない）。
servo_feedback_timeout_sec = 0.25

# 状態表示サイト: http://ロボットのIPアドレス:8000
dashboard_port = 8000
# 状態表示サイトの映像更新回数。小さいほど走行操作への負荷が減る。
dashboard_camera_hz = 3.0
# 自動中にAprilTagを読み取る回数。10Hzなら0.1秒ごとに位置を更新する。
dashboard_tag_hz = 10.0
