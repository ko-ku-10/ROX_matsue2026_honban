"""GAME3・操作練習専用の実機調整値。"""

# △で開始するGAME3連続動作の角度。
# 各段階はエンコーダー到達確認後に次へ進む。実機に合わせて調整する。
sequence_lift_first_angle = 110.0
sequence_catch_grab_angle = -70.0
sequence_lift_after_grab_angle = 20.0
sequence_catch_release_angle = 0.0
lift_fire_angle = 110.0

# lift/catchが発射姿勢へ到達するまでの最大待機時間[秒]。
mechanism_target_timeout_sec = 8.0

# 各角度へ到達してから、反動が収まるまで待つ時間[秒]。
# まだ速すぎる・揺れる場合は 0.8、1.0 の順に増やす。
mechanism_settle_sec = 0.5

# スティック中立の微小なズレでは走らない範囲（0.0〜1.0）。
# 勝手に動く場合は 0.20、0.25 の順に少しずつ大きくする。
manual_stick_deadzone = 0.18
