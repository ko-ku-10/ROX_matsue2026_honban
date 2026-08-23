"""GAME3・操作練習専用の実機調整値。"""

# 発射時のlift角度。未測定の間は None のままにして、誤発射を防ぐ。
# 実機で安全に測定できた角度へ変更する。例: 35.0
lift_fire_angle = None

# lift/catchが発射姿勢へ到達するまでの最大待機時間[秒]。
mechanism_target_timeout_sec = 8.0
