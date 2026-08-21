"""GAME1専用の実機調整値。

数値は安全な初期値。実機で低速から確認して変更する。
"""

# CREATEで展開する開始姿勢。角度の原点は起動時に合わせた位置。
game1_catch_start_angle = 0.0
game1_lift_start_angle = 0.0

# 自動移動の最大速度（0.0〜1.0）。
auto_speed = 0.20
center_gain = 0.45
center_tolerance = 0.08
target_distance_m = 0.80
distance_tolerance_m = 0.08

# Tag 1/9の反対を向く時間と、Tag 8を探す横移動。現地で調整する。
turn_around_speed = 0.20
turn_around_sec = 0.0
slide_speed = 0.20
slide_sec = 0.0
side_a_slide_sign = 1.0

# Tag 8通過、板への押し込み、6/10通過の時間式区間。
tunnel_speed = 0.20
tunnel_sec = 0.0
board_push_speed = 0.15
board_push_sec = 0.0
return_through_speed = 0.20
return_through_sec = 0.0

# Tagごとの目標距離。測定後に変更する。
tag8_distance_m = target_distance_m
tag12_13_distance_m = target_distance_m
tag6_10_distance_m = target_distance_m
tag0_distance_m = target_distance_m
