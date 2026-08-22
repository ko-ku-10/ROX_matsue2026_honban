"""GAME1専用の実機調整値。

数値は安全な初期値。実機で低速から確認して変更する。
"""

# CREATEで展開する開始姿勢は hensuu.py の共通「地面保持姿勢」を使う。
# GAME1の走行中は、catch/liftをここから動かさない。

# GAME1で使うTag番号。番号を変える時は、この欄だけを変更する。
tag_start_primary = 1
tag_start_fallback = 9
tag_gate = 8
tag_board_left = 12
tag_board_right = 13
tag_return_left = 6
tag_return_right = 10
tag_goal = 0

# メンテナンス画面で表示するGAME1のTag一覧。
game1_tag_ids = (
    tag_start_primary, tag_start_fallback, tag_gate,
    tag_board_left, tag_board_right,
    tag_return_left, tag_return_right, tag_goal,
)

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
