"""GAME2専用の実機調整値。"""

auto_speed = 0.20
center_gain = 0.45
center_tolerance = 0.08
distance_tolerance_m = 0.08

# 地面に置いた時と発射時のlift角度。実機で設定する。
lift_ground_angle = 0.0
lift_fire_angle = 0.0
lift_target_timeout_sec = 8.0

# 後退は時間式。現地で安全な値を設定する。
retreat_speed = 0.20
retreat_sec = 0.0

# Tag IDを「上・中央・下」の順に並べる。各行の左→右順。
panel_rows = {
    "top": (14, 15, 16),
    "middle": (17, 18, 19),
    "bottom": (20, 21, 22),
}

# 発射時に、カメラから各段のTagまで残す距離[m]。
# 高さを変えられないので、各段で実射して「最も当たる距離」を入力する。
# 初期値は仮の同一値。上・中央・下を測定後に個別に変更する。
shot_distance_m = {
    "top": 1.20,
    "middle": 1.20,
    "bottom": 1.20,
}

# メンテナンス画面や診断で使うGAME2のTag一覧。
game2_tag_ids = tuple(tag_id for row in panel_rows.values() for tag_id in row)
