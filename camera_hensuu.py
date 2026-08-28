"""本番用の単眼カメラ調整値。

まず ``python3 -m experiments.maintenance --camera-only`` の映像を見てTagを検出できることを確認する。焦点距離は
Tag実寸180 mmを使う単眼校正後に入力する。0.0の間は距離自動移動を安全のため行わない。
"""

# RDK公式MIPIカメラは /dev/video* ではなくhobot_vioで読む。
# USB/V4L2カメラを使う時だけ "v4l2" に変更する。
camera_backend = "rdk_mipi"
camera_device = 0
# RDK X5の ``open_cam(pipe_id, video_index, ...)`` 用の値。
# host=-1 はRDKに接続済みセンサーを
# 自動選択させる安全な既定値。物理MIPI番号をここへ書かないこと。
mipi_pipe_id = 0
mipi_host_index = -1
mipi_fps = 30
mipi_width = 1920
mipi_height = 1080
apriltag_size_m = 0.180
camera_focal_length_px = 619
tag_max_age_sec = 0.35

# カメラ中心がロボット中心から横にずれている量[m]。
# 右へ付いているなら正、左へ付いているなら負。最初は 0.0 のままにし、
# 実測した取付け位置（例: 右へ5cmなら +0.05）を入力する。
# Tagの距離に応じて画像上の中心位置を補正し、ロボット中心がTagへ向くようにする。
camera_lateral_offset_m = 0.0

# カメラがロボットの真正面からどれだけ回転して付いているか[度]。
# ロボットをTagへ真正面に置いた時に、サイトの「Tag角度」が +3.0°なら、
# ここへ -3.0 を入れる。補正後の角度0°をロボット真正面として扱う。
camera_yaw_offset_deg = 0.0

# ==================================================
# Tagを正面から見るための共通設定（GAME1・GAME2共通）
# ==================================================
# まずTagを画像・ロボットの中央へ寄せる許容値。左=-1、右=+1。
tag_center_tolerance = 0.03
# Tagがこの範囲にある時だけ、Tag面の角度(yaw)を信用する。
# 魚眼カメラの端では角度が大きく狂いやすいため、端のTagは発見用だけにする。
tag_yaw_trust_center_error = 0.12
# 中央へ向く旋回の設定。
tag_rotate_gain = 0.60
tag_rotate_max_speed = 0.20
tag_center_stable_sec = 0.30
# Tag面がロボット正面と平行とみなす角度。
tag_yaw_tolerance_deg = 1.0
tag_yaw_gain = 0.020
# 実機で角度合わせだけが逆回転なら -1.0 に変える。
tag_yaw_direction = 1.0
