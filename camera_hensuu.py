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
camera_focal_length_px = 729.30
tag_max_age_sec = 0.35

# カメラ中心がロボット中心から横にずれている量[m]。
# 右へ付いているなら正、左へ付いているなら負。最初は 0.0 のままにし、
# 実測した取付け位置（例: 右へ5cmなら +0.05）を入力する。
# Tagの距離に応じて画像上の中心位置を補正し、ロボット中心がTagへ向くようにする。
camera_lateral_offset_m = 0.0
