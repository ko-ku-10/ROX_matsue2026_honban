"""RDK Stereo Camera Module用の調整値。

まず maintenance.py の映像を見て左右の番号を確認する。焦点距離は
チェスボード校正後に入力する。0.0の間は距離自動移動を安全のため行わない。
"""

# RDK公式MIPIカメラは /dev/video* ではなくhobot_vioで読む。
# USB/V4L2カメラを使う時だけ "v4l2" に変更する。
camera_backend = "rdk_mipi"
left_camera_device = 0
right_camera_device = 1
left_mipi_camera_index = 0
right_mipi_camera_index = 1
mipi_fps = 30
mipi_width = 1920
mipi_height = 1080
apriltag_size_m = 0.180
camera_focal_length_px = 0.0
tag_max_age_sec = 0.35
