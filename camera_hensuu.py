"""RDK Stereo Camera Module用の調整値。

まず maintenance.py の映像を見て左右の番号を確認する。焦点距離は
チェスボード校正後に入力する。0.0の間は距離自動移動を安全のため行わない。
"""

# RDK公式MIPIカメラは /dev/video* ではなくhobot_vioで読む。
# USB/V4L2カメラを使う時だけ "v4l2" に変更する。
camera_backend = "rdk_mipi"
left_camera_device = 0
right_camera_device = 1
# RDK X5の ``open_cam(pipe_id, video_index, ...)`` 用の値。
# pipe はライブラリ内で使う処理レーン。host=-1 はRDKに接続済みセンサーを
# 自動選択させる安全な既定値。物理MIPI番号をここへ書かないこと。
left_mipi_pipe_id = 0
left_mipi_host_index = -1
right_mipi_pipe_id = 1
right_mipi_host_index = -1
mipi_fps = 30
mipi_width = 1920
mipi_height = 1080
apriltag_size_m = 0.180
camera_focal_length_px = 0.0
tag_max_age_sec = 0.35
