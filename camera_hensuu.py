"""RDK Stereo Camera Module用の調整値。

まず maintenance.py の映像を見て左右の番号を確認する。焦点距離は
チェスボード校正後に入力する。0.0の間は距離自動移動を安全のため行わない。
"""

left_camera_device = 0
right_camera_device = 1
apriltag_size_m = 0.180
camera_focal_length_px = 0.0
tag_max_age_sec = 0.35
