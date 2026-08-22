"""競技用AprilTagで魚眼カメラの歪みを校正する。

実行:
  python3 calibrate_fisheye.py

180 mmの平らなAprilTagを使う。Tagをカメラの中央・四隅・近距離・遠距離へ
ゆっくり動かし、正面だけでなく傾けた画像も集めると、30枚の画像を自動保存する。
完了したら fisheye_calibration.npz を作る。
"""

from __future__ import annotations

import time

import camera_hensuu
from rox_mecanum import open_camera


# 使用する競技用AprilTagの番号。印刷済みのTag 0でよい。
TAG_ID = 0
REQUIRED_SAMPLES = 30
CAPTURE_INTERVAL_SEC = 0.7


def main() -> None:
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("校正にはOpenCVとnumpyが必要です") from error

    camera = open_camera(
        backend=camera_hensuu.camera_backend, device=camera_hensuu.camera_device,
        pipe_id=camera_hensuu.mipi_pipe_id, host_index=camera_hensuu.mipi_host_index,
        fps=camera_hensuu.mipi_fps, width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    tag_size = camera_hensuu.apriltag_size_m
    # Tagの4隅の実寸座標[m]。検出器の角順に合わせて左上から時計回り。
    template = np.array([[
        [0.0, 0.0, 0.0],
        [tag_size, 0.0, 0.0],
        [tag_size, tag_size, 0.0],
        [0.0, tag_size, 0.0],
    ]], dtype=np.float64)
    object_points: list[object] = []
    image_points: list[object] = []
    image_size: tuple[int, int] | None = None
    last_capture = 0.0
    print(f"Tag {TAG_ID} を画面の中央・四隅・近距離・遠距離へゆっくり動かしてください。")
    print("正面だけでなく左右・上下へ傾けてください。Ctrl+Cで中止します。")
    try:
        while len(object_points) < REQUIRED_SAMPLES:
            image = camera.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _rejected = detector.detectMarkers(gray)
            now = time.monotonic()
            if ids is not None and now - last_capture >= CAPTURE_INTERVAL_SEC:
                for corner, raw_id in zip(corners, ids.flatten()):
                    if int(raw_id) != TAG_ID:
                        continue
                    refined = cv2.cornerSubPix(
                        gray, corner.reshape(-1, 1, 2), (11, 11), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                    )
                    object_points.append(template.copy())
                    image_points.append(refined.reshape(1, -1, 2))
                    image_size = gray.shape[::-1]
                    last_capture = now
                    print(f"保存 {len(object_points)}/{REQUIRED_SAMPLES}")
                    break
            time.sleep(0.01)
    finally:
        camera.close()

    if image_size is None:
        raise RuntimeError(f"Tag {TAG_ID} を認識できませんでした。Tag種類・明るさ・印刷サイズを確認してください")
    matrix = np.zeros((3, 3))
    distortion = np.zeros((4, 1))
    # RDK搭載版の古いOpenCVでは定数名を公開しない場合がある。
    # OpenCV fisheye APIで共通のビット値を予備値として使う。
    recompute_extrinsic = getattr(cv2.fisheye, "CALIB_RECOMPUTE_EXTRINSIC", 2)
    fix_skew = getattr(cv2.fisheye, "CALIB_FIX_SKEW", 8)
    # 4隅だけのAprilTagでは CALIB_CHECK_COND が過敏に失敗するため使わない。
    flags = recompute_extrinsic | fix_skew
    try:
        error, matrix, distortion, _rvecs, _tvecs = cv2.fisheye.calibrate(
            object_points, image_points, image_size, matrix, distortion,
            None, None, flags, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
    except cv2.error as calibration_error:
        raise RuntimeError("魚眼校正に失敗しました。Tagを中央・四隅・近距離・遠距離・傾きで撮り直してください") from calibration_error
    np.savez(
        camera_hensuu.fisheye_calibration_file,
        camera_matrix=matrix,
        distortion=distortion,
        image_size=np.array(image_size),
        reprojection_error=error,
    )
    print(f"保存しました: {camera_hensuu.fisheye_calibration_file}")
    print(f"再投影誤差: {error:.4f}（小さいほど良い）")
    print("camera_hensuu.py の fisheye_enabled = True に変更してください")


if __name__ == "__main__":
    main()
