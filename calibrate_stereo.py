"""RDK Stereo Camera Moduleのチェスボード校正。

使い方:
  python3 calibrate_stereo.py

左右カメラへチェスボードを同時に映し、SPACEを20回程度押す。
ESCで計算して ``stereo_calibration.npz`` を保存する。
表示された ``left_focal_length_px`` を camera_hensuu.py へ転記する。
"""

from __future__ import annotations

import numpy as np

import camera_hensuu
from rox_mecanum import open_stereo_camera


# チェスボードの「内側の角」の数。使う実物に合わせて変更する。
CHESSBOARD_CORNERS = (9, 6)
# 1マスの実寸[m]。例: 25mmなら0.025。
SQUARE_SIZE_M = 0.025
REQUIRED_SAMPLES = 20


def main() -> None:
    import cv2

    camera = open_stereo_camera(
        backend=camera_hensuu.camera_backend, left_device=camera_hensuu.left_camera_device,
        right_device=camera_hensuu.right_camera_device, left_index=camera_hensuu.left_mipi_camera_index,
        right_index=camera_hensuu.right_mipi_camera_index, fps=camera_hensuu.mipi_fps,
        width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
    )
    object_template = np.zeros((CHESSBOARD_CORNERS[0] * CHESSBOARD_CORNERS[1], 3), np.float32)
    object_template[:, :2] = np.mgrid[0:CHESSBOARD_CORNERS[0], 0:CHESSBOARD_CORNERS[1]].T.reshape(-1, 2)
    object_template *= SQUARE_SIZE_M
    object_points: list[object] = []
    left_points: list[object] = []
    right_points: list[object] = []
    image_size = None

    try:
        while True:
            left, right = camera.read()
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            left_ok, left_corner = cv2.findChessboardCorners(left_gray, CHESSBOARD_CORNERS)
            right_ok, right_corner = cv2.findChessboardCorners(right_gray, CHESSBOARD_CORNERS)
            preview = left.copy()
            cv2.putText(preview, f"samples: {len(object_points)}/{REQUIRED_SAMPLES}  SPACE=save ESC=finish", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if left_ok:
                cv2.drawChessboardCorners(preview, CHESSBOARD_CORNERS, left_corner, left_ok)
            cv2.imshow("ROX stereo calibration (left)", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord(" ") and left_ok and right_ok:
                left_corner = cv2.cornerSubPix(left_gray, left_corner, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                right_corner = cv2.cornerSubPix(right_gray, right_corner, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                object_points.append(object_template.copy())
                left_points.append(left_corner)
                right_points.append(right_corner)
                image_size = left_gray.shape[::-1]
                print(f"保存: {len(object_points)}枚")

        if len(object_points) < REQUIRED_SAMPLES or image_size is None:
            raise RuntimeError(f"校正には{REQUIRED_SAMPLES}枚以上必要です。現在 {len(object_points)}枚")
        _left_error, left_matrix, left_distortion, _rvec, _tvec = cv2.calibrateCamera(object_points, left_points, image_size, None, None)
        _right_error, right_matrix, right_distortion, _rvec, _tvec = cv2.calibrateCamera(object_points, right_points, image_size, None, None)
        _error, left_matrix, left_distortion, right_matrix, right_distortion, rotation, translation, _essential, _fundamental = cv2.stereoCalibrate(
            object_points, left_points, right_points, left_matrix, left_distortion, right_matrix, right_distortion, image_size,
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
        np.savez("stereo_calibration.npz", left_matrix=left_matrix, left_distortion=left_distortion, right_matrix=right_matrix, right_distortion=right_distortion, rotation=rotation, translation=translation)
        print("stereo_calibration.npz を保存しました")
        print(f"camera_hensuu.py の camera_focal_length_px = {left_matrix[0, 0]:.2f} に変更してください")
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
