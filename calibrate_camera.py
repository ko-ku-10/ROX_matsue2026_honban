"""単眼カメラ用のAprilTag距離校正。

実行:
  python3 calibrate_camera.py 1.00

180 mmの公式AprilTagをカメラ正面ちょうど1.00 mに置く。表示された
``camera_focal_length_px`` を camera_hensuu.py へ転記する。
"""

from __future__ import annotations

import argparse
import statistics
import time

import camera_hensuu
from rox_mecanum import AprilTagDetector, open_camera


def main() -> None:
    parser = argparse.ArgumentParser(description="AprilTag実寸から単眼距離を校正する")
    parser.add_argument("distance_m", type=float, help="Tag正面までを測った距離[m]。例: 1.00")
    parser.add_argument("--tag-id", type=int, default=0, help="映すTag番号。既定: 0")
    args = parser.parse_args()
    if args.distance_m <= 0.0:
        raise ValueError("distance_m は0より大きくしてください")

    camera = open_camera(
        backend=camera_hensuu.camera_backend, device=camera_hensuu.camera_device,
        pipe_id=camera_hensuu.mipi_pipe_id, host_index=camera_hensuu.mipi_host_index,
        fps=camera_hensuu.mipi_fps, width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
        fisheye_calibration_file=camera_hensuu.fisheye_calibration_file if camera_hensuu.fisheye_enabled else None,
        fisheye_balance=camera_hensuu.fisheye_balance,
    )
    detector = AprilTagDetector(camera_hensuu.apriltag_size_m)
    values: list[float] = []
    print(f"Tag {args.tag_id} を正面 {args.distance_m:.3f}m に固定してください。20回検出で完了します。")
    try:
        while len(values) < 20:
            image = camera.read()
            for observation in detector.detect(image):
                if observation.tag_id == args.tag_id and observation.pixel_size is not None:
                    # focal = 画像上のTag辺長[pixel] × 実測距離[m] ÷ Tag実寸[m]
                    values.append(observation.pixel_size * args.distance_m / camera_hensuu.apriltag_size_m)
                    print(f"取得 {len(values)}/20")
                    break
            time.sleep(0.03)
        print(f"camera_hensuu.py: camera_focal_length_px = {statistics.median(values):.2f}")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
